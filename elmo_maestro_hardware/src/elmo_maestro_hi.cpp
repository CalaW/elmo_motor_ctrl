#include "elmo_maestro_hardware/elmo_maestro_hi.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace elmo_maestro_hardware
{

ElmoMaestroHardwareInterface::~ElmoMaestroHardwareInterface()
{
  on_deactivate(rclcpp_lifecycle::State());
}

// initialize all variables and process parameters
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#pragma push_macro("ERROR")
#undef ERROR
    return CallbackReturn::ERROR;
#pragma pop_macro("ERROR")
  }
  host_ip_ = "192.168.1.2";
  target_ip_ = "192.168.1.3";

  hw_joint_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_joint_command_ = std::numeric_limits<double>::quiet_NaN();
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_init done");
  return CallbackReturn::SUCCESS;
}

// setup communication with the hardware
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    elmoInit(host_ip_, target_ip_);

    axes_[0].m_fAcceleration = 1000;
    axes_[0].m_fDeceleration = 10000;
    axes_[0].m_fVelocity = 1000;
  } catch (runtime_error & e) {
#pragma push_macro("ERROR")
#undef ERROR
    RCLCPP_ERROR(
      rclcpp::get_logger("ElmoMaestroHardwareInterface"),
      "Failed to initialize elmo_maestro_hardware: %s", e.what());
    return CallbackReturn::ERROR;
#pragma pop_macro("ERROR")
  }
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_configure done");
  return CallbackReturn::SUCCESS;
}

// cleanup communication with the hardware
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_cleanup done");
  return CallbackReturn::SUCCESS;
}

// define state resources
std::vector<hardware_interface::StateInterface>
ElmoMaestroHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_state_));

  return state_interfaces;
}

// define command resources
std::vector<hardware_interface::CommandInterface>
ElmoMaestroHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_command_));

  return command_interfaces;
}

// enable "power"
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("ElmoMaestroHardwareInterface"),
    "status before power on: 0x" << std::hex << axes_[0].ReadStatus());
  auto status = axes_[0].ReadStatus();
  if (status & NC_AXIS_DISABLED_MASK) {
    axes_[0].PowerOn();
    while (!(axes_[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
      ;
  }
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("ElmoMaestroHardwareInterface"),
    "status after power on: 0x" << std::hex << axes_[0].ReadStatus());
  // set some default values for joints
  if (std::isnan(hw_joint_state_) || std::isnan(hw_joint_command_)) {
    hw_joint_state_ = 0;
    hw_joint_command_ = 0;
  }
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_activate done");
  return CallbackReturn::SUCCESS;
}

// stop all movement
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & axis : axes_) {
    axis.Stop();
  }
  for (auto & axis : axes_) {
    while (!(axis.ReadStatus() & NC_AXIS_STAND_STILL_MASK))
      ;
    axis.PowerOff();
  }
  closeConnection();
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_deactivate done");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_shutdown done");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(rclcpp::get_logger("ElmoMaestroHardwareInterface"), "on_error done");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ElmoMaestroHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hw_joint_state_ = axes_[0].GetActualVelocity();
  // RCLCPP_INFO(
  //   rclcpp::get_logger("ElmoMaestroHardwareInterface"), "read %.5f for joint %s", hw_joint_state_,
  //   info_.joints[0].name.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ElmoMaestroHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // RCLCPP_INFO(
  //   rclcpp::get_logger("ElmoMaestroHardwareInterface"), "got command %.5f for joint %s",
  //   hw_joint_command_, info_.joints[0].name.c_str());

  // uint: cnt/sec ------------- 24cnt/r
  axes_[0].MoveVelocity(hw_joint_command_, MC_ABORTING_MODE);
  return hardware_interface::return_type::OK;
}

/**
 * @brief Initialize the system, including axes, communication, etc.
 * 
 * @return int 
 */
void ElmoMaestroHardwareInterface::elmoInit(
  const std::string & host_ip, const std::string & target_ip)
{
  printf("init connection\n");
  if (host_ip.empty() || target_ip.empty()) {
    throw runtime_error("host_ip or target_ip is empty");
  }
  if (host_ip.length() > 15 || target_ip.length() > 15) {
    throw runtime_error("host_ip or target_ip is too long");
  }
  char host_ip_c[16];
  std::copy(host_ip.begin(), host_ip.end(), host_ip_c);
  char target_ip_c[16];
  std::copy(target_ip.begin(), target_ip.end(), target_ip_c);
  conn_param_.uiTcpPort = 4000;
  std::copy(target_ip.begin(), target_ip.end(), conn_param_.ucIp);

  conn_hndl_ = conn_.ConnectRPCEx(host_ip_c, target_ip_c, 0x7fffffff, nullptr);

  // Register the callback function for Emergency:
  conn_.RegisterEventCallback(MMCPP_EMCY, nullptr);

  // Set Try-Catch flag Enable\Disable
  CMMCPPGlobal::Instance()->SetThrowFlag(false);
  CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

  try {
    axes_[0].InitAxisData("a01", conn_hndl_);

    for (auto & axis : axes_) {
      auto status = axis.ReadStatus();

      if (status & NC_AXIS_ERROR_STOP_MASK) {
        std::cout << std::hex << status << " " << std::hex << NC_AXIS_STOPPING_MASK << std::endl;
        axis.ResetAsync();
        status = axis.ReadStatus();
        if (status & NC_AXIS_ERROR_STOP_MASK) {
          throw runtime_error(to_string(status));
        }
      }
    }
  } catch (CMMCException exp) {
    cout << "init failed!!" << exp.what() << std::endl;
    exit(1);
  }
  std::cout << std::endl;
}

/**
 * @brief Call back function if emergency received
 * 
 * @param usAxisRef 
 * @param sEmcyCode 
 */
void ElmoMaestroHardwareInterface::onEmergency(unsigned short usAxisRef, short sEmcyCode)
{
  printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);
}

int ElmoMaestroHardwareInterface::closeConnection()
{
  int retval;
  printf("close connection\n");

  retval = MMC_CloseConnection(conn_hndl_);

  if (retval != 0) {
    printf("ERROR CloseConnection: MMC_CloseConnection fail %d\n", retval);
    return -1;
  }

  return 0;
}

}  // namespace elmo_maestro_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  elmo_maestro_hardware::ElmoMaestroHardwareInterface, hardware_interface::SystemInterface)