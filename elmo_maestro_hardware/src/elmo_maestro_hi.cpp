#include "elmo_maestro_hardware/elmo_maestro_hi.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace elmo_maestro_hardware
{

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
}

// setup communication with the hardware
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

// cleanup communication with the hardware
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

// define state resources
std::vector<hardware_interface::StateInterface>
ElmoMaestroHardwareInterface::export_state_interfaces()
{
}

// define command resources
std::vector<hardware_interface::CommandInterface>
ElmoMaestroHardwareInterface::export_command_interfaces()
{
}

// enable "power"
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

// stop all movement
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

//
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

ELMO_MAESTRO_HARDWARE_PUBLIC
hardware_interface::CallbackReturn ElmoMaestroHardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

ELMO_MAESTRO_HARDWARE_PUBLIC
hardware_interface::return_type ElmoMaestroHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

ELMO_MAESTRO_HARDWARE_PUBLIC
hardware_interface::return_type ElmoMaestroHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

/**
 * @brief Initialize the system, including axes, communication, etc.
 * 
 * @return int 
 */
void ElmoMaestroHardwareInterface::elmoInit()
{
  printf("init connection\n");

  // this is not safe but have to do this
  auto host_ip = (char *)"192.168.1.2";
  auto target_ip = (char *)"192.168.1.3";

  conn_param_.uiTcpPort = 4000;
  strcpy((char *)conn_param_.ucIp, target_ip);

  conn_hndl_ = conn_.ConnectRPCEx(host_ip, target_ip, 0x7fffffff, nullptr);

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

}  // namespace elmo_maestro_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  elmo_maestro_hardware::ElmoMaestroHardwareInterface, hardware_interface::SystemInterface)