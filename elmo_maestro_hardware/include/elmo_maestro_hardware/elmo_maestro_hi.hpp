#ifndef ELMO_MAESTRO_HARDWARE__ELMO_MAESTRO_HI_HPP_
#define ELMO_MAESTRO_HARDWARE__ELMO_MAESTRO_HI_HPP_

#include <array>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "elmo_maestro_hardware/visibility_control.h"

#define ENDIAN_SELECT 123
#define OS_PLATFORM 777
#include "elmo_maestro_api/OS_PlatformDependSetting.h"
#include "elmo_maestro_api/OS_PlatformDataTypeSizeSet.h"
#include "elmo_maestro_api/OS_PlatformLinuxRpc64.hpp"
#include "elmo_maestro_api/MMC_definitions.h"
#include "elmo_maestro_api/MMCPPlib.hpp"

namespace elmo_maestro_hardware
{
class ElmoMaestroHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElmoMaestroHardwareInterface)

   ~ElmoMaestroHardwareInterface() override;

  // initialize all variables and process parameters
  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // setup communication with the hardware
  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // cleanup communication with the hardware
  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // define state resources
  ELMO_MAESTRO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // define command resources
  ELMO_MAESTRO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // enable "power"
  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // stop all movement
  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  //
  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ELMO_MAESTRO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::array<CMMCSingleAxis, 1> axes_;
  double hw_joint_command_;
  double hw_joint_state_;
  std::string host_ip_;
  std::string target_ip_;
  MMC_CONNECTION_PARAM_STRUCT conn_param_;
  MMC_CONNECT_HNDL conn_hndl_;
  CMMCConnection conn_;
  void elmoInit(const std::string & host_ip, const std::string & target_ip);
  void onEmergency(unsigned short usAxisRef, short sEmcyCode);
  int closeConnection();
};
}  // namespace elmo_maestro_hardware

#endif  // ELMO_MAESTRO_HARDWARE_ELMO_MAESTRO_HW_HPP_