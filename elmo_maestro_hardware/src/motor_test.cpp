/**
 * @file motor_test.cpp
 * @author Chen Chen (maker_cc@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-20
 * 
 * 
 */

#define ENDIAN_SELECT 123
#define OS_PLATFORM 777
#include "elmo_maestro_api/OS_PlatformDependSetting.h"
#include "elmo_maestro_api/OS_PlatformDataTypeSizeSet.h"
#include "elmo_maestro_api/OS_PlatformLinuxRpc64.hpp"
#include "elmo_maestro_api/MMC_definitions.h"
#include "elmo_maestro_api/MMCPPlib.hpp"
#include "elmo_maestro_hardware/motor_test.h"
#include <iostream>
#include <stdint.h>
#include <sys/time.h>  // For time structure
#include <csignal>     // For Timer mechanism

// function prototypes
void mainInit();
int closeConnection();
void stopMotorCb(int signum);
void emergencyReceived(unsigned short usAxisRef, short sEmcyCode);

/**
 * @brief the main function of this project
 * 
 * @return int 
 */
int main()
{
  mainInit();

  try {
    // enable axis and wait till axis enable is done
    std::cout << "status before power on: 0x" << std::hex << axes[0].ReadStatus() << std::endl;
    auto status = axes[0].ReadStatus();
    if (status & NC_AXIS_DISABLED_MASK) {
      axes[0].PowerOn();
      while (!(axes[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
        ;
    }
    std::cout << "status after power on: 0x" << std::hex << axes[0].ReadStatus() << std::endl;

    std::signal(SIGINT, stopMotorCb);

    axes[0].m_fAcceleration = 1000;
    axes[0].m_fDeceleration = 10000;
    axes[0].m_fVelocity = 1000;

    axes[0].MoveVelocity(1000);  // uint: cnt/sec ------------- 24cnt/r
    sleep(10);
    // stop axis wait till its in a 'standstill' state
    axes[0].Stop();
    while (!(axes[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
      ;

    // disable axis
    axes[0].PowerOff();

  } catch (CMMCException exp) {
    cout << "main function exception!!" << exp.what() << "error" << exp.error() << endl;
  }

  closeConnection();
  return 0;
}

/**
 * @brief Initialize the system, including axes, communication, etc.
 * 
 * @return int 
 */
void mainInit()
{
  printf("init connection\n");

  // this is not safe but have to do this
  auto host_ip = (char *)"192.168.1.2";
  auto target_ip = (char *)"192.168.1.3";

  conn_param.uiTcpPort = 4000;
  strcpy((char *)conn_param.ucIp, target_ip);

  g_conn_hndl = cConn.ConnectRPCEx(host_ip, target_ip, 0x7fffffff, NULL);

  // Register the callback function for Emergency:
  cConn.RegisterEventCallback(MMCPP_EMCY, (void *)emergencyReceived);

  // Set Try-Catch flag Enable\Disable
  CMMCPPGlobal::Instance()->SetThrowFlag(false);
  CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

  try {
    axes[0].InitAxisData("a01", g_conn_hndl);

    for (auto & axis : axes) {
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
    cout << "init failed!!" << exp.what() << endl;
    exit(1);
  }
  std::cout << std::endl;
}

/**
 * @brief Close connection
 * 
 * @return int 
 */
int closeConnection()
{
  int retval;
  printf("close connection\n");

  retval = MMC_CloseConnection(g_conn_hndl);

  if (retval != 0) {
    printf("ERROR CloseConnection: MMC_CloseConnection fail %d\n", retval);
    return -1;
  }

  return 0;
}

void stopMotorCb(int signum)
{
  std::cout << "caught signal: " << signum << std::endl;
  for (auto & axis : axes) {
    axis.Stop();
  }
  exit(1);
}

/**
 * @brief Call back function if emergency received
 * 
 * @param usAxisRef 
 * @param sEmcyCode 
 */
void emergencyReceived(unsigned short usAxisRef, short sEmcyCode)
{
  printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);
}
