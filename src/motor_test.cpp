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
#include "OS_PlatformDependSetting.h"
#include "OS_PlatformDataTypeSizeSet.h"
#include "OS_PlatformLinuxRpc64.hpp"
#include "MMC_definitions.h"
#include "MMCPPlib.hpp"
#include "win32_motion.h"
#include <iostream>
#include <stdint.h>
#include <sys/time.h>  // For time structure
#include <csignal>     // For Timer mechanism

// function prototypes
void MainInit();
int CloseConnection();
void StopMotor_cb(int signum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);

/**
 * @brief the main function of this project
 * 
 * @return int 
 */
int main()
{
  MainInit();

  try {
    // enable axis and wait till axis enable is done
    std::cout << "status before power on: 0x" << std::hex << cAxis[0].ReadStatus() << std::endl;
    auto status = cAxis[0].ReadStatus();
    if (status & NC_AXIS_DISABLED_MASK) {
      cAxis[0].PowerOn();
      while (!(cAxis[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
        ;
    }
    std::cout << "status after power on: 0x" << std::hex << cAxis[0].ReadStatus() << std::endl;

    std::signal(SIGINT, StopMotor_cb);

    cAxis[0].MoveVelocity(1000);  // uint: cnt/sec ------------- 24cnt/r
    // cAxis[0].MoveAbsolute(500);
    sleep(10);
    // stop axis wait till its in a 'standstill' state
    cAxis[0].Stop();
    while (!(cAxis[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
      ;

    // disable axis
    cAxis[0].PowerOff();

  } catch (CMMCException exp) {
    cout << "main function exception!!" << exp.what() << "error" << exp.error() << endl;
  }

  CloseConnection();
  return 0;
}

/**
 * @brief Initialize the system, including axes, communication, etc.
 * 
 * @return int 
 */
void MainInit()
{
  printf("init connection\n");

  // this is not safe but have to do this
  auto host_ip = (char *)"192.168.1.2";
  auto target_ip = (char *)"192.168.1.3";

  conn_param.uiTcpPort = 4000;
  strcpy((char *)conn_param.ucIp, target_ip);

  g_conn_hndl = cConn.ConnectRPCEx(host_ip, target_ip, 0x7fffffff, NULL);

  // Register the callback function for Emergency:
  cConn.RegisterEventCallback(MMCPP_EMCY, (void *)Emergency_Received);

  // Set Try-Catch flag Enable\Disable
  CMMCPPGlobal::Instance()->SetThrowFlag(true);
  CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

  try {
    cAxis[0].InitAxisData("a01", g_conn_hndl);
    cAxis[1].InitAxisData("a02", g_conn_hndl);

    for (int i = 0; i < MAX_AXES; i++) {
      auto Status = cAxis[i].ReadStatus();

      if (Status & NC_AXIS_ERROR_STOP_MASK) {
        cAxis[i].Reset();
        // sleep 1s to wait for reset
        sleep(1);
        Status = cAxis[i].ReadStatus();
        if (Status & NC_AXIS_ERROR_STOP_MASK) {
          throw runtime_error("MainInit: axis status error");
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
int CloseConnection()
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

void StopMotor_cb(int signum)
{
  std::cout << "caught signal: " << signum << std::endl;
  for (int i = 0; i < MAX_AXES; i++) {
    cAxis[i].Stop();
  }
  exit(1);
}

/**
 * @brief Call back function if emergency received
 * 
 * @param usAxisRef 
 * @param sEmcyCode 
 */
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
  printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);
}
