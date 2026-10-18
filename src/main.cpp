/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       28715                                                     */
/*    Created:      2026/7/29 14:03:00                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot_config.h"
#include "chassis_contral.h"
#include "global_data.h"
#include "locate_5225A.h"
#include "moto_control.h"
#include "sensor_read.h"
#include "controller.h"
#include "PID.h"
#include "debug_send.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Inertial.calibrate();       // 惯性传感器校准
    while (Inertial.isCalibrating()) {
        wait(50, msec);
    }
  MotorEncoder_Init();  // 初始化电机编码器
  Position_Init   ();      // 初始化定位系统
  pidinit(&Rspeed_pid_data, 0.05f, 0.0f, 0.0f, 10.0f, 12.0f); // 初始化速度 PID
  pidinit(&Lspeed_pid_data, 0.05f, 0.0f, 0.0f, 10.0f, 12.0f); // 初始化速度 PID
  pidinit(&angle_pid_data, 3, 1, 0.1f, 10.0f, 80.0f); // 初始化角度 PID
  pidinit(&distance_pid_data, 0.0f, 0.0f, 0.0f, 10.0f, 100.0f); // 初始化 定位 PID
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
 
  // User control code here, inside the loop 
  data_process();

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    //  controller_data_read();      // ← 填充 controller_data
    //     data_process();              // ← 填充 robot_data[0..7] & [10,11] & [8,9]
    //     UpdateGlobalPosition();      // ← 唯一更新 position_data 的地方
    handleDriveControl(); 
    // UpdateGlobalPosition();
     wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  // Run the pre-autonomous function.
  debug_send_init();      // 启动调试发送任务

  // Prevent main from exiting with an infinite loop.
  while (true) {
      UpdateGlobalPosition();
  }
}
