
#include"controller.h"
#include"moto_control.h"
#include"vex.h"
#include"robot_config.h"
using namespace vex;

Controller_Data controller_data;
void controller_data_read(void){
    controller_data.Ch1 = Controller1.Axis1.value();
    controller_data.Ch3 = Controller1.Axis3.value();
    controller_data.Ch4 = Controller1.Axis4.value();
    controller_data.BtnX = Controller1.ButtonX.pressing();
    controller_data.BtnY = Controller1.ButtonY.pressing();
    controller_data.BtnUp = Controller1.ButtonUp.pressing();
    controller_data.BtnDown = Controller1.ButtonDown.pressing();
    controller_data.BtnL = Controller1.ButtonLeft.pressing();
    controller_data.BtnR = Controller1.ButtonRight.pressing();
    controller_data.LL1 = Controller1.ButtonX.pressing();
    controller_data.LL2 = Controller1.ButtonY.pressing();
    }

void handleDriveControl() {
    const int DEADZONE = 10;  // 摇杆死区，10%以内认为没动
    controller_data_read();
    // 左摇杆上下：Axis3
    int forward = controller_data.Ch3;

    // 右摇杆左右：Axis1
    int turn = controller_data.Ch1;

    // 死区处理
    if (forward > -DEADZONE && forward < DEADZONE) {
        forward = 0;
    }
    if (turn > -DEADZONE && turn < DEADZONE) {
        turn = 0;
    }

    // 左摇杆控制前进后退
    if (forward > 0) {
        car_forword(forward);      // 上推前进
    } else if (forward < 0) {
        car_backword(-forward);    // 下推后退，取正值
    }

    // 右摇杆控制左转右转
    if (turn < 0) {
        car_left(-turn);           // 左推左转，取正值
    } else if (turn > 0) {
        car_right(turn);           // 右推右转
    }

    // 前后和转向都没输入时停止
    if (forward == 0 && turn == 0) {
        car_stop();
    }

    // 十字键左/右：触发定点转角（上升沿，只触发一次）
    static bool lastLeft  = false;
    static bool lastRight = false;

    bool currLeft  = controller_data.BtnL;
    bool currRight = controller_data.BtnR;

    if (currLeft && !lastLeft) {
        car_left_angle();
    }
    if (currRight && !lastRight) {
        car_right_angle();
    }

    lastLeft  = currLeft;
    lastRight = currRight;
}