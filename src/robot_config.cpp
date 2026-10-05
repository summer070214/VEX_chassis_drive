#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

motor right_chassis_1 = motor(PORT17, ratio6_1, false);
motor right_chassis_2 = motor(PORT18, ratio6_1, true);
motor right_chassis_3 = motor(PORT19, ratio6_1, false);
motor right_chassis_4 = motor(PORT20, ratio6_1, true);

motor left_chassis_1 = motor(PORT6, ratio6_1, true);
motor left_chassis_2 = motor(PORT7, ratio6_1, false);
motor left_chassis_3 = motor(PORT8 , ratio6_1, true);
motor left_chassis_4 = motor(PORT9 , ratio6_1, false);//底盘

rotation RotationX = rotation(PORT1, true);
rotation RotationY = rotation(PORT2, true);//定位轮

controller Controller1 = controller(primary);//手柄

inertial Inertial(PORT11);