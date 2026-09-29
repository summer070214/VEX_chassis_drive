#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

motor right_chassis_1 = motor(PORT1, ratio6_1, false);
motor right_chassis_2 = motor(PORT2, ratio6_1, true);
motor right_chassis_3 = motor(PORT3, ratio6_1, false);
motor right_chassis_4 = motor(PORT4, ratio6_1, true);

motor left_chassis_1 = motor(PORT5, ratio6_1, true);
motor left_chassis_2 = motor(PORT6, ratio6_1, false);
motor left_chassis_3 = motor(PORT7 , ratio6_1, true);
motor left_chassis_4 = motor(PORT8 , ratio6_1, false);//底盘

rotation RotationX = rotation(PORT10, true);
rotation RotationY = rotation(PORT9, true);//定位轮

controller Controller1 = controller(primary);//手柄

inertial Inertial(PORT11);