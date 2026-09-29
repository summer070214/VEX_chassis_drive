#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include"vex.h"
using namespace vex;

extern vex::brain Brain;
extern motor right_chassis_1;
extern motor right_chassis_2;
extern motor right_chassis_3;
extern motor right_chassis_4;
extern motor left_chassis_1;
extern motor left_chassis_2;
extern motor left_chassis_3;
extern motor left_chassis_4;

extern rotation RotationX;
extern rotation RotationY;

extern controller Controller1;
extern inertial Inertial;
#endif
