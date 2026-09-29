#ifndef SENSOR_READ_H
#define SENSOR_READ_H
#include "vex.h"

using namespace vex;

//----------------------传感器数据采集------------------------//
void  Init_Rotation_Filters();

float Inertial_Heading_Get();
float Inertial_Rotation_Get();
float Rotation_X_Get();
float Rotation_Y_Get();

//--------------------传感器数据处理---------------------//
void Pos_yaw_process();
void Pos_inertial_process();
void Pos_rotation_process();
void Update_single_motor_rpm();

void data_process();
#endif