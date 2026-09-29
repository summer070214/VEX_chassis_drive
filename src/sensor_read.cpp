#include "sensor_read.h"
#include "robot_config.h"
#include "D:\DESKTOP\vex\Chassis_drive\APP\kalman\kalman.h"
#include "locate.h"
#include "global_data.h"
#include "stdio.h"
#include "moto_control.h"
//----------------------传感器数据采集------------------------//

// void Init_Rotation_Filters() {
//     // 读取初始角度
//     float initX = RotationX.position(deg);
//     float initY = RotationY.position(deg);

//     // Kalman_Init(&kalmanX, 0.001, 0.05, initX);  // X轴滤波器
//     // Kalman_Init(&kalmanY, 0.001, 0.05, initY);  // Y轴滤波器
// }

float Inertial_Heading_Get(){
    float heading_angle;
    heading_angle=Inertial.heading();
    heading_angle=static_cast<int>(heading_angle*10)/10.0;
    return heading_angle;
}
float Inertial_Rotation_Get(){
    float inertial_angle;
    inertial_angle=Inertial.rotation();
    inertial_angle=static_cast<int>(inertial_angle*10)/10.0;
    //robot_data[9].Value = inertial_angle;
    return inertial_angle;
}
float Rotation_X_Get(){
    float rotation_x_angle;
    rotation_x_angle=RotationX.position(deg);
    //rotation_x_angle=Kalman_Update(&kalmanX,rotation_x_angle);
    rotation_x_angle=static_cast<int>(rotation_x_angle*10)/10.0;
    return rotation_x_angle;
}
float Rotation_Y_Get(){
    float rotation_y_angle;
    rotation_y_angle=RotationY.position(deg);
    //rotation_y_angle=Kalman_Update(&kalmanY,rotation_y_angle);
    rotation_y_angle=static_cast<int>(rotation_y_angle*10)/10.0;
    return rotation_y_angle;
}


//--------------------传感器数据处理---------------------//
void Pos_yaw_process(){
    position_data.cur_yaw_angle=Inertial_Heading_Get();
    if(position_data.cur_yaw_angle>=180){
        position_data.cur_yaw_angle=position_data.cur_yaw_angle-360;
    }
    robot_data[8].Value = position_data.cur_yaw_angle;
}
void Pos_inertial_process(){
    position_data.cur_inertial_angle=Inertial_Rotation_Get();
    robot_data[9].Value = position_data.cur_inertial_angle;
    position_data.delta_inertial_angle=position_data.cur_inertial_angle-position_data.last_inertial_angle;//获取惯性角度差
    position_data.last_inertial_angle=position_data.cur_inertial_angle;
    position_data.delta_dis_inertial_angle=position_data.delta_inertial_angle/360*2*3.14;
}
void Pos_rotation_process(){
    float flocur_rotation_angle_x=Rotation_X_Get();
    float flocur_rotation_angle_y=Rotation_Y_Get();

    robot_data[10].Value = flocur_rotation_angle_x;
    robot_data[11].Value = flocur_rotation_angle_y;
    
    
}
void Update_single_motor_rpm(){
    double v_left1=left_chassis_1.velocity(rpm);
    double v_left2=left_chassis_2.velocity(rpm);
    double v_left3=left_chassis_3.velocity(rpm);
    double v_left4=left_chassis_4.velocity(rpm);
    double v_right1=right_chassis_1.velocity(rpm);
    double v_right2=right_chassis_2.velocity(rpm);
    double v_right3=right_chassis_3.velocity(rpm);
    double v_right4=right_chassis_4.velocity(rpm);
    robot_data[0].Value=v_left1;
    robot_data[1].Value=v_left2;
    robot_data[2].Value=v_left3;
    robot_data[3].Value=v_left4;
    robot_data[4].Value=v_right1;
    robot_data[5].Value=v_right2;
    robot_data[6].Value=v_right3;
    robot_data[7].Value=v_right4;
}



void data_process(){
    Pos_yaw_process();
    Pos_inertial_process();
    Pos_rotation_process();
    Update_single_motor_rpm();
}
