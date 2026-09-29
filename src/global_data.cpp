#include"global_data.h"
#include"PID.h"
#include"D:\DESKTOP\vex\Chassis_drive\APP\kalman\kalman.h"
// 1. 先声明外部的变量（假设它们已经在其他 .cpp 里定义了）
float v_left1, v_left2, v_left3, v_left4;
float v_right1, v_right2, v_right3, v_right4;
float cur_yaw_angle, cur_inertial_angle;
float cur_rotation_angle_x, cur_rotation_angle_y;


MENU_para robot_data[] = {
    {"left_1      ", v_left1},
    {"left_2      ", v_left2},
    {"left_3      ", v_left3},
    {"left_4      ", v_left4},
    {"right_1     ", v_right1},
    {"right_2     ", v_right2},
    {"right_3     ", v_right3},
    {"right_4     ", v_right4},
    {"inertial-H  ", cur_yaw_angle},
    {"inertial-R  ", cur_inertial_angle},
    {"rotation-X  ", cur_rotation_angle_x},
    {"rotation-Y  ", cur_rotation_angle_y}
};


