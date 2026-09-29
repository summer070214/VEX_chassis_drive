//
// Created by 28715 on 2026/1/24.
//

#ifndef PID_H
#define PID_H
typedef struct{
    float kp;             //比例系数
    float ki;             //积分系数
    float kd;             //微分系数

    float goal;           //目标值
    float actual;         //当前值
    float error;
    float last_error;     //误差

    float kp_output;      //比例值
    float ki_output;      //积分值
    float kd_output;      //微分值
    float output;         //总输出

    float i_limit;        //积分值限幅
    float output_limit;   //输出限幅
}PID_Data_t;

void pidinit( PID_Data_t *pid,float kp,float ki,float kd,float i_limit,float output_limit);
void pid_changegoal( PID_Data_t *pid,float goal);
float pid_compute( PID_Data_t *pid,float goal,float current);
extern PID_Data_t angle_pid_data;
extern PID_Data_t distance_pid_data;
#endif

