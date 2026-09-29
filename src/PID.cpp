//
// Created by 28715 on 2026/1/24.
//
#include "PID.h"
#include <stdint.h>
PID_Data_t angle_pid_data;
PID_Data_t distance_pid_data;
void pidinit( PID_Data_t *pid,float kp,float ki,float kd,float i_limit,float output_limit) {
    pid->kp=kp;
    pid->ki=ki;
    pid->kd=kd;
    pid->goal=0.0f;
	pid->actual=0.0f;
	pid->error=0.0f;
    pid->last_error=0.0f;
	pid->i_limit=i_limit;
	pid->output_limit=output_limit;
	pid->kp_output=0.0f;
	pid->ki_output=0.0f;
	pid->kd_output=0.0f;
	pid->output=0.0f;
}
void pid_changegoal( PID_Data_t *pid,float goal) {
    pid->goal=goal;
}
float pid_compute( PID_Data_t *pid,float goal,float current) {
	pid_changegoal(pid,goal);
    pid->actual=current;
	pid->error=pid->goal-pid->actual;
    pid->kp_output=pid->kp*pid->error;
    pid->ki_output+=pid->ki*pid->error;
	if(pid->ki_output > pid->i_limit)
		pid->ki_output = pid->i_limit;
	pid->kd_output=pid->kd*(pid->error-pid->last_error);
    pid->output=pid->kp_output+pid->ki_output+pid->kd_output;
	if(pid->output>pid->output_limit)
		pid->output=pid->output_limit;
	pid->last_error=pid->error;

    return pid->output;
}
