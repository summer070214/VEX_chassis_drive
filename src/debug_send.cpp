#include "vex.h"
#include "global_data.h"
#include "locate_5225A.h"
#include "controller.h"
#include "PID.h"
#include "debug_send.h"
using namespace vex;

// 外部全局变量
extern position_data_t   position_data;
extern ControllerData    controller_data;
extern MENU_para         robot_data[12];

// 4 个 PID 结构体
extern PID_Data_t        angle_pid_data;
extern PID_Data_t        distance_pid_data;
extern PID_Data_t        Lspeed_pid_data;
extern PID_Data_t        Rspeed_pid_data;

// ---------- robot_data 的 12 个变量 ----------
static void send_robot_data(void) {
    printf("RD:");
    for (int i = 0; i < 12; i++) {
        printf("%.3f", robot_data[i].Value);
        if (i < 11) printf(",");
    }
    printf("\n");
}

// ---------- 一个 PID 结构体 ----------
static void send_pid(const char* name, const PID_Data_t* p) {
    printf("PID:%s,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        name,
        p->kp, p->ki, p->kd,
        p->goal, p->actual, p->error, p->last_error,
        p->kp_output, p->ki_output, p->kd_output,
        p->output, p->i_limit, p->output_limit);
}

// ---------- 全局坐标 ----------
static void send_pos(void) {
    printf("POS:%.2f,%.2f,%.2f\n",
        position_data.gxT,
        position_data.gyT,
        position_data.cur_yaw_angle);
}

// ---------- 遥控数据（16 项）----------
static void send_rc(void) {
    printf("RC:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        controller_data.Ch1,
        controller_data.Ch2,
        controller_data.Ch3,
        controller_data.Ch4,
        controller_data.BtnA     ? 1 : 0,
        controller_data.BtnB     ? 1 : 0,
        controller_data.BtnX     ? 1 : 0,
        controller_data.BtnY     ? 1 : 0,
        controller_data.BtnUp    ? 1 : 0,
        controller_data.BtnDown  ? 1 : 0,
        controller_data.BtnLeft  ? 1 : 0,
        controller_data.BtnRight ? 1 : 0,
        controller_data.L1       ? 1 : 0,
        controller_data.L2       ? 1 : 0,
        controller_data.R1       ? 1 : 0,
        controller_data.R2       ? 1 : 0);
}

// ---------- 发送任务 ----------
static int debug_send_task(void) {
    while (true) {
        send_pos();
        send_robot_data();
        send_pid("angle",    &angle_pid_data);
        send_pid("distance", &distance_pid_data);
        send_pid("Lspeed",   &Lspeed_pid_data);
        send_pid("Rspeed",   &Rspeed_pid_data);
        send_rc();
        wait(20, msec);
    }
    return 0;
}

void debug_send_init(void) {
    task t(debug_send_task);
}