// debug_send.cpp —— 完整替换版本
#include "vex.h"
#include "global_data.h"
#include "locate_5225A.h"
#include "controller.h"
#include "PID.h"
#include "debug_send.h"

using namespace vex;

extern position_data_t   position_data;
extern ControllerData    controller_data;
extern MENU_para         robot_data[12];

extern PID_Data_t        angle_pid_data;
extern PID_Data_t        distance_pid_data;
extern PID_Data_t        Lspeed_pid_data;
extern PID_Data_t        Rspeed_pid_data;

struct PidRef {
    const char* name;
    PID_Data_t* pid;
};

static const PidRef pids[4] = {
    {"angle",    &angle_pid_data},
    {"distance", &distance_pid_data},
    {"Lspeed",   &Lspeed_pid_data},
    {"Rspeed",   &Rspeed_pid_data},
};

static void debug_send_once(void) {
    printf("POS:%.2f,%.2f,%.2f\n",
           position_data.gxT,
           position_data.gyT,
           position_data.cur_yaw_angle);

    printf("RD:");
    for (int i = 0; i < 12; i++) {
        printf("%.3f", robot_data[i].Value);
        if (i < 11) printf(",");
    }
    printf("\n");

    for (int i = 0; i < 4; i++) {
        const PID_Data_t* p = pids[i].pid;
        printf(
            "PID:%s,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            pids[i].name,
            p->kp, p->ki, p->kd,
            p->goal, p->actual, p->error, p->last_error,
            p->kp_output, p->ki_output, p->kd_output,
            p->output, p->i_limit, p->output_limit);
    }

    printf(
        "RC:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        controller_data.Ch1,     controller_data.Ch2,
        controller_data.Ch3,     controller_data.Ch4,
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

static int debug_send_task(void) {
    while (true) {
        debug_send_once();
        wait(20, msec);
    }
    return 0;
}

void debug_send_init(void) {
    task t(debug_send_task);
}