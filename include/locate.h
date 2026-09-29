#ifndef LOCATE_H
#define LOCATE_H

#include "vex.h"
#include <cmath>

// ============================================================
//  机器人几何参数（必须按实车测量后填写！）
// ============================================================
#define CODING_R        1.0f        // 定位轮半径 (cm)
#define TWO_PI          6.2831853f  // 2π

// ---- 定位轮安装角度（与车头 x_b 轴的夹角，逆时针为正，单位：度）----
#define WHEEL1_ANGLE_DEG  0.0f      // 轮1（原 RotationX）安装角 α
#define WHEEL2_ANGLE_DEG  90.0f     // 轮2（原 RotationY）安装角 β

// 追踪点(结构中心) 相对 旋转中心 的偏移 (cm)
#define DELTA_LXA       0.0f
#define DELTA_LYA       0.0f

// 旋转中心 相对 车体参考点 的偏移 (cm) —— 一般填 0
#define DELTA_LX        0.0f
#define DELTA_LY        0.0f

// ============================================================
//  定位状态结构体
// ============================================================
typedef struct {
    // --- 原始传感器读数 ---
    float cur_rotation_angle_x;    // 轮1 累计角度 (deg)
    float cur_rotation_angle_y;    // 轮2 累计角度 (deg)
    float cur_inertial_angle;      // 惯性累计转角 (deg)
    float cur_yaw_angle;           // 航向角, -180 ~ +180 (deg)

    // --- 上一次的值（差分用） ---
    float last_rotation_angle_x;
    float last_rotation_angle_y;
    float last_inertial_angle;

    // --- 本周期增量 ---
    float delta_rotation_angle_x;  // 轮1 角度差 (deg)
    float delta_rotation_angle_y;  // 轮2 角度差 (deg)
    float delta_inertial_angle;    // 惯性转角差 (deg)
    float delta_dis_inertial_angle;// 惯性转角差 (rad)

    // --- 两个轮子的原始线位移 (cm) ---
    float s1;                      // 轮1 线位移
    float s2;                      // 轮2 线位移

    // --- 局部位移（解耦后, cm） ---
    float delta_x;                 // 沿车头方向
    float delta_y;                 // 沿车左方向

    // --- 全局位移增量 (cm) ---
    float g_delta_x;
    float g_delta_y;

    // --- 瞬心在局部坐标系的坐标 (cm) ---
    float r_xc, r_yc;

    // --- 最终输出：全局坐标 (cm) ---
    float gxT, gyT;   // 追踪点(结构中心) 全局坐标
    float gx0, gy0;   // 旋转中心 全局坐标
    float gxA, gyA;   // 辅助点 全局坐标

    // --- 备份 ---
    float f_gxT, f_gyT;

    // --- 状态标志 ---
    int   test_a;     // 0 = 近似直线, 1 = 有转动
} position_data_t;

extern position_data_t position_data;

// ============================================================
//  函数声明
// ============================================================
void Position_Init();                                  // 初始化(清零)
void Position_Reset(float x, float y);                 // 重设坐标原点
void UpdateGlobalPosition();                           // 每 2ms 调用一次

#endif