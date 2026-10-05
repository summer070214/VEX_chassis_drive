#include "vex.h"
#include "locate_5225A.h"
#include "robot_config.h"

#include <cmath>
#include <cstring>

using namespace vex;


// ============================================================
//                  外部 IMU
// ============================================================

extern vex::inertial Inertial;


// ============================================================
//                  全局定位数据
// ============================================================

position_data_t position_data;


// ============================================================
//                  数学工具
// ============================================================

static inline float deg_to_rad(float deg)
{
    return deg * DEG_TO_RAD;
}


// ------------------------------------------------------------
// 角度归一化到 [-180°, +180°]
// ------------------------------------------------------------

static inline float normalize_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle < -180.0f)
        angle += 360.0f;

    return angle;
}


// ============================================================
//                  定位轮安装角
// ============================================================

static const float ALPHA_RAD =
    deg_to_rad(WHEEL1_ANGLE_DEG);

static const float BETA_RAD =
    deg_to_rad(WHEEL2_ANGLE_DEG);


// ============================================================
//                  两个定位轮方向解算
// ============================================================
//
// 两个轮分别测量：
//
//     s1 = dx*cos(A) + dy*sin(A)
//
//     s2 = dx*cos(B) + dy*sin(B)
//
// 解：
//
//     dx = [s1*sin(B) - s2*sin(A)] / sin(B-A)
//
//     dy = [-s1*cos(B) + s2*cos(A)] / sin(B-A)
//
// 因此：
//
//     任意两个不平行的定位轮都可以使用。
//
// ============================================================

static const float SIN_DA =
    sinf(BETA_RAD - ALPHA_RAD);


// ------------------------------------------------------------
// 防止两个轮安装方向几乎平行
// ------------------------------------------------------------

static const float INV_SIN_DA =
    (fabsf(SIN_DA) > 1e-5f)
        ? (1.0f / SIN_DA)
        : 0.0f;


// ============================================================
//                  定位轮位移 → 机器人局部 X/Y
// ============================================================

static inline void decode_wheel_to_local(
    float s1,
    float s2,
    float &dx,
    float &dy)
{
    // --------------------------------------------------------
    // 如果两个定位轮方向接近平行，
    // 无法稳定解算 X/Y。
    // --------------------------------------------------------

    if (fabsf(SIN_DA) < 1e-5f)
    {
        dx = 0.0f;
        dy = 0.0f;
        return;
    }


    // --------------------------------------------------------
    // 机器人局部 X
    //
    // X+ = 右
    // --------------------------------------------------------

    dx =
        INV_SIN_DA *
        (
            s1 * sinf(BETA_RAD)
            -
            s2 * sinf(ALPHA_RAD)
        );


    // --------------------------------------------------------
    // 机器人局部 Y
    //
    // Y+ = 前
    // --------------------------------------------------------

    dy =
        INV_SIN_DA *
        (
            -s1 * cosf(BETA_RAD)
            +
            s2 * cosf(ALPHA_RAD)
        );
}


// ============================================================
//                  初始化
// ============================================================

void Position_Init()
{
    // --------------------------------------------------------
    // 清空定位数据
    // --------------------------------------------------------

    memset(
        &position_data,
        0,
        sizeof(position_data)
    );


    // --------------------------------------------------------
    // 定位轮编码器清零
    // --------------------------------------------------------

    RotationX.setPosition(0, deg);
    RotationY.setPosition(0, deg);


    // --------------------------------------------------------
    // IMU旋转角度清零
    // --------------------------------------------------------

    Inertial.resetRotation();


    // --------------------------------------------------------
    // 编码器历史值
    // --------------------------------------------------------

    position_data.last_rotation_angle_x = 0.0f;
    position_data.last_rotation_angle_y = 0.0f;


    // --------------------------------------------------------
    // IMU历史值
    // --------------------------------------------------------

    position_data.last_inertial_angle = 0.0f;

    position_data.delta_inertial_angle = 0.0f;

    position_data.delta_dis_inertial_angle = 0.0f;


    // --------------------------------------------------------
    // 初始坐标
    // --------------------------------------------------------

    position_data.gxT = 0.0f;
    position_data.gyT = 0.0f;

    position_data.gx0 = 0.0f;
    position_data.gy0 = 0.0f;

    position_data.f_gxT = 0.0f;
    position_data.f_gyT = 0.0f;


    // --------------------------------------------------------
    // 当前航向
    // --------------------------------------------------------

    position_data.cur_yaw_angle = 0.0f;
}


// ============================================================
//                  设置机器人初始坐标
// ============================================================

void Position_Reset(float x, float y)
{
    // --------------------------------------------------------
    // 当前 IMU 航向
    // --------------------------------------------------------

    float yaw =
        Inertial.heading();


    yaw =
        normalize_angle(yaw);


    float yaw_rad =
        deg_to_rad(yaw);


    // --------------------------------------------------------
    // 设置当前位置
    // --------------------------------------------------------

    position_data.gxT = x;
    position_data.gyT = y;


    // --------------------------------------------------------
    // 计算旋转中心
    //
    // 本程序坐标：
    //
    // X+ = 右
    // Y+ = 前
    // --------------------------------------------------------

    position_data.gx0 =
        x
        -
        (
            DELTA_LX * cosf(yaw_rad)
            -
            DELTA_LY * sinf(yaw_rad)
        );


    position_data.gy0 =
        y
        -
        (
            DELTA_LX * sinf(yaw_rad)
            +
            DELTA_LY * cosf(yaw_rad)
        );


    // --------------------------------------------------------
    // 最终位置备份
    // --------------------------------------------------------

    position_data.f_gxT =
        position_data.gxT;

    position_data.f_gyT =
        position_data.gyT;


    // --------------------------------------------------------
    // 编码器清零
    // --------------------------------------------------------

    RotationX.setPosition(0, deg);
    RotationY.setPosition(0, deg);


    position_data.last_rotation_angle_x = 0.0f;
    position_data.last_rotation_angle_y = 0.0f;


    // --------------------------------------------------------
    // 建立 IMU 差分基准
    // --------------------------------------------------------

    position_data.last_inertial_angle =
        Inertial.rotation();

    position_data.cur_inertial_angle =
        position_data.last_inertial_angle;

    position_data.cur_yaw_angle =
        Inertial.heading();


    position_data.delta_inertial_angle = 0.0f;

    position_data.delta_dis_inertial_angle = 0.0f;
}


// ============================================================
//                  核心定位
// ============================================================
//
// 两个任意角度的定位轮
// +
// IMU
//
// 最终得到：
//
//     position_data.gxT
//     position_data.gyT
//
// 坐标定义：
//
//     X+ = 左 → 右
//     Y+ = 下 → 上
//
// ============================================================

void UpdateGlobalPosition()
{
    // ========================================================
    // 1. 读取两个定位轮编码器
    // ========================================================

    float cur_rotation_angle_x =
        RotationX.position(deg);

    float cur_rotation_angle_y =
        RotationY.position(deg);


    // ========================================================
    // 2. 计算编码器角度变化
    // ========================================================

    float delta_rotation_angle_x =
        cur_rotation_angle_x
        -
        position_data.last_rotation_angle_x;


    float delta_rotation_angle_y =
        cur_rotation_angle_y
        -
        position_data.last_rotation_angle_y;


    // ========================================================
    // 3. 编码器角度 → 定位轮实际滚动距离
    //
    //     s = Δangle / 360 × 2πR
    //
    // 单位：cm
    // ========================================================

    float s1 =
        delta_rotation_angle_x
        / 360.0f
        * TWO_PI
        * CODING_R;


    float s2 =
        delta_rotation_angle_y
        / 360.0f
        * TWO_PI
        * CODING_R;


    // ========================================================
    // 4. 两个定位轮 → 机器人局部 X/Y
    //
    // X+ = 右
    // Y+ = 前
    // ========================================================

    float delta_x = 0.0f;
    float delta_y = 0.0f;


    decode_wheel_to_local(
        s1,
        s2,
        delta_x,
        delta_y
    );


    // ========================================================
    // 5. 更新编码器历史值
    // ========================================================

    position_data.last_rotation_angle_x =
        cur_rotation_angle_x;

    position_data.last_rotation_angle_y =
        cur_rotation_angle_y;


    // ========================================================
    // 6. 读取 IMU
    // ========================================================

    position_data.cur_inertial_angle =
        Inertial.rotation();


    // ========================================================
    // 7. IMU角度变化
    // ========================================================

    position_data.delta_inertial_angle =
        position_data.cur_inertial_angle
        -
        position_data.last_inertial_angle;


    // ========================================================
    // 8. 角度归一化
    // ========================================================

    position_data.delta_inertial_angle =
        normalize_angle(
            position_data.delta_inertial_angle
        );


    // ========================================================
    // 9. degree → rad
    // ========================================================

    position_data.delta_dis_inertial_angle =
        position_data.delta_inertial_angle
        * DEG_TO_RAD;


    // ========================================================
    // 10. 更新 IMU历史值
    // ========================================================

    position_data.last_inertial_angle =
        position_data.cur_inertial_angle;


    // ========================================================
    // 11. 当前机器人航向
    //
    // VEX heading：
    //
    //     0°   = 初始方向
    //     顺时针增加
    //
    // 为了统一数学坐标系，
    // 这里直接使用 heading 做旋转。
    // ========================================================

    float yaw =
        Inertial.heading();


    yaw =
        normalize_angle(yaw);


    position_data.cur_yaw_angle =
        yaw;


    float yaw_rad =
        deg_to_rad(yaw);


    // ========================================================
    // 12. 本次旋转角
    // ========================================================

    float dtheta =
        position_data.delta_dis_inertial_angle;


    // ========================================================
    // 13. 定位轮旋转补偿
    // ========================================================
    //
    // 如果定位参考点不在旋转中心，
    // 机器人旋转会导致定位轮产生额外位移。
    //
    // 这里使用定位参考点：
    //
    //     r = (DELTA_LX, DELTA_LY)
    //
    // 的刚体旋转模型计算旋转产生的位移。
    //
    // 然后从编码器测量位移中扣除。
    //
    // ========================================================

    float rotation_dx = 0.0f;
    float rotation_dy = 0.0f;


    if (fabsf(dtheta) > 1e-7f)
    {
        float cos_d =
            cosf(dtheta);

        float sin_d =
            sinf(dtheta);


        rotation_dx =
            DELTA_LX * (cos_d - 1.0f)
            -
            DELTA_LY * sin_d;


        rotation_dy =
            DELTA_LX * sin_d
            +
            DELTA_LY * (cos_d - 1.0f);
    }


    // --------------------------------------------------------
    // 从编码器测量结果中去除旋转造成的位移
    // --------------------------------------------------------

    delta_x -= rotation_dx;
    delta_y -= rotation_dy;


    // ========================================================
    // 14. 机器人局部坐标 → 场地全局坐标
    // ========================================================
    //
    // 当前机器人：
    //
    //     delta_x = 机器人右方向
    //     delta_y = 机器人前方向
    //
    // yaw = 机器人相对于场地的旋转角
    //
    // ========================================================

    float global_dx =
        delta_x * cosf(yaw_rad)
        -
        delta_y * sinf(yaw_rad);


    float global_dy =
        delta_x * sinf(yaw_rad)
        +
        delta_y * cosf(yaw_rad);


    // ========================================================
    // 15. 更新全局坐标
    // ========================================================

    position_data.gxT +=
        global_dx;

    position_data.gyT +=
        global_dy;


    // ========================================================
    // 16. 更新旋转中心
    // ========================================================

    position_data.gx0 =
        position_data.gxT
        -
        (
            DELTA_LX * cosf(yaw_rad)
            -
            DELTA_LY * sinf(yaw_rad)
        );


    position_data.gy0 =
        position_data.gyT
        -
        (
            DELTA_LX * sinf(yaw_rad)
            +
            DELTA_LY * cosf(yaw_rad)
        );


    // ========================================================
    // 17. 最终位置备份
    // ========================================================

    position_data.f_gxT =
        position_data.gxT;

    position_data.f_gyT =
        position_data.gyT;
}