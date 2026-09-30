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
//                  角度转弧度
// ============================================================

static inline float deg_to_rad(float deg)
{
    return deg * TWO_PI / 360.0f;
}


// ============================================================
//                  角度归一化到 -180 ~ +180
// ============================================================

static inline float normalize_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle < -180.0f)
        angle += 360.0f;

    return angle;
}


// ============================================================
//                  安装角
// ============================================================

static const float ALPHA_RAD =
    deg_to_rad(WHEEL1_ANGLE_DEG);

static const float BETA_RAD =
    deg_to_rad(WHEEL2_ANGLE_DEG);


static const float SIN_DA =
    sinf(BETA_RAD - ALPHA_RAD);

static const float INV_SIN_DA =
    (fabsf(SIN_DA) > 1e-5f)
    ? (1.0f / SIN_DA)
    : 0.0f;


// ============================================================
//                  两轮解耦
// ============================================================

static inline void decode_wheel_to_local(
    float s1,
    float s2,
    float &dx,
    float &dy)
{
    dx =
        INV_SIN_DA *
        (
            s1 * sinf(BETA_RAD)
            -
            s2 * sinf(ALPHA_RAD)
        );

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
    memset(
        &position_data,
        0,
        sizeof(position_data)
    );

    // 编码器清零
    RotationX.setPosition(0, deg);
    RotationY.setPosition(0, deg);

    // IMU 清零
    Inertial.resetRotation();

    // 编码器历史值
    position_data.last_rotation_angle_x = 0.0f;
    position_data.last_rotation_angle_y = 0.0f;

    // IMU 历史值
    position_data.last_inertial_angle = 0.0f;
    position_data.delta_inertial_angle = 0.0f;
    position_data.delta_dis_inertial_angle = 0.0f;

    // 初始位置
    position_data.gxT = 0.0f;
    position_data.gyT = 0.0f;

    position_data.gx0 = 0.0f;
    position_data.gy0 = 0.0f;

    position_data.f_gxT = 0.0f;
    position_data.f_gyT = 0.0f;
}


// ============================================================
//                  重设坐标
// ============================================================

void Position_Reset(float x, float y)
{
    // 直接读取当前 IMU 航向
    float yaw = Inertial.heading();

    if (yaw >= 180.0f)
        yaw -= 360.0f;

    float yaw_rad = deg_to_rad(yaw);

    // 设置当前位置
    position_data.gxT = x;
    position_data.gyT = y;

    // 根据当前位置计算旋转中心
    position_data.gx0 =
        x -
        (
            DELTA_LX * cosf(yaw_rad)
            +
            DELTA_LY * sinf(yaw_rad)
        );

    position_data.gy0 =
        y -
        (
            DELTA_LY * cosf(yaw_rad)
            -
            DELTA_LX * sinf(yaw_rad)
        );

    // 备份
    position_data.f_gxT =
        position_data.gxT;

    position_data.f_gyT =
        position_data.gyT;

    // 编码器清零
    RotationX.setPosition(0, deg);
    RotationY.setPosition(0, deg);

    // 软件差分清零
    position_data.last_rotation_angle_x = 0.0f;
    position_data.last_rotation_angle_y = 0.0f;

    // IMU 差分重新建立基准
    position_data.last_inertial_angle =
        Inertial.rotation();

    position_data.delta_inertial_angle = 0.0f;
    position_data.delta_dis_inertial_angle = 0.0f;
}


// ============================================================
//                  核心定位函数
// ============================================================
//
// 2个定位轮 + IMU
//
// RotationX：定位轮1
// RotationY：定位轮2
//
// 默认：
// WHEEL1_ANGLE_DEG = 0°
// WHEEL2_ANGLE_DEG = 90°
//
// 因此：
// dx = 前后
// dy = 左右
// ============================================================

void UpdateGlobalPosition()
{
    // ========================================================
    // 1. 当前编码器角度
    // ========================================================

    float cur_rotation_angle_x =
        RotationX.position(deg);

    float cur_rotation_angle_y =
        RotationY.position(deg);


    // ========================================================
    // 2. 编码器角度增量
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
    // 3. 编码器角度 → 定位轮位移
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
    // 4. 两个定位轮解耦
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
    // 7. IMU 角度增量
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
    // 9. 角度 → 弧度
    //
    // 保持和你旧 locate 的结构兼容
    //
    // delta_inertial_angle：
    //      单位：°
    //
    // delta_dis_inertial_angle：
    //      单位：rad
    // ========================================================

    position_data.delta_dis_inertial_angle =
        position_data.delta_inertial_angle
        * TWO_PI
        / 360.0f;


    // 更新 IMU 历史值
    position_data.last_inertial_angle =
        position_data.cur_inertial_angle;


    // ========================================================
    // 10. 当前航向
    // ========================================================

    float yaw = Inertial.heading();

    if (yaw >= 180.0f)
        yaw -= 360.0f;

    float yaw_rad =
        deg_to_rad(yaw);


    // ========================================================
    // 11. 当前旋转量
    // ========================================================

    float dtheta =
        position_data.delta_dis_inertial_angle;


    // ========================================================
    // 12. 判断是否近似直线
    // ========================================================

    if (fabsf(dtheta) < deg_to_rad(0.55f))
    {
        // ====================================================
        // 近似直线运动
        // ====================================================

        float g_delta_x =
            delta_x * cosf(yaw_rad)
            -
            delta_y * sinf(yaw_rad);

        float g_delta_y =
            delta_x * sinf(yaw_rad)
            +
            delta_y * cosf(yaw_rad);


        // 更新全局坐标
        position_data.gxT +=
            g_delta_x;

        position_data.gyT +=
            g_delta_y;
    }
    else
    {
        // ====================================================
        // 发生旋转
        //
        // 通过瞬时旋转中心进行圆弧积分
        // ====================================================

        float safe_dtheta = dtheta;

        if (fabsf(safe_dtheta) < 1e-5f)
        {
            safe_dtheta =
                (safe_dtheta >= 0.0f)
                ? 1e-5f
                : -1e-5f;
        }


        // ----------------------------------------------------
        // 瞬时旋转中心
        // ----------------------------------------------------

        float r_x =
            delta_y / safe_dtheta;

        float r_y =
            -delta_x / safe_dtheta;


        // ----------------------------------------------------
        // 圆弧积分
        // ----------------------------------------------------

        float cos_d =
            cosf(safe_dtheta);

        float sin_d =
            sinf(safe_dtheta);


        float local_dx =
            r_x * sin_d
            +
            r_y * (1.0f - cos_d);


        float local_dy =
            r_y * sin_d
            -
            r_x * (1.0f - cos_d);


        // ----------------------------------------------------
        // 机器人坐标 → 场地坐标
        // ----------------------------------------------------

        float g_delta_x =
            local_dx * cosf(yaw_rad)
            -
            local_dy * sinf(yaw_rad);

        float g_delta_y =
            local_dx * sinf(yaw_rad)
            +
            local_dy * cosf(yaw_rad);


        // ----------------------------------------------------
        // 更新全局坐标
        // ----------------------------------------------------

        position_data.gxT +=
            g_delta_x;

        position_data.gyT +=
            g_delta_y;
    }


    // ========================================================
    // 13. 更新旋转中心
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
    // 14. 最终备份
    // ========================================================

    position_data.f_gxT =
        position_data.gxT;

    position_data.f_gyT =
        position_data.gyT;
}