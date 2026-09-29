#include "vex.h"
#include "locate.h"
#include "robot_config.h"
using namespace vex;

extern vex::inertial Inertial;

// ============================================================
//  结构体实例
// ============================================================
position_data_t position_data;

// ============================================================
//  小工具：保留一位小数
// ============================================================
static inline float q1(float v) {
    return (float)((int)(v * 10.0f)) / 10.0f;
}

// ============================================================
//  安装角（弧度），编译期常量
// ============================================================
static const float ALPHA_RAD = WHEEL1_ANGLE_DEG / 360.0f * TWO_PI;
static const float BETA_RAD  = WHEEL2_ANGLE_DEG  / 360.0f * TWO_PI;

// 解耦矩阵系数
// [ delta_x ]   =  1/sin(β-α) * [  sinβ  -sinα ] [ s1 ]
// [ delta_y ]                   [ -cosβ   cosα ] [ s2 ]
static const float SIN_DA = sinf(BETA_RAD - ALPHA_RAD);
static const float INV_SIN_DA = (fabsf(SIN_DA) > 1e-4f) ? (1.0f / SIN_DA) : 0.0f;

static inline void decode_wheel_to_local(float s1, float s2,
                                         float &dx, float &dy) {
    dx = INV_SIN_DA * ( s1 * sinf(BETA_RAD) - s2 * sinf(ALPHA_RAD));
    dy = INV_SIN_DA * (-s1 * cosf(BETA_RAD) + s2 * cosf(ALPHA_RAD));
}

// ============================================================
//  初始化：清零所有状态、传感器
// ============================================================
void Position_Init() {
    memset(&position_data, 0, sizeof(position_data));

    RotationX.setPosition(0, deg);
    RotationY.setPosition(0, deg);
    Inertial.resetRotation();

    position_data.last_rotation_angle_x = 0;
    position_data.last_rotation_angle_y = 0;
    position_data.last_inertial_angle   = 0;
    position_data.gxT = position_data.gyT = 0;
    position_data.gx0 = position_data.gy0 = 0;
    position_data.f_gxT = position_data.f_gyT = 0;
}

// ============================================================
//  重设坐标原点
//  注意：x 只赋给 x 分量，y 只赋给 y 分量
// ============================================================
void Position_Reset(float x, float y) {
    float yaw_rad = position_data.cur_yaw_angle / 360.0f * TWO_PI;

    // 追踪点新起点
    position_data.gxT = x;
    position_data.gyT = y;

    // 由追踪点反推旋转中心
    position_data.gx0 = x - (DELTA_LX * cosf(yaw_rad) + DELTA_LY * sinf(yaw_rad));
    position_data.gy0 = y - (DELTA_LY * cosf(yaw_rad) - DELTA_LX * sinf(yaw_rad));

    // 备份
    position_data.f_gxT = position_data.gxT;
    position_data.f_gyT = position_data.gyT;

    // 传感器清零
    RotationX.setPosition(0, deg);
    RotationY.setPosition(0, deg);
    Inertial.resetRotation();
    // 若需要朝向也归零，取消下一行注释：
    // Inertial.resetHeading();

    // 软件缓存清零
    position_data.last_rotation_angle_x = 0;
    position_data.last_rotation_angle_y = 0;
    position_data.last_inertial_angle   = 0;
}

// ============================================================
//  核心：更新全局坐标
//  建议在 2ms 定时中断 或 主循环里周期调用
// ============================================================
void UpdateGlobalPosition() {

    // --------------------------------------------------------
    // 1. 读原始传感器
    // --------------------------------------------------------
    position_data.cur_rotation_angle_x = q1(RotationX.position(deg));
    position_data.cur_rotation_angle_y = q1(RotationY.position(deg));
    position_data.cur_inertial_angle   = q1(Inertial.rotation());

    float yaw = Inertial.heading();
    if (yaw >= 180.0f) yaw -= 360.0f;
    position_data.cur_yaw_angle = q1(yaw);

    // --------------------------------------------------------
    // 2. 惯性角度增量 & 直/弯判断
    // --------------------------------------------------------
    position_data.delta_inertial_angle = position_data.cur_inertial_angle - position_data.last_inertial_angle;
    position_data.last_inertial_angle  = position_data.cur_inertial_angle;

    if (fabsf(position_data.delta_inertial_angle) <= 0.55f) {
        position_data.test_a = 0;   // 近似直线
    } else {
        position_data.test_a = 1;   // 有转动
    }

    const float yaw_rad = position_data.cur_yaw_angle / 360.0f * TWO_PI;

    // --------------------------------------------------------
    // 3. 读两轮角度差 → 原始线位移 → 解耦成局部 delta_x/delta_y
    //    这一步把“非 90° 安装”统一处理掉，
    //    后面的直线情况 / 瞬心法都直接用 delta_x/delta_y
    // --------------------------------------------------------
    position_data.delta_rotation_angle_x = position_data.cur_rotation_angle_x - position_data.last_rotation_angle_x;
    position_data.delta_rotation_angle_y = position_data.cur_rotation_angle_y - position_data.last_rotation_angle_y;

    position_data.s1 = position_data.delta_rotation_angle_x / 360.0f * TWO_PI * CODING_R;
    position_data.s2 = position_data.delta_rotation_angle_y / 360.0f * TWO_PI * CODING_R;

    decode_wheel_to_local(position_data.s1, position_data.s2,
                          position_data.delta_x, position_data.delta_y);

    // 更新差分基准
    position_data.last_rotation_angle_x = position_data.cur_rotation_angle_x;
    position_data.last_rotation_angle_y = position_data.cur_rotation_angle_y;

    // ========================================================
    // 情况 A：近似直线 —— 旋转矩阵
    // ========================================================
    if (position_data.test_a == 0) {

        position_data.g_delta_x = position_data.delta_x * cosf(yaw_rad) + position_data.delta_y * sinf(yaw_rad);
        position_data.g_delta_y = position_data.delta_y * cosf(yaw_rad) - position_data.delta_x * sinf(yaw_rad);

        position_data.gxT += position_data.g_delta_x;
        position_data.gyT += position_data.g_delta_y;

        position_data.gx0 = position_data.gxT - (DELTA_LX * cosf(yaw_rad) + DELTA_LY * sinf(yaw_rad));
        position_data.gy0 = position_data.gyT - (DELTA_LY * cosf(yaw_rad) - DELTA_LX * sinf(yaw_rad));

    // ========================================================
    // 情况 B：有转动 —— 瞬心法
    // ========================================================
    } else {

        position_data.delta_dis_inertial_angle = position_data.delta_inertial_angle / 360.0f * TWO_PI;
        if (fabsf(position_data.delta_dis_inertial_angle) < 1e-4f) {
            position_data.delta_dis_inertial_angle = 1e-4f;   // 防止除零
        }

        // 瞬心到 X / Y 定位轮径向线的距离
        float delta_dis_x = position_data.delta_x / position_data.delta_dis_inertial_angle;
        float delta_dis_y = position_data.delta_y / position_data.delta_dis_inertial_angle;

        // 瞬心在局部坐标系下的坐标
        position_data.r_yc = -delta_dis_x;
        position_data.r_xc =  delta_dis_y;

        // --- 旋转中心更新 ---
        float r_oc_x = position_data.r_xc;
        float r_oc_y = position_data.r_yc;

        float d_rad = position_data.delta_inertial_angle / 360.0f * TWO_PI;

        // r_oc 顺时针旋转 d_rad
        float r_oc_cl_x =  r_oc_x * cosf(d_rad) + r_oc_y * sinf(d_rad);
        float r_oc_cl_y = -r_oc_x * sinf(d_rad) + r_oc_y * cosf(d_rad);

        // 旋转前后向量差
        float r_oo2_x = r_oc_x - r_oc_cl_x;
        float r_oo2_y = r_oc_y - r_oc_cl_y;

        // 转到全局
        float r_oo2_cl_x =  r_oo2_x * cosf(yaw_rad) + r_oo2_y * sinf(yaw_rad);
        float r_oo2_cl_y = -r_oo2_x * sinf(yaw_rad) + r_oo2_y * cosf(yaw_rad);

        position_data.gx0 += r_oo2_cl_x;
        position_data.gy0 += r_oo2_cl_y;

        // --- 追踪点更新 ---
        float r_ot_cl_x =  DELTA_LXA * cosf(yaw_rad) + DELTA_LYA * sinf(yaw_rad);
        float r_ot_cl_y = -DELTA_LXA * sinf(yaw_rad) + DELTA_LYA * cosf(yaw_rad);

        position_data.gxT = position_data.gx0 + r_ot_cl_x;
        position_data.gyT = position_data.gy0 + r_ot_cl_y;
    }

    // --------------------------------------------------------
    // 4. 备份
    // --------------------------------------------------------
    position_data.f_gxT = position_data.gxT;
    position_data.f_gyT = position_data.gyT;
}