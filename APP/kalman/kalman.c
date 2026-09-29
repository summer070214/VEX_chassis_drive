#include "kalman.h"
// 全局滤波器实例
KalmanFilter kalmanX,kalmanY;
// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter* kf, float Q, float R, float init_value) {
    kf->Q = Q;               // 过程噪声，旋转变化通常较慢，设为较小值
    kf->R = R;               // 测量噪声，根据传感器精度设置
    kf->x = init_value;      // 初始估计值
    kf->P = 1.0;             // 初始不确定度，设为较大值
    kf->K = 0.0;             // 初始增益
    kf->initialized = true;  // 标记为已初始化
}
// 卡尔曼滤波计算
float Kalman_Update(KalmanFilter* kf, float measurement) {
    if (!kf->initialized) {
        Kalman_Init(kf, 0.001, 0.1, measurement);
    }
    
    // 预测步骤
    // 对于旋转传感器，假设角度基本保持稳定（没有外力时）
    // 预测值不变，但不确定性增加
    kf->P = kf->P + kf->Q;
    
    // 更新步骤
    // 计算卡尔曼增益：不确定性越大，越信任测量值
    kf->K = kf->P / (kf->P + kf->R);
    
    // 状态更新：结合预测和测量
    kf->x = kf->x + kf->K * (measurement - kf->x);
    
    // 更新不确定性
    kf->P = (1.0 - kf->K) * kf->P;
    
    return kf->x;
}
// 初始化旋转传感器滤波器（在setup中调用）