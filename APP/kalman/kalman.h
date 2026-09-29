#ifndef KALMAN_H
#define KALMAN_H
#include "vex.h"
typedef struct {
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float x;        // 状态估计值
    float P;        // 估计误差协方差
    float K;        // 卡尔曼增益
    bool initialized; // 初始化标志
} KalmanFilter;
extern KalmanFilter kalmanX;
extern KalmanFilter kalmanY;

void Kalman_Init(KalmanFilter* kf, float Q, float R, float init_value);
float Kalman_Update(KalmanFilter* kf, float measurement);
#endif