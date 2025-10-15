// kalman.h
#ifndef KALMAN_H // 如果没有定义 KALMAN_H
#define KALMAN_H // 就定义它

// 定义卡尔曼滤波器结构体
typedef struct
{
    float   q; // 过程噪声协方差
    float   r; // 观测噪声协方差
    float   p; // 估计误差协方差
    float   k; // 卡尔曼增益
    float   x; // 估计值

    
} KalmanFilter;

// 初始化卡尔曼滤波器的函数声明
void KalmanFilter_Init(KalmanFilter *kf, float process_noise, float measurement_noise, float initial_value, float initial_error);

// 声明卡尔曼滤波器更新函数
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

void KalmanFilter_SetParameters(KalmanFilter *kf, float process_noise, float measurement_noise);

float KalmanFilter_GetValue(KalmanFilter *kf);

#endif // KALMAN_H
