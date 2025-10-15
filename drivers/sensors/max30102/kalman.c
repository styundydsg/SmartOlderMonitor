#include "kalman.h"

KalmanFilter kf;

void KalmanFilter_Init(KalmanFilter *filter, float q, float r, float initial_value, float initial_error)
{
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = initial_error;
    filter->k = 0;
}

float KalmanFilter_Update(KalmanFilter *filter, float measurement)
{
    // 预测步骤
    filter->p = filter->p + filter->q;

    // 更新步骤
    filter->k = filter->p / (filter->p + filter->r);
    filter->x = filter->x + filter->k * (measurement - filter->x);
    filter->p = (1 - filter->k) * filter->p;

    return filter->x;
}

void KalmanFilter_SetParameters(KalmanFilter *kf, float process_noise, float measurement_noise)
{
    kf->q = process_noise;
    kf->r = measurement_noise;
}

float KalmanFilter_GetValue(KalmanFilter *kf)
{
    return kf->x;
}