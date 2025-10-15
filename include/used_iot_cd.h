#ifndef USED_IOT_CD_H
#define USED_IOT_CD_H

#include "../drivers/sensors/max30102/kalman.h"
#include "../drivers/sensors/max30102/max30102.h"

// =====================
// 系统配置常量
// =====================
#define TASK_STACK_SIZE 1024
#define SENSOR_SAMPLING_RATE 100 // Hz
#define DATA_BUFFER_SIZE 128

#pragma pack(push, 1)
typedef struct
{
    // 加速度计数据 (原始值)
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    // 陀螺仪数据 (原始值)
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    // 生理数据
    uint32_t heart_rate;
    uint8_t spO2;

    // 时间戳和状态
    uint32_t timestamp;
    uint8_t signal_quality; // 0-100, 信号质量百分比
    char alert[32];         // 报警信息
} SensorData;
#pragma pack(pop)



#endif // USED_IOT_CD_H