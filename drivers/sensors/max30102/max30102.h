#ifndef __MYIIC_H
#define __MYIIC_H

#include "hi_types_base.h"
#include "hi_i2c.h"

// 解决NULL重定义警告的跨平台方案
#ifdef __cplusplus
#define MYIIC_NULL nullptr
#else
#define MYIIC_NULL ((void *)0)
#endif

// =====================
// I2C 基础配置
// =====================
#define MYIIC_I2C_IDX HI_I2C_IDX_0
#define MYIIC_I2C_SPEED 400000 // 400kHz

// I2C 方向定义
#define MYIIC_WRITE_DIR 0x00
#define MYIIC_READ_DIR 0x01

// =====================
// MAX30102 设备配置
// =====================
#define MAX30102_I2C_ADDRESS 0x57
#define MAX30102_WRITE_ADDR (MAX30102_I2C_ADDRESS << 1)
#define MAX30102_READ_ADDR ((MAX30102_I2C_ADDRESS << 1) | MYIIC_READ_DIR)

// =====================
// MAX30102 寄存器定义
// =====================
typedef enum
{
    MAX30102_REG_INTR_STATUS_1 = 0x00,
    MAX30102_REG_INTR_STATUS_2 = 0x01,
    MAX30102_REG_INTR_ENABLE_1 = 0x02,
    MAX30102_REG_INTR_ENABLE_2 = 0x03,
    MAX30102_REG_FIFO_WR_PTR = 0x04,
    MAX30102_REG_OVF_COUNTER = 0x05,
    MAX30102_REG_FIFO_RD_PTR = 0x06,
    MAX30102_REG_FIFO_DATA = 0x07,
    MAX30102_REG_FIFO_CONFIG = 0x08,
    MAX30102_REG_MODE_CONFIG = 0x09,
    MAX30102_REG_SPO2_CONFIG = 0x0A,
    MAX30102_REG_LED1_PA = 0x0C,
    MAX30102_REG_LED2_PA = 0x0D,
    MAX30102_REG_PILOT_PA = 0x10,
    MAX30102_REG_MULTI_LED_CTRL1 = 0x11,
    MAX30102_REG_MULTI_LED_CTRL2 = 0x12,
    MAX30102_REG_TEMP_INTR = 0x1F,
    MAX30102_REG_TEMP_FRAC = 0x20,
    MAX30102_REG_TEMP_CONFIG = 0x21,
    MAX30102_REG_PROX_INT_THRESH = 0x30,
    MAX30102_REG_REV_ID = 0xFE,
    MAX30102_REG_PART_ID = 0xFF
} MAX30102_RegisterMap;

// =====================
// 设备工作模式定义
// =====================
typedef enum
{
    MAX30102_MODE_HEART_RATE = 0x02, // 仅心率模式
    MAX30102_MODE_SPO2 = 0x03,       // 血氧+心率模式
    MAX30102_MODE_MULTI_LED = 0x07   // 多LED模式
} MAX30102_OperationMode;

// =====================
// 采样率配置定义
// =====================
typedef enum
{
    MAX30102_SAMPLE_RATE_50 = 0,
    MAX30102_SAMPLE_RATE_100 = 1,
    MAX30102_SAMPLE_RATE_200 = 2,
    MAX30102_SAMPLE_RATE_400 = 3,
    MAX30102_SAMPLE_RATE_800 = 4,
    MAX30102_SAMPLE_RATE_1000 = 5,
    MAX30102_SAMPLE_RATE_1600 = 6,
    MAX30102_SAMPLE_RATE_3200 = 7
} MAX30102_SampleRate;

// =====================
// LED脉冲宽度定义
// =====================
typedef enum
{
    MAX30102_PULSE_WIDTH_69US = 0,  // 15位分辨率
    MAX30102_PULSE_WIDTH_118US = 1, // 16位分辨率
    MAX30102_PULSE_WIDTH_215US = 2, // 17位分辨率
    MAX30102_PULSE_WIDTH_411US = 3  // 18位分辨率
} MAX30102_PulseWidth;

// =====================
// 算法参数配置
// =====================
#define MAX30102_DEFAULT_SAMPLING_RATE 100 // Hz
#define MAX30102_DEFAULT_LED_CURRENT 0x18  // 37mA
#define MAX30102_DEFAULT_SAMPLE_AVG 4      // 样本平均

// 数据有效性检查
#define MAX30102_MAX_VALID_VALUE 262143
#define MAX30102_MIN_VALID_VALUE 1000

// 滤波器参数
#define MAX30102_DC_ALPHA 0.01f     // DC分量滤波系数
#define MAX30102_HPF_BETA 0.95f     // 高通滤波器系数 (0.5Hz截止)
#define MAX30102_LPF_ALPHA 0.2f     // 低通滤波器系数 (10Hz截止)
#define MAX30102_LPF_ALPHA_DC 0.95f // DC分量低通滤波

// 心率检测参数
#define MAX30102_PEAK_THRESHOLD_RATIO 1.5f // 峰值检测阈值比例
#define MAX30102_MIN_PEAK_DISTANCE 60      // 最小峰值间距(样本数)
#define MAX30102_MAX_PEAKS 10              // 最大峰值数量

// 生理参数范围
#define MAX30102_MIN_HEART_RATE 30  // 最小有效心率
#define MAX30102_MAX_HEART_RATE 220 // 最大有效心率
#define MAX30102_MIN_SPO2 70        // 最小有效血氧
#define MAX30102_MAX_SPO2 100       // 最大有效血氧

// 缓冲区大小
#define MAX30102_DEFAULT_ROLLING_SIZE 200
#define MAX30102_HEART_RATE_WINDOW 100

// 血氧计算参数
#define MAX30102_SPO2_CALIBRATION_A 110.0f
#define MAX30102_SPO2_CALIBRATION_B 25.0f
#define MAX30102_SPO2_FORMULA_C1 -25.0f
#define MAX30102_SPO2_FORMULA_C2 110.0f

// 信号质量检测
#define MAX30102_MIN_AC_VALUE 700           // 最小AC信号值
#define MAX30102_MIN_DC_VALUE 5000          // 最小DC信号值
#define MAX30102_MAX_PULSE_AMPLITUDE 200000 // 最大脉冲幅度
#define MAX30102_AC_PEAK_THRESHOLD 50.0f    // AC峰值阈值

// 质量控制参数
#define MAX30102_SIGNAL_QUALITY_THRESHOLD 0.6f // 信号质量阈值
#define MAX30102_STABLE_READINGS_REQUIRED 5    // 稳定读数要求

// =====================
// 数据结构定义
// =====================
typedef struct
{
    uint32_t red_value;
    uint32_t ir_value;
    uint32_t timestamp;
    uint8_t signal_quality; // 0-100, 信号质量百分比
    bool data_valid;
} MAX30102_SensorData;

// =====================
// MAX30102 状态结构体
// =====================
typedef struct
{
    // 数据缓冲区
    uint32_t red_buffer[MAX30102_DEFAULT_ROLLING_SIZE];
    uint32_t ir_buffer[MAX30102_DEFAULT_ROLLING_SIZE];
    uint16_t buffer_index;

    // 实时生理数据
    uint32_t heart_rate;
    uint8_t spO2;
    bool finger_detected;

    // DC分量和基线
    float red_dc;
    float ir_dc;
    float red_baseline;
    float ir_baseline;

    // 滤波器状态
    struct
    {
        float hp_prev_input;
        float hp_prev_output;
        float hp_beta; // 高通滤波器系数

        float lp_prev1;
        float lp_prev2;
        float lp_alpha; // 低通滤波器系数
    } filter;

    // 信号处理状态
    struct
    {
        float red_sum;
        float ir_sum;
        float smoothed_hr;
        float spO2_smooth;

        float red_prev_filtered;
        float ir_prev_filtered;
        float red_peak;
        float red_trough;
        float ir_peak;
        float ir_trough;
        bool new_heartbeat_cycle;
    } signal;

    // 质量控制和稳定性
    struct
    {
        float signal_quality; // 0.0-1.0 信号质量
        float last_valid_hr;
        uint8_t consecutive_valid_readings;
        uint8_t signal_stability; // 0-100 稳定性评分
        uint32_t invalid_count;
        uint32_t last_update_time;
        bool kalman_initialized;
    } quality;

    // 卡尔曼滤波器
    struct
    {
        float process_noise;
        float measurement_noise;
        float estimate;
        float error_estimate;
        float last_estimate;
    } hr_filter, spO2_filter;

    // 配置参数
    struct
    {
        uint16_t min_heart_rate;
        uint16_t max_heart_rate;
        uint8_t min_spO2;
        uint8_t max_spO2;
    } limits;

} MAX30102State;

// =====================
// 函数声明
// =====================

// I2C 基础操作
hi_u32 MAX30102_I2C_Write(uint8_t reg_addr, const uint8_t *data, uint8_t data_len);
hi_u32 MAX30102_I2C_Read(uint8_t reg_addr, uint8_t *data, uint8_t data_len);

// 设备初始化与控制
hi_u32 MAX30102_Device_Init(MAX30102State *state);
hi_u32 MAX30102_Device_Reset(void);
hi_u32 MAX30102_Check_Device_ID(void);

// 配置函数
hi_u32 MAX30102_Set_Operation_Mode(MAX30102_OperationMode mode);
hi_u32 MAX30102_Set_Sample_Rate(MAX30102_SampleRate sample_rate);
hi_u32 MAX30102_Set_Pulse_Width(MAX30102_PulseWidth pulse_width);
hi_u32 MAX30102_Set_LED_Current(uint8_t red_current, uint8_t ir_current);
hi_u32 MAX30102_Set_FIFO_Config(uint8_t sample_avg, uint8_t rollover_enable);

// 数据读取
hi_u32 MAX30102_Read_FIFO_Data(uint32_t *red_value, uint32_t *ir_value);
hi_u32 MAX30102_Read_Sensor_Data(MAX30102_SensorData *sensor_data);

// 数据处理
float MAX30102_Calculate_Signal_Quality(const uint32_t *ir_buffer, uint16_t buffer_size);
bool MAX30102_Detect_Finger_Presence(uint32_t red_value, uint32_t ir_value);
void MAX30102_Update_Heart_Rate(MAX30102State *state, uint32_t ir_value);
void MAX30102_Update_SpO2(MAX30102State *state, uint32_t red_value, uint32_t ir_value);
void MAX30102_Read_And_Update_Data(MAX30102State *state);

    // 工具函数
    uint32_t MAX30102_Get_Current_Time(void);
float MAX30102_Fast_Sqrtf(float x);
float MAX30102_Fast_Fminf(float a, float b);
float MAX30102_Fast_Fmaxf(float a, float b);

// 状态管理
void MAX30102_Reset_State(MAX30102State *state);
void MAX30102_Set_HeartRate_Limits(MAX30102State *state, uint16_t min_hr, uint16_t max_hr);
void MAX30102_Set_SpO2_Limits(MAX30102State *state, uint8_t min_spo2, uint8_t max_spo2);

// 调试功能
void MAX30102_Print_Sensor_Data(const MAX30102_SensorData *data);
void MAX30102_Print_State(const MAX30102State *state);



#endif // __MYIIC_H