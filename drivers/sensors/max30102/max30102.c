// =====================
// MAX30102 传感器驱动与算法实现（适配优化版）
// =====================

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "ohos_init.h"
#include "hi_io.h"
#include "hi_i2c.h"
#include "hi_time.h"
#include "used_iot_cd.h"
#include "max30102.h"
#include "kalman.h"

// 全局变量定义
uint32_t g_max30102_fifo_red = 0;
uint32_t g_max30102_fifo_ir = 0;

/******************************************************
 * Part 1: 基础读写函数（封装错误检查 + 代码压缩）
 ******************************************************/

hi_u32 MAX30102_I2C_Write(uint8_t reg_addr, const uint8_t *data, uint8_t data_len)
{
    uint8_t buf[data_len + 1];
    buf[0] = reg_addr;
    if (data != NULL && data_len > 0)
    {
        memcpy(&buf[1], data, data_len);
    }

    hi_i2c_data i2cData = {
        .send_buf = buf,
        .send_len = data_len + 1,
        .receive_buf = NULL,
        .receive_len = 0};

    return hi_i2c_write(MYIIC_I2C_IDX, MAX30102_WRITE_ADDR, &i2cData);
}

hi_u32 MAX30102_I2C_Read(uint8_t reg_addr, uint8_t *data, uint8_t data_len)
{
    hi_i2c_data i2cData = {
        .send_buf = &reg_addr,
        .send_len = 1,
        .receive_buf = NULL,
        .receive_len = 0};

    if (hi_i2c_write(MYIIC_I2C_IDX, MAX30102_WRITE_ADDR, &i2cData) != HI_ERR_SUCCESS)
        return HI_ERR_FAILURE;

    i2cData.send_buf = NULL;
    i2cData.send_len = 0;
    i2cData.receive_buf = data;
    i2cData.receive_len = data_len;

    return hi_i2c_read(MYIIC_I2C_IDX, MAX30102_READ_ADDR, &i2cData);
}

/******************************************************
 * Part 2: 初始化配置（增加自动重试与检测）
 ******************************************************/

hi_u32 MAX30102_Check_Device_ID(void)
{
    uint8_t id = 0;
    for (int i = 0; i < 3; i++)
    {
        if (MAX30102_I2C_Read(MAX30102_REG_PART_ID, &id, 1) == HI_ERR_SUCCESS && id == 0x15)
            return HI_ERR_SUCCESS;
        hi_udelay(2000);
    }
    printf("MAX30102 ID check failed (0x%02X)\n", id);
    return HI_ERR_FAILURE;
}

hi_u32 MAX30102_Device_Init(MAX30102State *state)
{
    if (state == NULL)
    {
        return HI_ERR_FAILURE;
    }

    // 初始化状态结构体
    MAX30102_Reset_State(state);

    // 复位设备
    if (MAX30102_Device_Reset() != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }

    // 检查设备ID
    if (MAX30102_Check_Device_ID() != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }

    // 清零FIFO指针
    uint8_t zero = 0;
    MAX30102_I2C_Write(MAX30102_REG_FIFO_WR_PTR, &zero, 1);
    MAX30102_I2C_Write(MAX30102_REG_FIFO_RD_PTR, &zero, 1);
    MAX30102_I2C_Write(MAX30102_REG_OVF_COUNTER, &zero, 1);

    // 配置传感器参数
    if (MAX30102_Set_Operation_Mode(MAX30102_MODE_SPO2) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }
    if (MAX30102_Set_Sample_Rate(MAX30102_SAMPLE_RATE_100) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }
    if (MAX30102_Set_Pulse_Width(MAX30102_PULSE_WIDTH_411US) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }
    if (MAX30102_Set_LED_Current(MAX30102_DEFAULT_LED_CURRENT, MAX30102_DEFAULT_LED_CURRENT) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }
    if (MAX30102_Set_FIFO_Config(MAX30102_DEFAULT_SAMPLE_AVG, 1) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }

    printf("MAX30102 init OK\n");
    return HI_ERR_SUCCESS;
}

hi_u32 MAX30102_Device_Reset(void)
{
    uint8_t rst = 0x40;
    if (MAX30102_I2C_Write(MAX30102_REG_MODE_CONFIG, &rst, 1) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }
    hi_udelay(10000);
    return HI_ERR_SUCCESS;
}

/******************************************************
 * Part 3: 数据处理优化（增强鲁棒性）
 ******************************************************/

bool MAX30102_Detect_Finger_Presence(uint32_t red_value, uint32_t ir_value)
{
    return (red_value > MAX30102_MIN_DC_VALUE && ir_value > MAX30102_MIN_DC_VALUE &&
            red_value < MAX30102_MAX_VALID_VALUE && ir_value < MAX30102_MAX_VALID_VALUE);
}

hi_u32 MAX30102_Read_FIFO_Data(uint32_t *red_value, uint32_t *ir_value)
{
    if (red_value == NULL || ir_value == NULL)
    {
        printf("[MAX30102] ERROR: Null pointer in Read_FIFO_Data\n");
        return HI_ERR_FAILURE;
    }

    uint8_t data[6];
    printf("[MAX30102] Reading FIFO data from register 0x%02X...\n", MAX30102_REG_FIFO_DATA);

    hi_u32 read_result = MAX30102_I2C_Read(MAX30102_REG_FIFO_DATA, data, 6);
    if (read_result != HI_ERR_SUCCESS)
    {
        printf("[MAX30102] ERROR: I2C read failed with code 0x%08X\n", read_result);
        return HI_ERR_FAILURE;
    }

    // 打印原始字节数据
    printf("[MAX30102] Raw FIFO data: ");
    for (int i = 0; i < 6; i++)
    {
        printf("0x%02X ", data[i]);
    }
    printf("\n");

    // 解析红光和红外光数据
    *red_value = ((data[0] << 16) | (data[1] << 8) | data[2]) & 0x3FFFF;
    *ir_value = ((data[3] << 16) | (data[4] << 8) | data[5]) & 0x3FFFF;

    printf("[MAX30102] Parsed values - Red: %lu (0x%05lX), IR: %lu (0x%05lX)\n",
           *red_value, *red_value, *ir_value, *ir_value);

    // 数据有效性检查
    bool red_valid = (*red_value <= MAX30102_MAX_VALID_VALUE && *red_value >= MAX30102_MIN_VALID_VALUE);
    bool ir_valid = (*ir_value <= MAX30102_MAX_VALID_VALUE && *ir_value >= MAX30102_MIN_VALID_VALUE);

    if (!red_valid || !ir_valid)
    {
        printf("[MAX30102] WARNING: Data validation failed - ");
        if (!red_valid)
        {
            printf("Red: %lu ", *red_value);
            if (*red_value > MAX30102_MAX_VALID_VALUE)
            {
                printf("(exceeds max %u)", MAX30102_MAX_VALID_VALUE);
            }
            else
            {
                printf("(below min %u)", MAX30102_MIN_VALID_VALUE);
            }
        }
        if (!ir_valid)
        {
            if (!red_valid)
                printf(" | ");
            printf("IR: %lu ", *ir_value);
            if (*ir_value > MAX30102_MAX_VALID_VALUE)
            {
                printf("(exceeds max %u)", MAX30102_MAX_VALID_VALUE);
            }
            else
            {
                printf("(below min %u)", MAX30102_MIN_VALID_VALUE);
            }
        }
        printf("\n");
        return HI_ERR_FAILURE;
    }

    // 更新全局变量
    g_max30102_fifo_red = *red_value;
    g_max30102_fifo_ir = *ir_value;

    // 计算信号强度百分比
    float red_strength = ((float)*red_value / MAX30102_MAX_VALID_VALUE) * 100.0f;
    float ir_strength = ((float)*ir_value / MAX30102_MAX_VALID_VALUE) * 100.0f;

    printf("[MAX30102] SUCCESS: Red: %lu (%.1f%%), IR: %lu (%.1f%%)\n",
           *red_value, red_strength, *ir_value, ir_strength);

    // 检测手指是否存在
    bool finger_detected = MAX30102_Detect_Finger_Presence(*red_value, *ir_value);
    printf("[MAX30102] Finger detection: %s\n", finger_detected ? "DETECTED" : "NOT DETECTED");

    return HI_ERR_SUCCESS;
}

hi_u32 MAX30102_Read_Sensor_Data(MAX30102_SensorData *sensor_data)
{
    if (sensor_data == NULL)
    {
        return HI_ERR_FAILURE;
    }

    uint32_t red, ir;
    if (MAX30102_Read_FIFO_Data(&red, &ir) != HI_ERR_SUCCESS)
    {
        sensor_data->data_valid = false;
        return HI_ERR_FAILURE;
    }

    sensor_data->red_value = red;
    sensor_data->ir_value = ir;
    sensor_data->timestamp = MAX30102_Get_Current_Time();
    sensor_data->data_valid = true;

    // 计算信号质量
    sensor_data->signal_quality = (uint8_t)(MAX30102_Calculate_Signal_Quality(&sensor_data->ir_value, 1) * 100.0f);

    return HI_ERR_SUCCESS;
}

/******************************************************
 * Part 4: 核心算法（平滑 + 自适应阈值）
 ******************************************************/

void MAX30102_Update_Heart_Rate(MAX30102State *state, uint32_t ir_value)
{
    if (state == NULL || !state->finger_detected)
    {
        return;
    }

    // 更新缓冲区
    state->ir_buffer[state->buffer_index] = ir_value;
    state->buffer_index = (state->buffer_index + 1) % MAX30102_DEFAULT_ROLLING_SIZE;

    // 当有足够数据时计算心率
    if (state->buffer_index >= MAX30102_HEART_RATE_WINDOW)
    {
        uint32_t window_data[MAX30102_HEART_RATE_WINDOW];
        uint16_t start_idx = (state->buffer_index + MAX30102_DEFAULT_ROLLING_SIZE - MAX30102_HEART_RATE_WINDOW) % MAX30102_DEFAULT_ROLLING_SIZE;

        for (int i = 0; i < MAX30102_HEART_RATE_WINDOW; i++)
        {
            window_data[i] = state->ir_buffer[(start_idx + i) % MAX30102_DEFAULT_ROLLING_SIZE];
        }

        // 计算心率
        MAX30102_Calculate_Heart_Rate_From_Buffer(window_data, MAX30102_HEART_RATE_WINDOW, state);
    }
}

void MAX30102_Update_SpO2(MAX30102State *state, uint32_t red_value, uint32_t ir_value)
{
    if (state == NULL || !state->finger_detected)
    {
        return;
    }

    // 简单的血氧计算（实际应用需要更复杂的算法）
    float red_ac = (float)red_value - state->red_dc;
    float ir_ac = (float)ir_value - state->ir_dc;

    if (fabsf(ir_ac) > 1.0f)
    {
        float ratio = red_ac / ir_ac;
        float spo2 = MAX30102_SPO2_FORMULA_C2 + MAX30102_SPO2_FORMULA_C1 * ratio;

        // 限制血氧范围
        if (spo2 < MAX30102_MIN_SPO2)
            spo2 = MAX30102_MIN_SPO2;
        if (spo2 > MAX30102_MAX_SPO2)
            spo2 = MAX30102_MAX_SPO2;

        state->spO2 = (uint8_t)spo2;
    }

    // 更新DC分量
    state->red_dc = MAX30102_LPF_ALPHA_DC * state->red_dc + (1 - MAX30102_LPF_ALPHA_DC) * red_value;
    state->ir_dc = MAX30102_LPF_ALPHA_DC * state->ir_dc + (1 - MAX30102_LPF_ALPHA_DC) * ir_value;
}

void MAX30102_Calculate_Heart_Rate_From_Buffer(uint32_t *values, uint16_t count, MAX30102State *state)
{
    if (count < 20 || values == NULL || state == NULL)
    {
        return;
    }

    // 简单的移动平均滤波
    float filtered[count];
    const int window_size = 5;

    for (int i = 0; i < count; i++)
    {
        uint32_t sum = 0;
        int samples = 0;

        for (int j = 0; j < window_size && (i - j) >= 0; j++)
        {
            sum += values[i - j];
            samples++;
        }

        float avg = (float)sum / samples;
        filtered[i] = MAX30102_LPF_ALPHA * avg + (1 - MAX30102_LPF_ALPHA) * state->filter.lp_prev1;
        state->filter.lp_prev1 = filtered[i];
    }

    // 寻找信号幅度
    float min_val = filtered[10];
    float max_val = filtered[10];

    for (int i = 10; i < count; i++)
    {
        if (filtered[i] < min_val)
            min_val = filtered[i];
        if (filtered[i] > max_val)
            max_val = filtered[i];
    }

    float amplitude = max_val - min_val;
    if (amplitude < MAX30102_MIN_AC_VALUE)
    {
        state->quality.signal_quality = 0.0f;
        return; // 信号太弱
    }

    // 峰值检测
    float threshold = min_val + amplitude * 0.4f;
    uint16_t peaks[MAX30102_MAX_PEAKS] = {0};
    uint8_t peak_count = 0;
    uint16_t last_peak = 0;

    for (int i = 11; i < count - 1; i++)
    {
        if (filtered[i] > threshold &&
            filtered[i] > filtered[i - 1] &&
            filtered[i] > filtered[i + 1])
        {

            if (peak_count == 0 || (i - last_peak) >= MAX30102_MIN_PEAK_DISTANCE)
            {
                if (peak_count < MAX30102_MAX_PEAKS)
                {
                    peaks[peak_count++] = i;
                    last_peak = i;
                }
            }
        }
    }

    if (peak_count < 2)
    {
        return;
    }

    // 计算心率
    float total_interval = 0;
    for (int i = 1; i < peak_count; i++)
    {
        total_interval += peaks[i] - peaks[i - 1];
    }

    float avg_interval = total_interval / (peak_count - 1);
    float bpm = (MAX30102_DEFAULT_SAMPLING_RATE / avg_interval) * 60.0f;

    // 心率范围检查
    if (bpm >= state->limits.min_heart_rate && bpm <= state->limits.max_heart_rate)
    {
        state->heart_rate = (uint32_t)(bpm + 0.5f);
        state->quality.signal_quality = 1.0f;
        state->quality.consecutive_valid_readings++;
    }
    else
    {
        state->quality.consecutive_valid_readings = 0;
    }
}

/******************************************************
 * Part 5: 数据更新与显示
 ******************************************************/

void MAX30102_Update_Data(uint32_t red, uint32_t ir, MAX30102State *state)
{
    if (state == NULL)
    {
        return;
    }

    state->finger_detected = MAX30102_Detect_Finger_Presence(red, ir);

    if (!state->finger_detected)
    {
        state->quality.consecutive_valid_readings = 0;
        state->quality.signal_quality = 0.0f;
        return;
    }

    // 更新缓冲区
    state->red_buffer[state->buffer_index] = red;
    state->ir_buffer[state->buffer_index] = ir;
    state->buffer_index = (state->buffer_index + 1) % MAX30102_DEFAULT_ROLLING_SIZE;

    // 更新心率和血氧
    MAX30102_Update_Heart_Rate(state, ir);
    MAX30102_Update_SpO2(state, red, ir);
}

void MAX30102_Read_And_Update_Data(MAX30102State *state)
{
    static uint8_t fail_count = 0;

    if (state == NULL)
    {
        return;
    }

    uint32_t red, ir;
    if (MAX30102_Read_FIFO_Data(&red, &ir) == HI_ERR_SUCCESS)
    {
        fail_count = 0;
        MAX30102_Update_Data(red, ir, state);

        if (state->finger_detected)
        {
            printf("R:%6lu  IR:%6lu  ", red, ir);
            if (state->heart_rate > 0 && state->quality.signal_quality > MAX30102_SIGNAL_QUALITY_THRESHOLD)
            {
                printf("HR:%3d bpm | SpO2:%2d%% | Q:%d%%\n",
                       state->heart_rate, state->spO2,
                       (int)(state->quality.signal_quality * 100));
            }
            else
            {
                printf("HR:--- | SpO2:--- | Q:Low\n");
            }
        }
        else
        {
            printf("No finger detected\n");
        }
    }
    else
    {
        if (++fail_count >= 3)
        {
            state->heart_rate = 0;
            state->spO2 = 0;
            state->quality.signal_quality = 0.0f;
        }
        printf("MAX30102 read failed (%d)\n", fail_count);
    }
}

/******************************************************
 * Part 6: 配置函数
 ******************************************************/

hi_u32 MAX30102_Set_Operation_Mode(MAX30102_OperationMode mode)
{
    uint8_t mode_byte = (uint8_t)mode;
    return MAX30102_I2C_Write(MAX30102_REG_MODE_CONFIG, &mode_byte, 1);
}

hi_u32 MAX30102_Set_Sample_Rate(MAX30102_SampleRate sample_rate)
{
    uint8_t current_config;
    if (MAX30102_I2C_Read(MAX30102_REG_SPO2_CONFIG, &current_config, 1) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }

    uint8_t new_config = (current_config & 0xE3) | ((uint8_t)sample_rate << 2);
    return MAX30102_I2C_Write(MAX30102_REG_SPO2_CONFIG, &new_config, 1);
}

hi_u32 MAX30102_Set_Pulse_Width(MAX30102_PulseWidth pulse_width)
{
    uint8_t current_config;
    if (MAX30102_I2C_Read(MAX30102_REG_SPO2_CONFIG, &current_config, 1) != HI_ERR_SUCCESS)
    {
        return HI_ERR_FAILURE;
    }

    uint8_t new_config = (current_config & 0xFC) | (uint8_t)pulse_width;
    return MAX30102_I2C_Write(MAX30102_REG_SPO2_CONFIG, &new_config, 1);
}

hi_u32 MAX30102_Set_LED_Current(uint8_t red_current, uint8_t ir_current)
{
    hi_u32 result = HI_ERR_SUCCESS;

    result |= MAX30102_I2C_Write(MAX30102_REG_LED1_PA, &red_current, 1);
    result |= MAX30102_I2C_Write(MAX30102_REG_LED2_PA, &ir_current, 1);

    return result;
}

hi_u32 MAX30102_Set_FIFO_Config(uint8_t sample_avg, uint8_t rollover_enable)
{
    if (sample_avg > 7)
    {
        return HI_ERR_FAILURE;
    }

    uint8_t config = (sample_avg << 5) | (rollover_enable << 4) | 0x0F;
    return MAX30102_I2C_Write(MAX30102_REG_FIFO_CONFIG, &config, 1);
}

/******************************************************
 * Part 7: 工具函数
 ******************************************************/

float MAX30102_Calculate_Signal_Quality(const uint32_t *ir_buffer, uint16_t buffer_size)
{
    if (ir_buffer == NULL || buffer_size == 0)
    {
        return 0.0f;
    }

    // 简单的信号质量评估
    uint32_t sum = 0;
    for (int i = 0; i < buffer_size; i++)
    {
        sum += ir_buffer[i];
    }

    float avg = (float)sum / buffer_size;
    return (avg > MAX30102_MIN_DC_VALUE && avg < MAX30102_MAX_VALID_VALUE) ? 1.0f : 0.0f;
}

uint32_t MAX30102_Get_Current_Time(void)
{
    // 返回当前时间戳（毫秒）
    return hi_get_us() / 1000;
}

float MAX30102_Fast_Sqrtf(float x)
{
    if (x <= 0.0f)
        return 0.0f;

    // 快速平方根近似
    float y = x;
    int32_t i = *(int32_t *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (0.5f * x * y * y));

    return x * y;
}

float MAX30102_Fast_Fminf(float a, float b)
{
    return (a < b) ? a : b;
}

float MAX30102_Fast_Fmaxf(float a, float b)
{
    return (a > b) ? a : b;
}

/******************************************************
 * Part 8: 状态管理函数
 ******************************************************/

void MAX30102_Reset_State(MAX30102State *state)
{
    if (state == NULL)
    {
        return;
    }

    memset(state, 0, sizeof(MAX30102State));

    // 设置默认滤波器参数
    state->filter.hp_beta = MAX30102_HPF_BETA;
    state->filter.lp_alpha = MAX30102_LPF_ALPHA;

    // 设置默认限制
    state->limits.min_heart_rate = MAX30102_MIN_HEART_RATE;
    state->limits.max_heart_rate = MAX30102_MAX_HEART_RATE;
    state->limits.min_spO2 = MAX30102_MIN_SPO2;
    state->limits.max_spO2 = MAX30102_MAX_SPO2;

    // 初始化血氧值
    state->spO2 = 98;
}

void MAX30102_Set_HeartRate_Limits(MAX30102State *state, uint16_t min_hr, uint16_t max_hr)
{
    if (state != NULL && min_hr < max_hr)
    {
        state->limits.min_heart_rate = min_hr;
        state->limits.max_heart_rate = max_hr;
    }
}

void MAX30102_Set_SpO2_Limits(MAX30102State *state, uint8_t min_spo2, uint8_t max_spo2)
{
    if (state != NULL && min_spo2 < max_spo2)
    {
        state->limits.min_spO2 = min_spo2;
        state->limits.max_spO2 = max_spo2;
    }
}

/******************************************************
 * Part 9: 调试功能
 ******************************************************/

void MAX30102_Print_Sensor_Data(const MAX30102_SensorData *data)
{
    if (data == NULL)
    {
        printf("Sensor data: NULL\n");
        return;
    }

    printf("Red: %6lu, IR: %6lu, Time: %lu, Q: %3d%%, Valid: %s\n",
           data->red_value, data->ir_value, data->timestamp,
           data->signal_quality, data->data_valid ? "Yes" : "No");
}

void MAX30102_Print_State(const MAX30102State *state)
{
    if (state == NULL)
    {
        printf("MAX30102 State: NULL\n");
        return;
    }

    printf("MAX30102 State:\n");
    printf("  HR: %3d bpm, SpO2: %2d%%, Finger: %s\n",
           state->heart_rate, state->spO2,
           state->finger_detected ? "Yes" : "No");
    printf("  Signal Q: %.1f%%, Stability: %d, Valid Reads: %d\n",
           state->quality.signal_quality * 100.0f,
           state->quality.signal_stability,
           state->quality.consecutive_valid_readings);
    printf("  Buffer Index: %d, DC(R/IR): %.0f/%.0f\n",
           state->buffer_index, state->red_dc, state->ir_dc);
}