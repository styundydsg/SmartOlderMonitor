/**
 * MPU6050 传感器驱动
 * I2C接口实现，适用于Hi3861平台
 *
 * Code By: HelloKun 2021.09.05
 * Optimized Version
 */

#include <stdio.h>
#include <unistd.h>
#include <stddef.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_errno.h"

#include "hi_io.h"
#include "hi_gpio.h"
#include "hi_time.h"
#include "hi_i2c.h"
#include "mpu6050.h"

/* 内部常量定义 */
#define MPU_RESET_DELAY_MS 10
#define MPU_CALIBRATION_SAMPLES 200
#define MPU_CALIBRATION_INTERVAL 10000 // 10ms

/* 全局变量 */
static int16_t g_accel_bias[3] = {0};
static int16_t g_gyro_bias[3] = {0};
static uint8_t g_mpu_initialized = 0;

/**
 * @brief I2C总线扫描
 */
static void MPU_I2C_Scan(void)
{
    hi_i2c_data dummy = {
        .send_buf = NULL,
        .send_len = 0,
        .receive_buf = NULL,
        .receive_len = 0};

    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        uint32_t status = hi_i2c_write(MPU_I2C_IDX, addr << 1, &dummy);
        if (status == IOT_SUCCESS)
        {
            printf("Found device at 0x%02X\n", addr);
        }
        hi_udelay(1000);
    }
}

/**
 * @brief 发送命令到MPU设备
 * @param regAddr 寄存器地址
 * @return 操作结果
 */
static uint32_t MPU_Send_Command(uint8_t regAddr)
{
    uint8_t buffer[] = {regAddr};
    hi_i2c_data i2c_data = {
        .send_buf = buffer,
        .send_len = sizeof(buffer),
        .receive_buf = NULL,
        .receive_len = 0};

    return hi_i2c_write(MPU_I2C_IDX, (MPU_ADDR << 1), &i2c_data);
}

/**
 * @brief 向MPU设备写入数据
 * @param regAddr 寄存器地址
 * @param data 数据指针
 * @param dataLen 数据长度
 * @return 操作结果
 */
uint32_t MPU_Write_Data(uint8_t regAddr, uint8_t *data, uint32_t dataLen)
{
    if (data == NULL || dataLen == 0)
    {
        return IOT_FAILURE;
    }

    // 构造发送缓冲区：寄存器地址 + 数据
    uint8_t buffer[dataLen + 1];
    buffer[0] = regAddr;

    for (uint32_t i = 0; i < dataLen; i++)
    {
        buffer[i + 1] = data[i];
    }

    hi_i2c_data i2c_data = {
        .send_buf = buffer,
        .send_len = sizeof(buffer),
        .receive_buf = NULL,
        .receive_len = 0};

    uint32_t result = hi_i2c_write(MPU_I2C_IDX, (MPU_ADDR << 1), &i2c_data);

    if (result != IOT_SUCCESS)
    {
        printf("MPU write error: reg=0x%02X, status=0x%lX\n", regAddr, result);
    }

    return result;
}

/**
 * @brief 从MPU设备读取数据
 * @param regAddr 寄存器地址
 * @param data 数据缓冲区
 * @param dataLen 数据长度
 * @return 操作结果
 */
uint32_t MPU_Read_Data(uint8_t regAddr, uint8_t *data, uint32_t dataLen)
{
    if (data == NULL || dataLen == 0)
    {
        return IOT_FAILURE;
    }

    // 第一阶段：发送寄存器地址
    uint32_t cmd_result = MPU_Send_Command(regAddr);
    if (cmd_result != IOT_SUCCESS)
    {
        printf("MPU command error: reg=0x%02X, status=0x%lX\n", regAddr, cmd_result);
        return cmd_result;
    }

    hi_udelay(100); // 短暂延迟确保设备准备数据

    // 第二阶段：读取数据
    hi_i2c_data i2c_data = {
        .send_buf = NULL,
        .send_len = 0,
        .receive_buf = data,
        .receive_len = dataLen};

    uint32_t read_result = hi_i2c_read(MPU_I2C_IDX, (MPU_ADDR << 1) | 1, &i2c_data);

    // 调试信息
    if (read_result == IOT_SUCCESS)
    {
        printf("Read reg 0x%02X: ", regAddr);
        for (uint32_t i = 0; i < dataLen; i++)
        {
            printf("0x%02X ", data[i]);
        }
        printf("\n");
    }
    else
    {
        printf("MPU read error: reg=0x%02X, status=0x%lX\n", regAddr, read_result);
    }

    return read_result;
}

/**
 * @brief MPU6050初始化
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Init(void)
{
    if (g_mpu_initialized)
    {
        printf("MPU6050 already initialized\n");
        return IOT_SUCCESS;
    }

    uint8_t device_id = 0;
    uint8_t reset_cmd = 0x80;
    uint8_t wakeup_cmd = 0x00;

    printf("Initializing MPU6050...\n");

    // 扫描I2C总线
    MPU_I2C_Scan();

    // 复位MPU6050
    if (MPU_Write_Data(MPU_PWR_MGMT1_REG, &reset_cmd, 1) != IOT_SUCCESS)
    {
        printf("MPU6050 reset failed\n");
        return 1;
    }
    hi_udelay(MPU_RESET_DELAY_MS * 1000);

    // 唤醒MPU6050
    if (MPU_Write_Data(MPU_PWR_MGMT1_REG, &wakeup_cmd, 1) != IOT_SUCCESS)
    {
        printf("MPU6050 wakeup failed\n");
        return 1;
    }

    // 配置传感器
    MPU_Set_Gyro_Fsr(MPU_GYRO_FS_2000);
    MPU_Set_Accel_Fsr(MPU_ACCEL_FS_2G);
    MPU_Set_Rate(50);

    // 关闭不需要的功能
    uint8_t zero = 0x00;
    MPU_Write_Data(MPU_INT_EN_REG, &zero, 1);    // 关闭所有中断
    MPU_Write_Data(MPU_USER_CTRL_REG, &zero, 1); // I2C主模式关闭
    MPU_Write_Data(MPU_FIFO_EN_REG, &zero, 1);   // 关闭FIFO

    uint8_t int_config = 0x80;
    MPU_Write_Data(MPU_INTBP_CFG_REG, &int_config, 1); // INT引脚低电平有效

    // 读取器件ID验证连接
    if (MPU_Read_Data(MPU_DEVICE_ID_REG, &device_id, 1) != IOT_SUCCESS)
    {
        printf("Failed to read MPU6050 device ID\n");
        return 1;
    }

    printf("MPU6050 Device ID: 0x%02X\n", device_id);

    if (device_id == 0x68 || device_id == 0x70)
    {
        uint8_t clk_sel = 0x01;
        MPU_Write_Data(MPU_PWR_MGMT1_REG, &clk_sel, 1); // 设置CLKSEL, PLL X轴为参考
        MPU_Write_Data(MPU_PWR_MGMT2_REG, &zero, 1);    // 加速度与陀螺仪都工作
        MPU_Set_Rate(50);                               // 设置采样率为50Hz

        g_mpu_initialized = 1;
        printf("MPU6050 initialized successfully\n");

        // 执行校准
        MPU_Calibrate();

        return IOT_SUCCESS;
    }
    else
    {
        printf("MPU6050 connection failed! Expected ID: 0x68/0x70, Got: 0x%02X\n", device_id);
        return 1;
    }
}

/**
 * @brief 设置陀螺仪量程
 */
uint8_t MPU_Set_Gyro_Fsr(mpu_gyro_fs_t fsr)
{
    if (fsr > MPU_GYRO_FS_2000)
    {
        fsr = MPU_GYRO_FS_2000;
    }

    uint8_t config = (uint8_t)fsr << 3;
    return (MPU_Write_Data(MPU_GYRO_CFG_REG, &config, 1) == IOT_SUCCESS) ? 0 : 1;
}

/**
 * @brief 设置加速度计量程
 */
uint8_t MPU_Set_Accel_Fsr(mpu_accel_fs_t fsr)
{
    if (fsr > MPU_ACCEL_FS_16G)
    {
        fsr = MPU_ACCEL_FS_16G;
    }

    uint8_t config = (uint8_t)fsr << 3;
    return (MPU_Write_Data(MPU_ACCEL_CFG_REG, &config, 1) == IOT_SUCCESS) ? 0 : 1;
}

/**
 * @brief 设置数字低通滤波器
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data;

    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;

    return (MPU_Write_Data(MPU_CFG_REG, &data, 1) == IOT_SUCCESS) ? 0 : 1;
}

/**
 * @brief 设置采样率
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;

    uint8_t data = 1000 / rate - 1;
    uint8_t result = (MPU_Write_Data(MPU_SAMPLE_RATE_REG, &data, 1) == IOT_SUCCESS) ? 0 : 1;

    // 自动设置DLPF为采样率的一半
    if (result == 0)
    {
        result = MPU_Set_LPF(rate / 2);
    }

    return result;
}

/**
 * @brief 获取温度数据
 */
short MPU_Get_Temperature(void)
{
    uint8_t buf[2] = {0};
    short raw_temp;
    float temperature;

    if (MPU_Read_Data(MPU_TEMP_OUTH_REG, buf, 2) == IOT_SUCCESS)
    {
        raw_temp = ((uint16_t)buf[0] << 8) | buf[1];
        temperature = 36.53f + ((float)raw_temp) / 340.0f;
        return (short)(temperature * 100); // 扩大100倍返回
    }

    return 0;
}

/**
 * @brief 获取陀螺仪原始数据
 */
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6] = {0};

    if (gx == NULL || gy == NULL || gz == NULL)
    {
        return 1;
    }

    uint8_t result = MPU_Read_Data(MPU_GYRO_XOUTH_REG, buf, 6);
    if (result == IOT_SUCCESS)
    {
        *gx = ((int16_t)buf[0] << 8) | buf[1];
        *gy = ((int16_t)buf[2] << 8) | buf[3];
        *gz = ((int16_t)buf[4] << 8) | buf[5];
        return 0;
    }

    return 1;
}

/**
 * @brief 获取加速度计原始数据
 */
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6] = {0};

    if (ax == NULL || ay == NULL || az == NULL)
    {
        return 1;
    }

    uint8_t result = MPU_Read_Data(MPU_ACCEL_XOUTH_REG, buf, 6);
    if (result == IOT_SUCCESS)
    {
        *ax = ((int16_t)buf[0] << 8) | buf[1];
        *ay = ((int16_t)buf[2] << 8) | buf[3];
        *az = ((int16_t)buf[4] << 8) | buf[5];
        return 0;
    }

    return 1;
}

/**
 * @brief 获取所有传感器数据
 */
uint8_t MPU_Get_All_Data(mpu_data_t *data)
{
    if (data == NULL)
    {
        return 1;
    }

    data->temperature = MPU_Get_Temperature();

    return (MPU_Get_Accelerometer(&data->accel_x, &data->accel_y, &data->accel_z) == 0 &&
            MPU_Get_Gyroscope(&data->gyro_x, &data->gyro_y, &data->gyro_z) == 0)
               ? 0
               : 1;
}

/**
 * @brief 获取转换后的加速度数据(g)
 */
uint8_t MPU_Get_Accelerometer_G(float accel_g[3])
{
    short raw_data[3];

    if (accel_g == NULL)
    {
        return 1;
    }

    if (MPU_Get_Accelerometer(&raw_data[0], &raw_data[1], &raw_data[2]) == 0)
    {
        // 零偏校准
        raw_data[0] -= g_accel_bias[0];
        raw_data[1] -= g_accel_bias[1];
        raw_data[2] -= g_accel_bias[2];

        // 转换为g值(±2g量程)
        accel_g[0] = raw_data[0] / 16384.0f;
        accel_g[1] = raw_data[1] / 16384.0f;
        accel_g[2] = raw_data[2] / 16384.0f;

        return 0;
    }

    return 1;
}

/**
 * @brief 获取转换后的陀螺仪数据(°/s)
 */
uint8_t MPU_Get_Gyroscope_DPS(float gyro_dps[3])
{
    short raw_data[3];

    if (gyro_dps == NULL)
    {
        return 1;
    }

    if (MPU_Get_Gyroscope(&raw_data[0], &raw_data[1], &raw_data[2]) == 0)
    {
        // 零偏校准
        raw_data[0] -= g_gyro_bias[0];
        raw_data[1] -= g_gyro_bias[1];
        raw_data[2] -= g_gyro_bias[2];

        // 转换为°/s(±2000°/s量程)
        gyro_dps[0] = raw_data[0] / 16.4f;
        gyro_dps[1] = raw_data[1] / 16.4f;
        gyro_dps[2] = raw_data[2] / 16.4f;

        return 0;
    }

    return 1;
}

/**
 * @brief 传感器校准(需水平静止放置)
 */
void MPU_Calibrate(void)
{
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    printf("Starting MPU6050 calibration...\n");
    printf("Please keep the sensor stationary and level\n");

    for (uint16_t i = 0; i < MPU_CALIBRATION_SAMPLES; i++)
    {
        short ax, ay, az, gx, gy, gz;

        if (MPU_Get_Accelerometer(&ax, &ay, &az) == 0)
        {
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
        }

        if (MPU_Get_Gyroscope(&gx, &gy, &gz) == 0)
        {
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
        }

        hi_udelay(MPU_CALIBRATION_INTERVAL);

        if ((i % 50) == 0)
        {
            printf("Calibration progress: %d/%d\n", i, MPU_CALIBRATION_SAMPLES);
        }
    }

    // 计算零偏
    g_accel_bias[0] = ax_sum / MPU_CALIBRATION_SAMPLES;
    g_accel_bias[1] = ay_sum / MPU_CALIBRATION_SAMPLES;
    g_accel_bias[2] = (az_sum / MPU_CALIBRATION_SAMPLES) - 16384; // Z轴减去1g

    g_gyro_bias[0] = gx_sum / MPU_CALIBRATION_SAMPLES;
    g_gyro_bias[1] = gy_sum / MPU_CALIBRATION_SAMPLES;
    g_gyro_bias[2] = gz_sum / MPU_CALIBRATION_SAMPLES;

    printf("Calibration completed:\n");
    printf("Accel Bias: X=%d, Y=%d, Z=%d\n",
           g_accel_bias[0], g_accel_bias[1], g_accel_bias[2]);
    printf("Gyro Bias: X=%d, Y=%d, Z=%d\n",
           g_gyro_bias[0], g_gyro_bias[1], g_gyro_bias[2]);
}

/**
 * @brief 打印原始传感器数据
 */
void MPU_Print_Raw_Data(void)
{
    mpu_data_t data;

    if (MPU_Get_All_Data(&data) == 0)
    {
        printf("=== MPU6050 Raw Data ===\n");
        printf("Temperature: %.1f C\n", data.temperature / 100.0f);
        printf("Accelerometer: X=%6d, Y=%6d, Z=%6d\n",
               data.accel_x, data.accel_y, data.accel_z);
        printf("Gyroscope:     X=%6d, Y=%6d, Z=%6d\n",
               data.gyro_x, data.gyro_y, data.gyro_z);
        printf("========================\n");
    }
    else
    {
        printf("Failed to read MPU6050 data\n");
    }
}

/**
 * @brief 打印转换后的传感器数据
 */
void MPU_Print_Converted_Data(void)
{
    float accel_g[3], gyro_dps[3];
    short temperature = MPU_Get_Temperature();

    if (MPU_Get_Accelerometer_G(accel_g) == 0 &&
        MPU_Get_Gyroscope_DPS(gyro_dps) == 0)
    {
        printf("=== MPU6050 Converted Data ===\n");
        printf("Temperature: %.1f C\n", temperature / 100.0f);
        printf("Accelerometer: X=%7.3fg, Y=%7.3fg, Z=%7.3fg\n",
               accel_g[0], accel_g[1], accel_g[2]);
        printf("Gyroscope:     X=%7.2f deg/s, Y=%7.2f deg/s, Z=%7.2f deg/s\n",
               gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("==============================\n");
    }
    else
    {
        printf("Failed to read MPU6050 converted data\n");
    }
}