/***
 * MPU6050 接入Hi3861
 * I2C通信，使用Hi3861的I2C0 (I2C1保留给OLED)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_errno.h"

#include "hi_io.h"
#include "hi_gpio.h"
#include "hi_time.h"
#include "hi_i2c.h"

/* MPU6050 寄存器定义 */
#define MPU_SELF_TESTX_REG 0x0D  // 自检寄存器X
#define MPU_SELF_TESTY_REG 0x0E  // 自检寄存器Y
#define MPU_SELF_TESTZ_REG 0x0F  // 自检寄存器Z
#define MPU_SELF_TESTA_REG 0x10  // 自检寄存器A
#define MPU_SAMPLE_RATE_REG 0x19 // 采样频率分频器
#define MPU_CFG_REG 0x1A         // 配置寄存器
#define MPU_GYRO_CFG_REG 0x1B    // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG 0x1C   // 加速度计配置寄存器
#define MPU_MOTION_DET_REG 0x1F  // 运动检测阈值设置寄存器
#define MPU_FIFO_EN_REG 0x23     // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG 0x24 // IIC主机控制寄存器
#define MPU_INT_EN_REG 0x38      // 中断使能寄存器
#define MPU_INTBP_CFG_REG 0x37   // INT引脚/旁路设置寄存器

#define MPU_ACCEL_XOUTH_REG 0x3B // 加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG 0x3C // 加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG 0x3D // 加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG 0x3E // 加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG 0x3F // 加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG 0x40 // 加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG 0x41 // 温度值高八位寄存器
#define MPU_TEMP_OUTL_REG 0x42 // 温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG 0x43 // 陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG 0x44 // 陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG 0x45 // 陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG 0x46 // 陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG 0x47 // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG 0x48 // 陀螺仪值,Z轴低8位寄存器

#define MPU_USER_CTRL_REG 0x6A // 用户控制寄存器
#define MPU_PWR_MGMT1_REG 0x6B // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG 0x6C // 电源管理寄存器2
#define MPU_DEVICE_ID_REG 0x75 // 器件ID寄存器

/* 硬件配置 */
#define MPU_ADDR 0x68        // I2C地址(AD0接地)
#define MPU_SDA_IO 13        // SDA引脚
#define MPU_SCL_IO 14        // SCL引脚
#define MPU_I2C_IDX 0        // Hi3861 I2C0
#define MPU_I2C_BAUDRATE 400 // I2C工作频率(kHz)

/* 量程定义 */
typedef enum
{
    MPU_ACCEL_FS_2G = 0, // ±2g
    MPU_ACCEL_FS_4G = 1, // ±4g
    MPU_ACCEL_FS_8G = 2, // ±8g
    MPU_ACCEL_FS_16G = 3 // ±16g
} mpu_accel_fs_t;

typedef enum
{
    MPU_GYRO_FS_250 = 0,  // ±250°/s
    MPU_GYRO_FS_500 = 1,  // ±500°/s
    MPU_GYRO_FS_1000 = 2, // ±1000°/s
    MPU_GYRO_FS_2000 = 3  // ±2000°/s
} mpu_gyro_fs_t;

/* 数据结构 */
typedef struct
{
    short accel_x;
    short accel_y;
    short accel_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short temperature;
} mpu_data_t;

/* 函数声明 */

/**
 * @brief MPU6050初始化
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Init(void);

/**
 * @brief 设置加速度计量程
 * @param fs 量程选择
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Set_Accel_Fsr(mpu_accel_fs_t fs);

/**
 * @brief 设置陀螺仪量程
 * @param fs 量程选择
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Set_Gyro_Fsr(mpu_gyro_fs_t fs);

/**
 * @brief 设置低通滤波器
 * @param lpf 滤波器频率(Hz)
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Set_LPF(uint16_t lpf);

/**
 * @brief 设置采样率
 * @param rate 采样率(Hz)
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Set_Rate(uint16_t rate);

/**
 * @brief 读取温度数据
 * @return 温度原始值
 */
short MPU_Get_Temperature(void);

/**
 * @brief 读取陀螺仪数据
 * @param gx X轴角速度输出
 * @param gy Y轴角速度输出
 * @param gz Z轴角速度输出
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz);

/**
 * @brief 读取加速度计数据
 * @param ax X轴加速度输出
 * @param ay Y轴加速度输出
 * @param az Z轴加速度输出
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az);

/**
 * @brief 读取所有传感器数据
 * @param data 数据输出结构体
 * @return 0-成功, 其他-错误码
 */
uint8_t MPU_Get_All_Data(mpu_data_t *data);

/**
 * @brief 打印原始传感器数据
 */
void MPU_Print_Raw_Data(void);

/**
 * @brief I2C写入数据
 * @param regAddr 寄存器地址
 * @param data 数据缓冲区
 * @param dataLen 数据长度
 * @return 0-成功, 其他-错误码
 */
uint32_t MPU_Write_Data(uint8_t regAddr, uint8_t *data, uint32_t dataLen);

/**
 * @brief I2C读取数据
 * @param regAddr 寄存器地址
 * @param data 数据缓冲区
 * @param dataLen 数据长度
 * @return 0-成功, 其他-错误码
 */
uint32_t MPU_Read_Data(uint8_t regAddr, uint8_t *data, uint32_t dataLen);

#endif /* MPU6050_H */