#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "cmsis_os2.h"
#include "ohos_init.h"

#include "hi_io.h"
#include "hi_gpio.h"
#include "hi_i2c.h"
#include "hi_time.h"

#include "drivers/sensors/max30102/max30102.h"
#include "used_iot_cd.h"

// 线程回调入口函数
void TCPClientTask(void *argument)
{
    SensorData sensor_data = {0};
    uint32_t red, ir;
    MAX30102State max_state;

    // 初始化I2C引脚
    IoTGpioInit(HI_IO_NAME_GPIO_13);
    IoTGpioInit(HI_IO_NAME_GPIO_14);

    hi_io_set_func(HI_IO_NAME_GPIO_13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(HI_IO_NAME_GPIO_14, HI_IO_FUNC_GPIO_14_I2C0_SCL);
    hi_i2c_init(HI_I2C_IDX_0, 400000); // 400kHz I2C速率
    
    // 初始化MAX30102传感器
    if (MAX30102_Device_Init(&max_state) != 0)
    {
        printf("MAX30102 initialization failed!\n");
        return;
    }

    if (MPU_Init() != 0)
    {
        printf("MPU6050 initialization failed!\n");
        return;
    }

    MPU_Calibrate();

    printf("SmartOlderMonitor started successfully\n");


    

    // 主循环
    while (1) {
        MAX30102_Read_And_Update_Data(&max_state);

        sensor_data.heart_rate = max_state.heart_rate;
        sensor_data.spO2 = max_state.spO2;

        printf("HR: %3d bpm, SpO2: %2d%%\n", 
               sensor_data.heart_rate, 
               sensor_data.spO2);

        MPU_Print_Raw_Data();

        // 打印转换后的物理量数据
        MPU_Print_Converted_Data();

        osDelay(50); // 50ms周期
    }
}

static void SensorDataUploadDemo(void)
{
    osThreadAttr_t attr = {
        .name = "TCPClientTask",
        .stack_size = 8192,
        .priority = osPriorityNormal,
    };

    if (osThreadNew(TCPClientTask, 0, &attr) == 0)
    {
        printf("Failed to create TCPClientTask!\n");
    }
}

SYS_RUN(SensorDataUploadDemo);
