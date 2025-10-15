/**
 * 
 *    采集MPU 6050 原始数据 
 *处理通过串口发出，使用python脚本解析后执行相应动作
 * Code By： HelloKun  2021.09.06
 * 
 */
#include <stdio.h>
#include <unistd.h>
#include "math.h"
#include "stdbool.h"
#include "ohos_init.h"
#include "cmsis_os2.h"


#include "hi_io.h"   //上拉、复用
#include "hi_gpio.h" //hi_gpio_set_dir()、hi_gpio_set(get)_output(input)_val()
#include "iot_gpio.h"//gpioInit
#include "hi_time.h"
#include "hi_i2c.h"
#include "mpu6050.h"

// 定义检测参数（需要根据实际测试调整）
#define FALL_IMPACT_THRESHOLD 2.5    // 冲击阈值(g)
#define FALL_ANGLE_THRESHOLD  45.0   // 姿态角阈值(度)
#define FREEFALL_THRESHOLD    0.3    // 自由落体阈值(g)
#define IMPACT_TIME_WINDOW    500    // 冲击时间窗口(ms)
#define POSTURE_TIME_WINDOW   2000   // 姿态时间窗口(ms)


// 状态结构体
typedef struct {
    bool freefall_detected;
    bool impact_detected;
    uint64_t freefall_time;
    uint64_t impact_time;
} FallDetectionState;

// 全局状态变量
static FallDetectionState fall_state = {0};

// 计算三轴加速度向量长度（单位：g）
static float calculate_acceleration_magnitude(short ax, short ay, short az)
{
    // MPU6050灵敏度：16384 LSB/g (±2g量程)
    const float scale = 16384.0f;
    float gx = ax / scale;
    float gy = ay / scale;
    float gz = az / scale;
    return sqrt(gx*gx + gy*gy + gz*gz);
}

// 计算俯仰角（单位：度）
static float calculate_pitch(short ax, short ay, short az)
{
    const float scale = 16384.0f;
    return atan2f(-ax / scale, sqrt(ay*ay + az*az) / scale) * 180.0 / M_PI;
}

// 检测摔倒函数
static bool detect_fall(short ax, short ay, short az, uint64_t timestamp)
{
    float accel_mag = calculate_acceleration_magnitude(ax, ay, az);
    float pitch = calculate_pitch(ax, ay, az);
    
    // 1. 自由落体检测 (加速度接近0g)
    if (accel_mag < FREEFALL_THRESHOLD) {
        fall_state.freefall_detected = true;
        fall_state.freefall_time = timestamp;
        return false;
    }
    
    // 2. 冲击检测 (高g值)
    if (accel_mag > FALL_IMPACT_THRESHOLD) {
        fall_state.impact_detected = true;
        fall_state.impact_time = timestamp;
    }
    
    // 3. 姿态检测 (身体平躺)
    bool posture_detected = fabs(pitch) > FALL_ANGLE_THRESHOLD;
    
    // 4. 组合条件检测：自由落体后发生冲击，然后姿态改变
    if (fall_state.freefall_detected && 
        fall_state.impact_detected &&
        posture_detected)
    {
        // 检查时间窗口
        bool impact_in_window = (fall_state.impact_time - fall_state.freefall_time) < IMPACT_TIME_WINDOW;
        bool posture_in_window = (timestamp - fall_state.impact_time) < POSTURE_TIME_WINDOW;
        
        if (impact_in_window && posture_in_window) {
            // 重置状态
            memset(&fall_state, 0, sizeof(fall_state));
            return true;
        }
    }
    
    // 超时重置
    if (timestamp - fall_state.freefall_time > POSTURE_TIME_WINDOW) {
        memset(&fall_state, 0, sizeof(fall_state));
    }
    
    return false;
}


static void MPUTask(void *arg)
{
    (void) arg;
    short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	uint8_t temp;					//温度	  
    uint64_t last_time = hi_get_us() / 1000;  

    IoTGpioInit(MPU_SDA_IO13);
    IoTGpioInit(MPU_SCL_IO14);
    IoTGpioSetDir(MPU_SDA_IO13, IOT_GPIO_DIR_OUT); //MPU_SDA_IO13
    IoTGpioSetDir(MPU_SCL_IO14, IOT_GPIO_DIR_OUT); //MPU_SCL_IO14
    hi_io_set_func(MPU_SDA_IO13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(MPU_SCL_IO14, HI_IO_FUNC_GPIO_14_I2C0_SCL);
    hi_i2c_init(HI_I2C_IDX_0, MPU_I2C_BAUDRATE);


    IoTGpioInit(9);
    IoTGpioSetDir(9,IOT_GPIO_DIR_OUT);

    //IoTGpioSetOutputVal(9,0);// 只要一直初始化不成功，则灯不闪
    
    MPU_Init(); //返回0 成功
    while (MPU_Init())
    {
        printf("MPU Initialize Faild \n");
    }
    
    
    while(1) //连接成功 led 一直闪烁
    {   
        short aacx, aacy, aacz;
        short gyrox, gyroy, gyroz;

        static char text[128] = {0};
        temp = MPU_Get_Temperature();
        MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
        MPU_Get_Accelerometer(&aacx,&aacy,&aacz);

        uint64_t current_time = hi_get_us() / 1000;

        // 摔倒检测
        if (detect_fall(aacx, aacy, aacz, current_time)) {
            printf("FALL DETECTED! Sending alert...\n");
            // 这里添加报警逻辑：发送消息/触发蜂鸣器/拨打电话等
        }

        // 原始数据输出（可选，调试时使用）
        static uint32_t counter = 0;
        if (counter++ % 10 == 0) { // 每10次循环输出一次
            printf("Acc: %6d,%6d,%6d | ", aacx, aacy, aacz);
            printf("Pitch: %.1f°\n", calculate_pitch(aacx, aacy, aacz));
        }

        // 控制采样频率 (50Hz)
        usleep(20 * 1000);

    //     if(aacx<4000 && aacx>1000)
    //     {
    //         //printf("a");
    //         if(aacy<-2000)
    //         {
    //          printf("al\n");
    //         }
    //         else if(aacy>3000)
    //         {
    //          printf("ar\n");
    //         }
    //         else printf("a\n");
    //     }
    //     else if(aacx>4000 && aacx<7000) 
    //     {
    //         if(aacy<-2000)
    //         {
    //          printf("zl\n");
    //         }
    //         else if(aacy>3000)
    //         {
    //          printf("zr\n");
    //         }
    //         else printf("z\n");
    //     }
    //     else if((aacx>7000) && (aacy<1000) && (aacy>-1000))
    //     {
    //         printf("t\n");
    //     } 
    //     else 
    //     {
    //         printf("t\n");
    //     }
    //     printf("temp: %d\n",temp);
         printf("gyrox: %d\n",gyrox);
         printf("gyroy: %d\n",gyroy);
         printf("gyroz: %d\n",gyroz); 
        
         printf("aacx: %d\n",aacx);
         printf("aacy: %d\n",aacy);
         printf("aacz: %d\n",aacz);

    //     //snprintf(text, sizeof(text), "temp:%.3f \n", temp);
    //     //OledShowString(0, 1, text, 1);
    //     //OledShowString(0,1,(char)(temp),1);

    //     IoTGpioSetOutputVal(9,0); 
    //     usleep(30000); 
        
    //     IoTGpioSetOutputVal(9,1); 
         usleep(30000); 
        
    //     //Print_Original_MPU_Data();
        
         usleep(500*1000);
    // }
    }
}

static void MPUDemo(void)
{
    osThreadAttr_t attr;

    attr.name = "MPUTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;

    if (osThreadNew(MPUTask, NULL, &attr) == NULL) {
        printf("[MPUDemo] Falied to create MPUTask!\n");
    }
}

//SYS_RUN(MPUDemo);


