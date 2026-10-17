/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define CONFIG_PWM_USING_V151 1
#define CHIP_WS63 1
#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_watchdog.h"
#include "iot_gpio_ex.h"
#include "iot_pwm.h"
#include "iot_errno.h"
#include "pwm.h"
#include "pinctrl.h"
#include "pinctrl_porting.h"
#include "pwm_porting.h"
#include "tcxo.h"
#include "gpio.h"
#include "osal_debug.h"
#include "data_cloud.h"
#include "mqtt_cloud.h"
#include "wifi_connecter.h"
#include <stdlib.h> 
#include <unistd.h>
#include "iot_watchdog.h"
#include "tcxo.h"

// MQTT 配置
#define CLIENTID "695db7b87f2e6c302f47e29f_motor1_0_0_2026010701"
#define NTP_SERVER_USERNAME "695db7b87f2e6c302f47e29f_motor1"
#define NTP_SERVER_PASSWORD "4c3c6e6c4bab2e15e31f1cb8fb7372138c33480dc9246359216d18d409353d82"


#define GPIO_ID_8 8

#define PWM_GPIO                   8
#define PWM_PIN_MODE               1
#define PWM_CHANNEL                0
#define PWM_CHANNEL_REVERSE        4
#define PWM_CHANNEL_FORWARD   0   // 正转: 通道0
#define PWM_CHANNEL_REVERSE   4   // 反转: 通道4
#define GPIO_FORWARD       8
#define GPIO_FORWARD_FUNC  1
#define GPIO_REVERSE       12
#define GPIO_REVERSE_FUNC  1
#define PWM_GROUP_ID               0
#define TEST_MAX_TIMES             5
#define DALAY_MS                   100
#define TEST_TCXO_DELAY_1000MS     1000
#define PWM_TASK_STACK_SIZE        0x1000
#define PWM_TASK_PRIO              (osPriority_t)(17)

#define IOT_PWM_CFG_OFFEST_TIME    0
#define IOT_PWM_CFG_REPEAT_CYCLE   1000
#define IOT_PWM_CFG_REPEAT_STATE   true
#define PWM_FREQ                   20000
/**
 * 5V
 * GND
 * 3.3V
 * GND
 * 12 > 10  SPINSS
 * 10 > 7   SPICLK
 * 9 > 6   SPIMOSI 
 * 11 > 5   SPIMISO
 * 13 > 9
 * 2 > 2
 * 14 > 0
 * 7 > 8
 * 8 > 12
 * 3 > UART0T
 * 4 > UART0R
 * 1 > 14
 * 0 > 13
 * 6 > UART1T
 * 5 > UART1R
 * GND
 * 
 */


/*-----------------------------------------------------------
*  PWM 初始化调整转速部分
-----------------------------------------------------------*/
volatile bool directionChanged = true;
volatile bool speedChanged = true;
typedef enum {
    // DCmotor = 0,
    // StepperMotor
    Dcmotor = 1,
    StepperMotor = 2
} MotorStatus;

MotorStatus cmdMotorStatus = 0;

typedef struct {
    unsigned int port;     // 端口号
    unsigned int speed;    // 速度值
    int motordir;          // 转向（1 正转, 2 反转）
} MotorConfig;

MotorConfig motorConfig = { 
        .port = 0,        // 初始化端口号
        .speed = 60,      // 初始化速度
        .motordir = 1     // 初始化转向（1 正转, 2 反转）
};

// 新增变量用于循环控制
static int cycle_count = 0;
static int current_speed = 50;  // 初始速度
static int direction = 1;       // 1: 正转, 2: 反转
static bool dc_motor_running = false;


static errcode_t pwm_sample_callback(uint8_t channel)
{
    osal_printk("PWM %d, cycle done. \r\n", channel);
    return ERRCODE_SUCC;
}

void dc_motor_init(void) {
    printf("dc_motor_init called \n");

    // 初始化 GPIO8 和 GPIO12
    IoTGpioInit(8);
    IoTGpioSetDir(8, IOT_GPIO_DIR_OUT);
    IoSetFunc(8, 1);   //PWM

    IoTGpioInit(12);
    IoTGpioSetDir(12, IOT_GPIO_DIR_OUT);
    IoSetFunc(12, IOT_IO_FUNC_GPIO_12_GPIO);
}

/**
 * @brief 启动PWM
 * @param port 端口号
 * @param duty 转速 1 - 99
 * @return 无
 */
static void IoTPwmStart_new(unsigned int port, unsigned int duty, unsigned int freq) {
    printf("IotPwmStart_new called \n");
    // 限制 duty 范围在 1-99
    duty = (duty < 1) ? 1 : ((duty > 99) ? 99 : duty);

    uint32_t clk_freq = uapi_pwm_get_frequency((uint8_t)port);
    uint32_t div_num = (clk_freq + freq / 2) / freq;   // 计算分频数
    
    // 根据 duty 计算高低电平时间
    uint32_t high_time = div_num * duty / 100;
    uint32_t low_time = div_num - high_time;

    pwm_config_t cfg = {
        .low_time = low_time,
        .high_time = high_time,
        .offset_time = IOT_PWM_CFG_OFFEST_TIME,
        .cycles = IOT_PWM_CFG_REPEAT_CYCLE,
        .repeat = IOT_PWM_CFG_REPEAT_STATE
    };

    uapi_pwm_open(port, &cfg);

    uapi_tcxo_delay_ms((uint32_t)TEST_TCXO_DELAY_1000MS);
    uapi_pwm_unregister_interrupt(port);
    uapi_pwm_register_interrupt(port, pwm_sample_callback);

    uint8_t channel_id = port;
    /* channel_id can also choose to configure multiple channels, and the third parameter also needs to be adjusted
        accordingly. */
    uapi_pwm_set_group(PWM_GROUP_ID, &channel_id, 1);
    /* Here you can also call the uapi_pwm_start interface to open each channel individually. */
    uapi_pwm_start_group(PWM_GROUP_ID);

    uapi_tcxo_delay_ms((uint32_t)TEST_TCXO_DELAY_1000MS);
    printf("IotPwmStart_new called success and end\r\n");
}



void dc_motor_set_direction(MotorConfig *config) {
    printf("dc_motor_set_direction called \n");

    uapi_pwm_close(config->port);
    if(config->motordir == 1) {   // 正转
        config->port = 0;
        printf("dc_motor set to forward\nrun ....\n");
        IoTGpioSetOutputVal(8,1);
        IoTGpioSetOutputVal(12,0);
        
        IoSetFunc(8, 1);   // 设置GPIO8为PWM
        IoSetFunc(12, IOT_IO_FUNC_GPIO_12_GPIO);              
    }
    else if(config->motordir == 2) {    // 反转
        config->port = 4;
        printf("dc_motor set to reverse\nrev ....\n");
        IoTGpioSetOutputVal(8,0);
        IoTGpioSetOutputVal(12,1);

        IoSetFunc(8, IOT_IO_FUNC_GPIO_8_GPIO);   // 设置GPIO8为PWM
        IoSetFunc(12, 1);
    }
    else {   // 停止
        IoTGpioSetOutputVal(8,0);
        IoTGpioSetOutputVal(12,0);
        printf("dc_motor stopped\n");        
    }
}

void dc_motor_set_speed(MotorConfig *config) {
    printf("dc_motor_set_speed called, speed %d \n", config->speed);
    // 确保 speed 在 1 到 99 之间
    if (config->speed < 1) config->speed = 1;
    if (config->speed > 99) config->speed = 99;
    IoTPwmStart_new(config->port, config->speed, PWM_FREQ);
}

void motor_run(MotorConfig *config) {
    // 每次都设置方向和速度
    dc_motor_set_direction(config);
    dc_motor_set_speed(config);
    
    TaskMsleep(5000);
}

/*-----------------------------------------------------------
   步进电机旋转部分
-----------------------------------------------------------*/

unsigned char CodeArr[8] = 
{ //2相激励
  0xC, 0x6, 0x3, 0x9, 0xC, 0x6, 0x3, 0x9
};

void stepper_motor_init(void) {
    printf("moto thread running...\n");

    IoTGpioInit(8);
    IoTGpioInit(12);
    IoTGpioInit(6);
    IoTGpioInit(7);
    IoTGpioInit(5);
    IoTGpioInit(10);
    
    // 设置GPIO功能
    IoSetFunc(8, 0);
    IoSetFunc(12, 0); 
    IoSetFunc(6, 0);
    IoSetFunc(7, 0);
    IoSetFunc(5, 4);  // GPIO5 特殊处理，功能值设置为4
    IoSetFunc(10, 0);

    // 设置GPIO方向
    IoTGpioSetDir(8, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(12, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(6, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(7, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(5, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(10, IOT_GPIO_DIR_OUT);

    // 3. 统一将所有GPIO的输出设置为0低电平
    IoTGpioSetOutputVal(8, 0);
    IoTGpioSetOutputVal(12, 0);
    IoTGpioSetOutputVal(6, 0);
    IoTGpioSetOutputVal(7, 0);
    IoTGpioSetOutputVal(5, 0);
    IoTGpioSetOutputVal(10, 0);
}

void stepper_motor_run(void) {
    TaskMsleep(1000);//等待uart2（gpio7,gpio8）初始化完成后，再设置GPIO7为gpio功能
    printf("[stepper_motor] Starting...\n");

    IoTPwmStop(0);
    
    int idx = 0;   
    while(cmdMotorStatus == StepperMotor) {
        unsigned char code = CodeArr[idx];
        IoTGpioSetOutputVal(6, code & 0x1);
        code = code >> 1;
        IoTGpioSetOutputVal(7, code & 0x1); 
        code = code >> 1;
        IoTGpioSetOutputVal(5, code & 0x1);
        code = code >> 1;
        IoTGpioSetOutputVal(10, code & 0x1);
        code = code >> 1;
        printf("sleep");
        usleep(1000);
        idx = (idx + 1) % 8;
    }
}

// 修改主任务中的调用方式
void MotorControlTask(void) {
    fflush(stdout);
    printf("Motor control starting...\r\n");
    TaskMsleep(1000);   //等待uart2（gpio8,gpio12）初始化完成后，再设置GPIO8 12为gpio功能

    stepper_motor_init();   // 初始化 GPIO 8 12 6 7 5 10
    dc_motor_init();
    uapi_pwm_deinit();
    uapi_pwm_init();   
    IoTPwmStop(2);  // 关闭干扰蜂鸣器 GPIO 2 PWM 2
    // 初始化正转GPIO8为PWM功能

    for (;;) {
        if (cmdMotorStatus == Dcmotor && !dc_motor_running) {
            // 开始直流电机循环控制
            dc_motor_running = true;
            cycle_count = 0;
            current_speed = 50;  // 初始速度
            direction = 1;       // 开始正转

            while (cycle_count < 2) {
                // 正转加速阶段：每100ms速度加1，直到>90
                direction = 1;
                motorConfig.motordir = direction;
                dc_motor_set_direction(&motorConfig);

                while (current_speed <= 90) {
                    motorConfig.speed = current_speed;
                    dc_motor_set_speed(&motorConfig);
                    printf("Current speed: %d (Forward)\r\n", current_speed);
                    TaskMsleep(100);  // 100ms
                    current_speed++;
                }

                // 反转减速阶段：每10ms速度减1，直到<70
                direction = 2;
                motorConfig.motordir = direction;
                dc_motor_set_direction(&motorConfig);

                while (current_speed >= 70) {
                    motorConfig.speed = current_speed;
                    dc_motor_set_speed(&motorConfig);
                    printf("Current speed: %d (Reverse)\r\n", current_speed);
                    TaskMsleep(10);  // 10ms
                    current_speed--;
                }

                cycle_count++;
                printf("Cycle %d completed\r\n", cycle_count);
            }

            // 循环结束后，停止直流电机并切换到步进电机
            motorConfig.motordir = 0;  // 停止
            dc_motor_set_direction(&motorConfig);
            cmdMotorStatus = StepperMotor;
            dc_motor_running = false;
            printf("DC motor stopped, starting stepper motor\r\n");
        }
        else if (cmdMotorStatus == StepperMotor) {
            stepper_motor_run();
        }
        else {
            TaskMsleep(50);
        }
        // 喂狗操作，确保系统在线程正常运行时不会重启
        IoTWatchDogDisable();
        IoTWatchDogEnable();
        TaskMsleep(5); // 控制循环速度
    }
}

static void MotorControlTaskEntry(void) {
    osThreadAttr_t attr;

    attr.name = "IotMainTaskEntry";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000; /* 堆栈大小为1024 */
    attr.priority = osPriorityNormal;
    // 报错
    if (osThreadNew((osThreadFunc_t)MotorControlTask, NULL, &attr) == NULL) {
        printf("[stepper_motor] Failed to create LedTask!\n");
    }
    printf("ledcontroltask_start222...\r\n");
}

/*-----------------------------------------------------------
    MQTT 回调函数，云端下发，属性上报
-----------------------------------------------------------*/

void rec_cbk1(void *result) {
    if (result != NULL) {
        printf("Callback 1: Temperature is %s\n", (char *)result);
        free(result); 
    } else {
        printf("Callback 1: No result for temperature\n");
    }
}

// 回调函数 2
void rec_cbk2(void *result) {
    if (result != NULL) {
        printf("start change result \n");
        char templine[32] = {0};
        sprintf(templine, ": %.2f", *(double *)result);
        printf("Callback 2: humidity is %s\n", templine);
        free(result); 
    } else {
        printf("Callback 2: No result for humidity\n");
    }
}

// MQTT 回调函数：根据云端下发内容修改显示模式
void cmdMotorStatusCallback(void *result) {
    if (result != NULL) {
        printf("Dcmotor mode changed to: %s\n", (char *)result);
        if (strcmp((char *)result, "Dcmotor") == 0) {
            cmdMotorStatus = Dcmotor;
        } else if (strcmp((char *)result, "StepperMotor") == 0) {
            cmdMotorStatus = StepperMotor;
        }
        free(result);
    }
}

void cmdMotorDirCallback(void *result) {
    if (result != NULL) {
        int dir = *(int *)result;  // 正确获取整型值
        printf("MotorDir changed to: %d\n", dir);
        motorConfig.motordir = dir;
        directionChanged = true;
        free(result);
    }
}


void cmdMotorSpeedCallback(void *result) {
    if (result != NULL) {
        int speed = *(int *)result;  // 正确获取整型值
        printf("MotorSpeed changed to: %d\n", speed);
        motorConfig.speed = speed;
        speedChanged = true;
        free(result);
    }
}


void mqtt_push(int i){
    CloudService* cloudService = cloud_createService("DcMOTOR");
    // double result1 = 3.5 + i; 
    //  double result2 = 6.1+i; 
    //  void *temperature= &result1;
    // void *humidity= &result2;
    // cloud_appendProperty(cloudService,"temperature",IOT_PROFILE_KEY_DATATYPE_DOUBLE,temperature);
    // cloud_appendProperty(cloudService,"humidity",IOT_PROFILE_KEY_DATATYPE_DOUBLE,humidity);
    cloud_appendProperty(cloudService,"motorStatus",IOT_PROFILE_KEY_DATATYPE_INT, (void*)& cmdMotorStatus);
    cloud_appendProperty(cloudService,"motorDir",IOT_PROFILE_KEY_DATATYPE_INT, (void*)& motorConfig.motordir);
    cloud_appendProperty(cloudService,"motorSpeed",IOT_PROFILE_KEY_DATATYPE_INT, (void*)& motorConfig.speed);


    int ret=cloud_upService(cloudService);
    if(ret!=0){
        printf("push failed %d\n",ret);
    }else{
        printf("push success\n");
    }
}

// mesPushTest 线程：定期调用 mqtt_push 上报状态
static void mesPushTest(void *arg) {
    (void)arg;
    printf("start push test!\n");
    int i = 0;
    while (1) {
        mqtt_push(i);
        i++;
        osDelay(500);
    }
}

static void mesPushTestEntry(void)
{
    osThreadAttr_t attr;
    attr.name = "mesPushTest";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000;
    attr.priority = osPriorityNormal;
    
    if (osThreadNew(mesPushTest, NULL, &attr) == NULL) {
        printf("[mqttTestEntry] Failed to create mesPushTest!\n");
    }
}

// MQTT 测试主任务线程：初始化 MQTT、订阅显示模式，并启动显示和上报线程
static void mqttTestTask(void *arg)
{
    (void)arg;
    int ret = mqttInit(CLIENTID, NTP_SERVER_USERNAME, NTP_SERVER_PASSWORD);
    if(ret != 0) {
        printf("mqtt init failed \n");
        return;
    } else {
         printf("mqtt init success \n");
    }
    // 订阅显示模式切换命令（云端下发字符串 "TUBE" 或 "EMATRIX1"）
    cloud_sub("motorStatus", IOT_PROFILE_KEY_DATATYPE_STRING, cmdMotorStatusCallback);
    cloud_sub("motorDir", IOT_PROFILE_KEY_DATATYPE_INT, cmdMotorDirCallback);
    cloud_sub("motorSpeed", IOT_PROFILE_KEY_DATATYPE_INT, cmdMotorSpeedCallback);
    MotorControlTaskEntry();
    mesPushTestEntry();
}

// 入口函数
static void mqttTestEntry(void)
{
    osThreadAttr_t attr;
    attr.name = "mqttTestTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000;
    attr.priority = osPriorityNormal;
    
    if (osThreadNew(mqttTestTask, NULL, &attr) == NULL) {
        printf("[mqttTestEntry] Failed to create mqttTestTask!\n");
    }
}

APP_FEATURE_INIT(mqttTestEntry);
