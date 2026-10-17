 /*
 Copyright (C) 2024 HiHope Open Source Organization .
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <stdio.h>    
#include <stdlib.h>   
#include <string.h>    
#include "ohos_init.h" 
#include "cmsis_os2.h" 

#include "data_cloud.h"  
#include "mqtt_cloud.h"
#include "wifi_connecter.h"
#include "iot_gpio.h"
#include "iot_gpio_ex.h"

#define CLIENTID "695b55c77f2e6c302f4747d2_LED01_0_0_2026010614"
#define NTP_SERVER_USERNAME "695b55c77f2e6c302f4747d2_LED01"
#define NTP_SERVER_PASSWORD "3ec461473ce0e7dc1881f51cbc4b14a8144c82bf18d2616f91d2d26b194203df"

#define MAX_STRING_VALUE_LEN 30
#define LED1 6
#define LED2 7
#define LED3 5
#define LED4 10
#define LED5 9

typedef enum
{
	OFF = 0,
	ON
} PaomadengStatus;

static int cmdPaomadengStatus = OFF;

// LED 引脚数组，便于循环操作
static unsigned int g_ledPins[] = {LED1, LED2, LED3, LED4, LED5};
// 对应的功能复用，注意有些引脚没有默认 GPIO 功能时需要先行确认
static unsigned int g_ledFuncs[] = {
    IOT_IO_FUNC_GPIO_6_GPIO, // GPIO6
    IOT_IO_FUNC_GPIO_7_GPIO, // GPIO7
    4,                       // GPIO5 的 GPIO 功能是 4
    IOT_IO_FUNC_GPIO_10_GPIO,// GPIO10
    IOT_IO_FUNC_GPIO_9_GPIO  // GPIO9
};


/**
 * @brief 初始化所有LED引脚并默认熄灭
 */
static void InitializeLEDs(void)
{
    // 等待 UART2（GPIO7、GPIO8）初始化完成后再设置 GPIO7 为 GPIO 功能，视实际情况而定
    osDelay(1000);

    for (int i = 0; i < sizeof(g_ledPins) / sizeof(g_ledPins[0]); i++) {
        IoTGpioInit(g_ledPins[i]);
        IoSetFunc(g_ledPins[i], g_ledFuncs[i]);
        IoTGpioSetDir(g_ledPins[i], IOT_GPIO_DIR_OUT);
        // 默认熄灭
        IoTGpioSetOutputVal(g_ledPins[i], 0);
    }
    printf("All LEDs initialized to OFF\n");
}

// 回调函数 1
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

void cmdPaomadengStatusCallback(void *result) {
    if (result != NULL) {
        printf("Callback 3: paomadeng is %s\n", (char *)result);

        // 比较字符串
        cmdPaomadengStatus = (strcmp((char *)result, "ON") == 0) ? ON : OFF;
        printf("cmdPaomadengStatus -> %d\n", cmdPaomadengStatus);       
        free(result); 
    } else {
        printf("Callback 3: No result for PaomadengStatus\n");
    }
}

void mqtt_push(int i){
    CloudService* cloudService = cloud_createService("STAR_LED");
    // double result1 = 3.5 + i; 
    //  double result2 = 6.1+i; 
    //  void *temperature= &result1;
    // void *humidity= &result2;
    // cloud_appendProperty(cloudService,"temperature",IOT_PROFILE_KEY_DATATYPE_DOUBLE,temperature);
    // cloud_appendProperty(cloudService,"humidity",IOT_PROFILE_KEY_DATATYPE_DOUBLE,humidity);
    cloud_appendProperty(cloudService,"PaomadengStatus",IOT_PROFILE_KEY_DATATYPE_INT, (void*)&cmdPaomadengStatus);

    int ret=cloud_upService(cloudService);
    if(ret!=0){
        printf("push failed %d\n",ret);
    }else{
        printf("push success\n");
    }
}

/**
 * @brief 关闭所有LED
 */
static void TurnOffAllLEDs(void)
{
    for (int i = 0; i < sizeof(g_ledPins) / sizeof(g_ledPins[0]); i++) {
        IoTGpioSetOutputVal(g_ledPins[i], 0);
    }
}

/**
 * @brief 点亮所有LED
 */
static void TurnOnAllLEDs(void)
{
    for (int i = 0; i < sizeof(g_ledPins) / sizeof(g_ledPins[0]); i++) {
        IoTGpioSetOutputVal(g_ledPins[i], 1);
    }
}

/**
 * @brief 执行流水灯效果序列
 * 1. 依次顺时针点亮，直到全部点亮
 * 2. 延时2秒
 * 3. 逆时针依次熄灭，直到全部熄灭
 * 4. 所有灯全部开启，延时0.5秒，所有灯全部熄灭，延时0.5秒，循环5次
 * 5. 最后将所有灯点亮
 */
static void ExecuteLEDSequence(void)
{
    int ledCount = sizeof(g_ledPins) / sizeof(g_ledPins[0]);
    
    // 步骤1: 依次顺时针点亮，直到全部点亮
    printf("Step 1: Lighting LEDs clockwise\n");
    for (int i = 0; i < ledCount; i++) {
        if (cmdPaomadengStatus != ON) return; // 检查是否被关闭
        IoTGpioSetOutputVal(g_ledPins[i], 1);
        printf("LED%d ON\n", i + 1);
        osDelay(300); // 每个LED间隔300ms
    }
    
    // 步骤2: 延时2秒（全部点亮状态）
    if (cmdPaomadengStatus != ON) {
        TurnOffAllLEDs();
        return;
    }
    printf("Step 2: All LEDs ON, delay 2 seconds\n");
    osDelay(200); // 延时2秒
    
    // 步骤3: 逆时针依次熄灭，直到全部熄灭
    if (cmdPaomadengStatus != ON) {
        TurnOffAllLEDs();
        return;
    }
    printf("Step 3: Turning off LEDs counter-clockwise\n");
    for (int i = ledCount - 1; i >= 0; i--) {
        if (cmdPaomadengStatus != ON) {
            TurnOffAllLEDs();
            return;
        }
        IoTGpioSetOutputVal(g_ledPins[i], 0);
        printf("LED%d OFF\n", i + 1);
        osDelay(300); // 每个LED间隔300ms
    }
    
    // 步骤4: 所有灯全部开启，延时0.5秒，所有灯全部熄灭，延时0.5秒，循环5次
    if (cmdPaomadengStatus != ON) return;
    printf("Step 4: Flash all LEDs 5 times\n");
    for (int cycle = 0; cycle < 5; cycle++) {
        if (cmdPaomadengStatus != ON) {
            TurnOffAllLEDs();
            return;
        }
        // 全部开启
        TurnOnAllLEDs();
        printf("Cycle %d: All LEDs ON\n", cycle + 1);
        osDelay(50); // 延时0.5秒
        
        if (cmdPaomadengStatus != ON) {
            TurnOffAllLEDs();
            return;
        }
        // 全部熄灭
        TurnOffAllLEDs();
        printf("Cycle %d: All LEDs OFF\n", cycle + 1);
        osDelay(50); // 延时0.5秒
    }
    
    // 步骤5: 最后将所有灯点亮
    if (cmdPaomadengStatus != ON) {
        TurnOffAllLEDs();
        return;
    }
    printf("Step 5: Final state - All LEDs ON\n");
    TurnOnAllLEDs();
}

void PaomadengThread(void *arg)
{   
    InitializeLEDs();
    printf("PaomadengThread started, waiting for cloud control command...\n");
    
    while (1) {
        if (cmdPaomadengStatus == ON) {
            // 如果跑马灯处于 ON 状态，执行完整的流水灯效果序列
            printf("Received ON command, starting LED sequence...\n");
            ExecuteLEDSequence();
            
            // 序列执行完成后，如果仍然是ON状态，保持所有灯点亮
            if (cmdPaomadengStatus == ON) {
                printf("LED sequence completed, keeping all LEDs ON\n");
                // 等待状态改变，或者可以选择重新执行序列
                while (cmdPaomadengStatus == ON) {
                    osDelay(100); // 持续检查状态
                }
            }
        } else {
            // OFF 状态：确保所有LED都灭
            TurnOffAllLEDs();
            osDelay(500); // 降低CPU占用
        }
    }
}

static void PaomadengEntry(void)
{
    // 定义线程属性
    osThreadAttr_t attr;
    attr.name = "PaomadengThread";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000;
    attr.priority = osPriorityNormal;

    // 创建线程
    if (osThreadNew(PaomadengThread, NULL, &attr) == NULL)
    {
        printf("[mqttDemoEntry] Falied to create PaomadengThread!\n");
    }
}

static void mesPushTest(void *arg){
       (void)arg;
       printf("start push test!\n");
       int i=0;
       while(1){
            mqtt_push(i);
            i++;
            osDelay(500);//10秒
       }
}

// 入口函数
static void mesPushTestEntry(void)
{
    // 定义线程属性
    osThreadAttr_t attr;
    attr.name = "mesPushTest";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000;
    attr.priority = osPriorityNormal;

    // 创建线程
    if (osThreadNew(mesPushTest, NULL, &attr) == NULL)
    {
        printf("[mqttDemoEntry] Falied to create mesPushTest!\n");
    }
}



static void mqttTestTask(void *arg)
{
    (void)arg;
    int ret=mqttInit(CLIENTID,NTP_SERVER_USERNAME,NTP_SERVER_PASSWORD);
    if(ret!=0){
        printf("mqtt init failed \n");
        return;
    }else{
         printf("mqtt init success \n");
    }
    // cloud_sub("temperature", IOT_PROFILE_KEY_DATATYPE_STRING, rec_cbk1);
    // cloud_sub("humidity", IOT_PROFILE_KEY_DATATYPE_DOUBLE, rec_cbk2);
    cloud_sub("PaomadengStatus", IOT_PROFILE_KEY_DATATYPE_STRING, cmdPaomadengStatusCallback);
    PaomadengEntry();
    mesPushTestEntry();
}


// 入口函数
static void mqttTestEntry(void)
{
    // 定义线程属性
    osThreadAttr_t attr;
    attr.name = "mqttTestTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 0x1000;
    attr.priority = osPriorityNormal;

    // 创建线程
    if (osThreadNew(mqttTestTask, NULL, &attr) == NULL)
    {
        printf("[mqttTestEntry] Falied to create mqttDemoTask!\n");
    }
}

// 运行入口函数
APP_FEATURE_INIT(mqttTestEntry);