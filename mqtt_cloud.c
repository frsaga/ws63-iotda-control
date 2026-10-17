/*
 * Copyright (c) 2024 HiSilicon (Shanghai) Technologies CO., LIMITED.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "osal_addr.h"

#include "MQTTClient.h"     // MQTTClient-C库接口文件
#include "mqtt_ohos.h"      // OHOS适配接口文件
#include "MQTTConnect.h"
#include "cjson_cloud.h"
#include "mqtt_cloud.h"
#include "wifi_connecter.h"
 
 
 
 //要连接热点的名称，根据实际情况修改
 #define SSID "博赛网络"

 //要连接热点的密码，根据实际情况修改
 #define PSK "Bossay@0419"


 #define MQTT_SERVER_IP "127.0.0.1"
 
 // MQTT服务器端口,请根据实际情况修改
 #define MQTT_SERVER_PORT "1883"
 
 // MQTT客户端
 static MQTTClient client = {0};
 
 // MQTT网络连接
 static Network network = {0};
 
 static MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
 // 发送和接收的数据缓冲区（增大以提高容错）
 static unsigned char sendbuf[1024], readbuf[1024];
 
 //MQTT消息
 MQTTMessage message;
 
 const char *g_host = MQTT_SERVER_IP;                               
 unsigned short g_port = NULL;                      
 const char *g_clientId = NULL; 
 const char *g_username = NULL;                                      
 const char *g_password = NULL;   
 messageHandler g_messageHandler;
 
 // 保持订阅主题指针在整个会话期间有效，避免被MQTTClient保存后悬空
 static char *g_subTopic = NULL;
 
 typedef struct {
     const char *fieldName;       
     int dataType;                
     CallbackFunction callback;   
 } RegisterEntry;
 
 static RegisterEntry g_registerTable[10]; 
 static int g_registerCount = 0;         
 
 char *get_last_part_after_equal(const char *str) {
     if (str == NULL || strlen(str) == 0) {
         return NULL; 
     }
     const char *last_equal = strrchr(str, '=');
     if (last_equal == NULL) {
         return NULL; 
     }
     size_t len_after_equal = strlen(last_equal) - 1;
     if (len_after_equal == 0) {
         return NULL; 
     }
     char *result = malloc((len_after_equal + 1) * sizeof(char));
     if (result == NULL) {
         return NULL; 
     }
     strncpy(result, last_equal + 1, len_after_equal); 
     result[len_after_equal] = '\0'; 
     return result;
 }
 
 
 int mqtt_publish_res(const char *request_id) {
     char *topic = combine_strings(4, "$oc/devices/", g_username, "/sys/commands/response/request_id=", request_id);
     message.qos = QOS1; // 响应使用 QoS1
     char *msg = makeJsonRes();
     message.retained = 0;
     message.payload = msg;

     message.payloadlen = strlen(msg);
     int ret = MQTTPublish(&client, topic, &message);
     if (ret != 0) {
         printf("MQTTPublish res failed: %d\r\n", ret);
     } else {
         printf("MQTT Publish res OK\r\n");
     }
     if (msg) {
         free(msg);
         msg = NULL;
     }
     if (topic) {
         free(topic);
         topic = NULL;
     }
     return ret == 0 ? 0 : -1;
 }
 
 
 void cloud_sub(const char *fieldName, int dataType, CallbackFunction callback) {
     if (callback == NULL || fieldName == NULL) {
         printf("Error: Invalid parameters\n");
         return;
     }
     if (g_registerCount >= sizeof(g_registerTable) / sizeof(g_registerTable[0])) {
         printf("Error: Register table is full\n");
         return;
     }
     g_registerTable[g_registerCount].fieldName = strdup(fieldName);
     g_registerTable[g_registerCount].dataType = dataType;
     g_registerTable[g_registerCount].callback = callback;
     g_registerCount++;
 }
 
 
 
// 接收订阅消息的回调函数
static void OnMessageArrived(MessageData *data)
{
    // 1) 复制并终止 topic，避免对非NUL字符串使用C字符串API
    char *topic = NULL;
    if (data->topicName->lenstring.len > 0 && data->topicName->lenstring.data != NULL) {
        int tlen = data->topicName->lenstring.len;
        topic = (char *)malloc((size_t)tlen + 1);
        if (topic != NULL) {
            memcpy(topic, data->topicName->lenstring.data, (size_t)tlen);
            topic[tlen] = '\0';
            printf("[OnMessageArrived] topic: %s\r\n", topic);
            char *request_id = get_last_part_after_equal(topic);
            if (request_id) {
                (void)mqtt_publish_res(request_id);
                free(request_id);
            }
        }
    }

    // 2) 复制并终止 payload，确保 cJSON 能正确解析
    char *payload = NULL;
    if (data->message->payloadlen > 0 && data->message->payload != NULL) {
        int plen = data->message->payloadlen;
        payload = (char *)malloc((size_t)plen + 1);
        if (payload != NULL) {
            memcpy(payload, (const char *)data->message->payload, (size_t)plen);
            payload[plen] = '\0';
            printf("[OnMessageArrived] payload: %s\r\n", payload);

            for (int i = 0; i < g_registerCount; i++) {
                const char *fieldName = g_registerTable[i].fieldName;
                int dataType = g_registerTable[i].dataType;
                CallbackFunction callback = g_registerTable[i].callback;
                void *result = parseJsonByFieldName(payload, (char *)fieldName, dataType);
                if (result != NULL) {
                    printf("[OnMessageArrived] matched field: %s\r\n", fieldName);
                    callback(result);
                    free(result);
                }
            }
        }
    }

    if (payload) {
        free(payload);
    }
    if (topic) {
        free(topic);
    }
}
 
 
int mqtt_reconnect(){
     // 确保安全断开旧连接
     if (client.isconnected) {
         (void)MQTTDisconnect(&client);
     }
     NetworkDisconnect(&network);

     int ret = NetworkConnect(&network, (char *)g_host, g_port);
     if (ret != 0) {
         printf("TCP Connect failed!,ret = %d\r\n", ret);
         return ret;
     }
     if ((ret = MQTTConnect(&client, &connectData)) != 0) {
         printf("Connect MQTT Broker failed!,ret = %d\r\n", ret);
         return ret;
     } else {
         printf("MQTT Connected!\r\n");
     }
    if (g_subTopic == NULL) {
        g_subTopic = combine_strings(3, "$oc/devices/", g_username, "/sys/commands/#");
    }
    if ((ret = MQTTSubscribe(&client, g_subTopic, QOS1, OnMessageArrived)) != 0) {
        printf("MQTTSubscribe failed: %d\r\n", ret);
    } else {
        printf("MQTT Subscribe OK\r\n");
    }
    return ret == 0 ? 0 : -1;
}
 
 
 
 static void MqttTask(void *arg)
 {
     MQTTClient *c = (MQTTClient *)arg;
     int connected = 0;
     while (c) {
         // 仅在短时间持锁读取连接状态，避免双重解锁
         mqttMutexLock(&c->mutex);
         connected = c->isconnected;
         mqttMutexUnlock(&c->mutex);

         if (connected) {
             MQTTYield(c, 2000);
         } else {
             (void)mqtt_reconnect();
         }
         osDelay(100);
     }
 }
 
 CloudService* cloud_createService(char *serviceId){
      CloudService* service = malloc(sizeof(CloudService));
     if(service == NULL){
         printf("malloc Cloud_Proprety failed!");
         return;
     }
     service->serviceID = serviceId;
     service->propertyLst = NULL;
     service->nxt = NULL;
     service->eventTime = NULL;
     return service;
 }
 
 
 int cloud_appendProperty(CloudService* cloudService,const char *name,IotProfileDataType type,void* value){
      CloudProperty* tempPrp;
     CloudProperty* prpty = malloc(sizeof(CloudProperty));
     if(prpty == NULL){
         printf("malloc Cloud_Proprety failed!");
         return -1;
     }
     prpty->nxt = NULL;
     prpty->type = type;
     prpty->value = value;
     prpty->key = name;
     tempPrp = cloudService->propertyLst;
     if(tempPrp == NULL){
         cloudService->propertyLst = prpty;
     }else{
         while(tempPrp->nxt != NULL){
             tempPrp = tempPrp->nxt;
         }
         tempPrp->nxt = prpty;
     }
     return 0;
 }
 
 
 
int cloud_upService(CloudService *cloudService) {
    char *stopic = combine_strings(3, "$oc/devices/", g_username, "/sys/properties/report");
    char *payload = toServiceJson(cloudService);
    // 上报建议使用 QoS0（或 QoS1）
    message.qos = QOS0;
     /*
     message.retained = 1,MQTT服务器收到这样的MQTTMessage数据包以后，将保存这个消息，
      当有一个新的订阅者订阅相应主题的时候，MQTT服务器会马上将这个消息发送给订阅.
      message.retained = 0，订阅者连接到MQTT服务器时，不会接收该主题的最新消息.
      */
     message.retained = 0;
 
     // 设置消息的内容
     message.payload = payload;
 
     // 设置消息的长度
     message.payloadlen = strlen(payload);
    int ret=MQTTPublish(&client, stopic, &message);
     //  printf("mqtt_publish(), the payload is %s, the topic is %s\r\n", payload, stopic);
     // 发布消息
     if (ret != 0)
     {
         // 发布消息失败
        printf("MQTTPublish failed: %d\r\n", ret);
        if (payload) { free(payload); payload = NULL; }
        if (stopic) { free(stopic); stopic = NULL; }
        return -1;
    }else{
        // 发布成功
        printf("MQTT Publish OK\r\n"); 
       
    }
    if (payload) { free(payload); payload = NULL; }
    if (stopic) { free(stopic); stopic = NULL; }
    return 0;
}
 
 
 
static void connectDataInit(){
     
     
     // 设置用户名和密码
     if (g_username != NULL && g_password != NULL)
     {
         connectData.username.cstring = (char *)g_username;
         connectData.password.cstring = (char *)g_password;
     }
 
     /* 设置MQTT版本*/
     // 3 = 3.1
     // 4 = 3.1.1
     connectData.MQTTVersion = 3;
 
     // 设置MQTT客户端ID
     connectData.clientID.cstring = (char *)g_clientId;
     connectData.keepAliveInterval=30;//秒
}
 
 
int mqtt_sub_init(){
    if (g_subTopic == NULL) {
        g_subTopic = combine_strings(3, "$oc/devices/", g_username, "/sys/commands/#");
    }
    int ret=0;
    if ((ret = MQTTSubscribe(&client, g_subTopic, QOS1, OnMessageArrived)) != 0) {
        printf("MQTTSubscribe failed: %d\r\n", ret);
        return ret;
    } else {
        printf("MQTT Subscribe OK\r\n");
    }
    return ret;
}
 
 
 int mqttInit(const char *clientId,const char *username,const char *password){
     g_clientId = clientId; 
     g_username = username;                                      
     g_password = password;  
     // 连接到热点
     if (ConnectToHotspot(SSID, PSK) != 0)
     {
         // 连接到热点失败
         printf("Connect to AP failed\r\n"); 
         return -1;
     }
     // 等待TCP连接完成
     osDelay(100);
     //初始化MQTT相关的参数和回调
     NetworkInit(&network);
     // 初始化MQTT客户端
     // 增大命令超时，提升网络波动下的稳定性
     MQTTClientInit(&client, &network, 5000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));
   
     g_port = atoi(MQTT_SERVER_PORT);                      
     connectDataInit();
     // 用于接收接口返回值
     int ret = NetworkConnect(&network, (char *)g_host, g_port);
 
     if(ret != 0){
         printf("TCP Connect failed!,ret = %d\r\n",ret);
         return ret;
     }
 
     // 发送MQTT连接包
     if ((ret = MQTTConnect(&client, &connectData)) != 0)
     {
         // 连接失败
         printf("Connect MQTT Broker failed!,ret = %d\r\n",ret);
         return ret;
     }else{
        // 成功连接到MQTT服务器
         printf("MQTT Connected!\r\n");
     }
    mqtt_sub_init();
  
     // 创建MQTT任务线程,接收订阅主题的消息
      ret = ThreadStart(&client.thread, MqttTask, &client);
      return ret;
 }

void cloud_freeService(CloudService* service)
{
    if (service == NULL) {
        return;
    }
    CloudProperty *curr = service->propertyLst;
    while (curr != NULL) {
        CloudProperty *next = (CloudProperty *)curr->nxt;
        free(curr);
        curr = next;
    }
    free(service);
}
