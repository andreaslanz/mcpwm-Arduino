/*
Dragrace*/
#include <esp32/rom/uart.h>
#include <esp_log.h>
#include <HardwareSerial.h>
#include "include/main.h"
#include "include/zeit.h"
#include "include/utility.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_task_wdt.h>

#include "Arduino.h"

#define WEBSERVER 0 //alt geht nicht
#define WEBSERVER2 0  //neuer Server
#define OLED_DISPLAY 0

xQueueHandle cap_queue;
static const char* TAG="main.cpp";

//Webservertask
[[noreturn]] static void Web_Task(void *arg){
    while(1){
#if WEBSERVER
        Webserver_loop();
#endif
    }
}



void setup()
{
    // Test Set Pins as Output
    //dragrace_set_Test_Pin_as_Output(BIT(DRAGRACE_PIN_TEST_L1_OUTPUT));

#if WEBSERVER2
    //Start Webserver
    dragrace_webserver();
#endif


    Serial_Start();



#if OLED_DISPLAY
    display_setup();
#endif

#if WEBSERVER
    Webserver_Setup();
#endif

    mcpwm_setup();
    printf("Start\n");

#if WEBSERVER
    xTaskCreate(Web_Task,"Web_Task",8000,NULL,2,NULL);
#endif



/// List all Task need:
//+CONFIG_FREERTOS_USE_TRACE_FACILITY=y
//+CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
//+CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID=y
//char buf[1000];
//    vTaskList(buf);
//    ESP_LOGI(TAG,"*********************************");
//    ESP_LOGI(TAG,"Task         State Prio Stack Num");
//    ESP_LOGI(TAG,"*********************************");
//    ESP_LOGI(TAG,"%s",buf);
//    Serial.println(F("*********************************"));
//    Serial.println(F("Task         State Prio Stack Num"));
//    Serial.println(F("*********************************"));
//    Serial.println(buf);


    loopTaskWDTEnabled = true;
    enableLoopWDT();

}



void loop()
{
    vTaskDelay(100);
}

