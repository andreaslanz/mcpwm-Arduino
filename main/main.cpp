/*
Dragrace*/
#include <esp32/rom/uart.h>
#include <esp_log.h>
#include <HardwareSerial.h>
#include "include/main.h"
#include "include/zeit.h"
#include "include/display.h"
#include "include/web.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_task_wdt.h>

#include "Arduino.h"

#define WEBSERVER 0
#define OLED_DISPLAY 0

xQueueHandle cap_queue;

[[noreturn]] static void Web_Task(void *arg){

    while(1){
#if WEBSERVER
        Webserver_loop();
#endif

    }
}



void setup()
{

    Serial_Start();

#if OLED_DISPLAY
    display_setup();
#endif

#if WEBSERVER
    Webserver_Setup();
#endif

    mcpwm_setup();

#if WEBSERVER
    xTaskCreate(Web_Task,"Web_Task",8000,NULL,2,NULL);
#endif

//char buf[1000];
//    vTaskList(buf);
//    Serial.println(F("*********************************"));
//    Serial.println(F("Task         State Prio Stack Num"));
//    Serial.println(F("*********************************"));
//    Serial.println(buf);


    loopTaskWDTEnabled = true;
    enableLoopWDT();

}



void loop()
{
    vTaskDelay(10);
}

