//
// Created by andi on 27.03.2021.
//

#include "FreeRTOS.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include "include/utility.h"
#include "include/zeit.h"

const char* TAG = "utility";
dragrace_puls_struct_t PM;

void dragrace_set_Test_Pin_as_Output(uint32_t pin){
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode= GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = pin;
    gpio_config(&gp);
}
void dragrace_set_Test_Pin_as_Input(uint32_t pin){
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_INPUT;
    gp.pin_bit_mask = pin;
    gp.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gp);
}

static void  impuls_task(void *pm) {

    uint32_t pins;

    pins=0;

    if( PM.softstart){
        dragrace.Status.Gestartet=1;
        ESP_LOGD(TAG,"pm soft start");
    }

    if(PM.L1){
        pins |= BIT(DRAGRACE_PIN_TEST_L1_OUTPUT);
    }

    dragrace_set_Test_Pin_as_Output(pins);

    *(uint32_t *)GPIO_OUT_W1TS_REG=pins; //Set Pin High
    vTaskDelay(1);             //delay of 10ms
    *(uint32_t *)GPIO_OUT_W1TC_REG=pins; //Set Pin Low
    vTaskDelay(1);             //delay of 10ms

    dragrace_set_Test_Pin_as_Input(pins);

    vTaskDelete(NULL);
}


void dragrace_impulse(void *pvParameter1, uint32_t ulParameter2) {


    PM.val=0;
    PM.softstart=1;
    PM.L1=1;
    if(ulParameter2!=0)
        vTaskDelay(ulParameter2);
    xTaskCreate(impuls_task, "pulse", 4096, &PM, 5, NULL);

}