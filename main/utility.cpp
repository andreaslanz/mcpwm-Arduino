//
// Created by andi on 27.03.2021.
//

#include "freertos/FreeRTOS.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include "include/utility.h"
#include "include/zeit.h"

const char* TAG = "utility";
dragrace_puls_struct_t PM;
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1}; //MCPWM Register
TaskHandle_t impulse_task_handle;

uint16_t CRC16(unsigned char *ptr,unsigned int count)
{
    uint16_t crc =0;
    unsigned char i;
    while(count>0)
    {
        count--;
        crc = ( crc^(((uint16_t)*ptr)<<8));
        for(i=0;i<8;i++)
        {
            if(crc&0x8000) crc= ((crc<<1)^0x1021);
            else crc <<= 1;
        }
        ptr++;
    }
    return crc;
}

void anzeige(int a,int b,uint8_t *buf){
    buf[10]=a/1000;     ///1. Zahl
    buf[11]=a%1000/100;
    buf[12]=a%100/10;
    buf[13]=a%10;
    buf[6]=b/1000;      ///2.Zahl
    buf[7]=b%1000/100;
    buf[8]=b%100/10;
    buf[9]=b%10;
    uint16_t crc= CRC16(buf,15);
    uint8_t crc1 = (crc>>8);
    uint8_t crc2 = (crc&0xff);
    buf[15]=crc1;       ///Prüfsumme
    buf[16]=crc2;
}


void dragrace_set_Test_Pin_as_Output(uint64_t  pin){
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode= GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = pin;
    gpio_config(&gp);
}
void dragrace_set_Test_Pin_as_Output_OpenDrain(uint32_t pin){
    gpio_config_t gp = {};
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode= GPIO_MODE_OUTPUT_OD;
    gp.pin_bit_mask = pin;
    gpio_config(&gp);
}
void dragrace_set_Test_Pin_as_Input_Pullup(uint32_t pin){
    gpio_config_t gp = {};
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_INPUT;
    gp.pin_bit_mask = pin;
    gp.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gp);
}
void dragrace_set_Test_Pin_as_Input(uint64_t pin){
    gpio_config_t gp = {};
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_INPUT;
    //gp.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    gp.pin_bit_mask = pin;
    //gp.pull_up_en = GPIO_PULLUP_ENABLE;
    gp.pull_up_en = GPIO_PULLUP_DISABLE;
    gp.pull_down_en = GPIO_PULLDOWN_DISABLE;
    esp_err_t e= gpio_config(&gp);
    ESP_LOGI(TAG,"err:%d",e);
}
void dragrace_set_Test_Pin_as_Input_Pulldown(uint64_t  pin){
    gpio_config_t gp = {};
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_INPUT;
    gp.pin_bit_mask = pin;
    gp.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&gp);
}

static void  impuls_task(void *pm) {

    /// Start Sequenz
    uint32_t pins;

    pins=0;
    dragrace.Status_new.Gestartet_NEW=true;
    dragrace.Status_new.Ready_NEW=false;

    dragrace.Status_new.Orange1=1;
    dragrace_show("Start Orange1");
    vTaskDelay(100);
    dragrace.Status_new.Orange2=1;
    dragrace.Status_new.Orange1=0;
    dragrace_show("Orange2");
    vTaskDelay(PM.randomstart);
    dragrace.Status_new.Orange2=0;
    dragrace.Status_new.Laeuft_NEW=1;



    ///disable interrupt für start
    MCPWM[0]->int_ena.cap2_int_ena=0;
    MCPWM[1]->int_ena.cap2_int_ena=0;
    //Startzeit erfassen durch Software Caputure on channel 2 (3.Lichtschranken)
    MCPWM[MCPWM_UNIT_0]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;
    MCPWM[MCPWM_UNIT_1]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;
    ///enable interrupt
    MCPWM[0]->int_ena.cap2_int_ena=1;
    MCPWM[1]->int_ena.cap2_int_ena=1;


    if( PM.softstart){
        dragrace.Status_old.Gestartet=1;
        ESP_LOGD(TAG,"pm soft start");
    }

    if(PM.L1){
        pins |= BIT(DRAGRACE_PIN_TEST_L1_OUTPUT);
    }

    // Log Level
    esp_log_level_set("gpio", ESP_LOG_WARN);

    dragrace_set_Test_Pin_as_Output(pins);

    *(uint32_t *)GPIO_OUT_W1TS_REG=pins; //Set Pin High
    vTaskDelay(1);             //delay of 10ms
    *(uint32_t *)GPIO_OUT_W1TC_REG=pins; //Set Pin Low
    vTaskDelay(1);             //delay of 10ms

    dragrace_set_Test_Pin_as_Input_Pullup(pins);

    vTaskDelete(nullptr);
}


void dragrace_impulse(void *pvParameter1, uint32_t ulParameter2) {


    PM.val=0;
    PM.softstart=1;
    PM.L1=1;
    PM.randomstart=dragrace.randomstart;

    //Start Task kreieren
    impulse_task_handle=NULL;
    xTaskCreate(impuls_task, "pulse", 4096, &PM, 5, &impulse_task_handle);

}