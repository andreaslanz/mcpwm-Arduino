/*
Dragrace*/
#include <esp32/rom/uart.h>
#include <HardwareSerial.h>
#include "include/main.h"
#include "include/zeit.h"
#include "include/web.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_task_wdt.h>

#include "Arduino.h"

#define WEBSERVER 0

xQueueHandle cap_queue;

static void t1(void *arg){
    int a=0;

uint32_t Zahl;


    while (1){
        Serial.println(a++);
        delay(1);
    }



}



void setup()
{
    //initArduino();
//    Serial.begin(115200);
//    Serial.println("Hello");
//    delay(1);
    Serial_Start();
#if WEBSERVER
    Webserver_Setup();
#endif
    mcpwm_setup();
    //xTaskCreate(t1,"t1",8000,NULL,3,NULL);

//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
//    gpio_pulldown_en(GPIO_NUM_23);    //Enable pull down on CAP0   signal
//
//    mcpwm_config_t pwm_config;
//
//    pwm_config.frequency = 10;    //frequency = 1000Hz
//    pwm_config.cmpr_a = 60.0;       //duty cycle of PWMxA = 60.0%
//    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
//    pwm_config.counter_mode = MCPWM_UP_COUNTER;
//    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
//
//    mcpwm_capture_enable(MCPWM_UNIT_0,MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
    loopTaskWDTEnabled = true;
    enableLoopWDT();

}



void loop()
{
    //Serial.println( mcpwm_capture_signal_get_value(MCPWM_UNIT_0,MCPWM_SELECT_CAP0));
#if WEBSERVER
    Webserver_loop();
#endif
//    STATUS s = uart_rx_one_char(&c);
//    if (s == OK) {
//        printf("%c\n", c);
//    }

//    printf("1 %lu\n",millis());
//    delay(1);
//    printf("2 %lu\n",millis());
//    delay(1);

//    uint8_t c;
//    if(Serial.available()) {
//        c = Serial.read();
//        if (c == 's'){
//            printf("Start");
//
//            start();
//        }
//        printf("%c\n", c);
//    }

//    vTaskDelay(1);
//    printf("fuck");
//yield();
//feedLoopWDT();
//    esp_task_wdt_reset();
    vTaskDelay(1);
}

