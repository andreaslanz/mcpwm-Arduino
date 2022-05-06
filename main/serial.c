#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "include/zeit.h"
#include "include/utility.h"

static const char *TAG = "uart_events";

#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
static QueueHandle_t uart0_queue;

void start_task(){
    uint32_t r=esp_random()>>24;
    //ESP_LOGI(TAG,"Rennen wird in %d.%d Sekunden gestartet",r/100,r%100);
    printf("Rennen wird in %d.%d Sekunden gestartet\n",r/100,r%100);
    vTaskDelay(r);
    dragrace_impulse(NULL, 0);
    vTaskDelete(NULL);
}

_Noreturn static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    char c;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    while(uart_read_bytes(EX_UART_NUM, (uint8_t *) &c, 1, portMAX_DELAY)){
                        if(c=='n'){
                            ESP_LOGI(TAG, "Neu");
                            action(neu);
                        }
                        if(c=='s'){
                            ESP_LOGI(TAG, "Start");
                            action(drag_start);
                        }
                        if(c=='1'){
                            ESP_LOGI(TAG, "L1");
                            L1();
                        }
                        if(c=='2'){
                            ESP_LOGI(TAG, "L2");
                            L2();
                        }
                        if(c=='3'){
                            ESP_LOGI(TAG, "L3");
                            L3();
                        }
                        if(c=='4'){
                            ESP_LOGI(TAG, "R1");
                            R1();
                        }
                        if(c=='5'){
                            ESP_LOGI(TAG, "R2");
                            R2();
                        }
                        if(c=='6'){
                            ESP_LOGI(TAG, "R3");
                            R3();
                        }
                        if(c=='p'){
                            ESP_LOGI(TAG, "Pulse");
//                            xTaskCreate(start_task,"Start-Task",2048,NULL,5,NULL);
                            dragrace_impulse(NULL,500);
                        }
                        // Write
//                        uart_write_bytes(EX_UART_NUM, (const char*) &c, 1);
                    }
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

void Serial_Start()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);


    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}
