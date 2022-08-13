//
// Created by andi on 27.03.2021.
//

#ifndef MCPWM_ARDUINO_UTILITY_H
#define MCPWM_ARDUINO_UTILITY_H
#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/task.h"


uint16_t CRC16(unsigned char *ptr,unsigned int count);
void anzeige(int a,int b,uint8_t *buf);
void dragrace_set_Test_Pin_as_Output(uint64_t pin);
void dragrace_set_Test_Pin_as_Input_Pullup(uint64_t pin);
void dragrace_set_Test_Pin_as_Input_Pulldown(uint64_t pin);
void dragrace_set_Test_Pin_as_Input(uint64_t pin);
extern TaskHandle_t impulse_task_handle;

typedef struct {
    union {
        struct{
            uint32_t softstart:1;
            uint32_t L1:1;
            uint32_t L2:1;
            uint32_t L3:1;
        };
        uint32_t val;
    };
    uint32_t randomstart;

}dragrace_puls_struct_t;


typedef enum {
     dragrace_pulsmode_soft_start,
     dragrace_pulsmode_
}dragrace_pulse_mode_t;

void dragrace_impulse(void *pvParameter1, uint32_t delay_ticks);

#ifdef __cplusplus
}

#endif


#endif //MCPWM_ARDUINO_UTILITY_H
