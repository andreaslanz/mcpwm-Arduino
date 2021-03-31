//
// Created by andi on 27.03.2021.
//

#ifndef MCPWM_ARDUINO_UTILITY_H
#define MCPWM_ARDUINO_UTILITY_H
#ifdef __cplusplus
extern "C" {
#endif

void dragrace_set_Test_Pin_as_Output(uint32_t pin);
void dragrace_set_Test_Pin_as_Input(uint32_t pin);

typedef struct {
    union {
    uint32_t softstart:1;
    uint32_t L1:1;
    uint32_t L2:1;
    uint32_t L3:1;

    };
    uint32_t val;

}dragrace_puls_struct_t;


typedef enum {
     dragrace_pulsmode_soft_start,
     dragrace_pulsmode_
}dragrace_pulse_mode_t;

void dragrace_impulse();

#ifdef __cplusplus
}
#endif


#endif //MCPWM_ARDUINO_UTILITY_H
