#ifndef mcpwm
#define mcpwm

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define MCPWM_EN_CARRIER 0   //Make this 1 to test carrier submodule of mcpwm, set high frequency carrier parameters
#define MCPWM_EN_DEADTIME 0  //Make this 1 to test deadtime submodule of mcpwm, set deadtime value and deadtime mode
#define MCPWM_EN_FAULT 0     //Make this 1 to test fault submodule of mcpwm, set action on MCPWM signal on fault occurence like overcurrent, overvoltage, etc
#define MCPWM_EN_SYNC 0      //Make this 1 to test sync submodule of mcpwm, sync timer signals
#define MCPWM_EN_CAPTURE 1   //Make this 1 to test capture submodule of mcpwm, measure time between rising/falling edge of captured signal
#define MCPWM_GPIO_INIT 1    //select which function to use to initialize gpio signals
#define CAP_SIG_NUM 3   //Three capture signals

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit


#define GPIO_PWM0A_OUT 13   //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 37   //Set GPIO 18 as PWM0B
#define GPIO_PWM1A_OUT 17   //Set GPIO 17 as PWM1A
#define GPIO_PWM1B_OUT 38   //Set GPIO 16 as PWM1B
#define GPIO_PWM2A_OUT 39   //Set GPIO 15 as PWM2A
#define GPIO_PWM2B_OUT 22   //Set GPIO 14 as PWM2B
#define GPIO_CAP0_IN   23   //Set GPIO 23 as  CAP0
#define GPIO_CAP1_IN   25   //Set GPIO 25 as  CAP1
#define GPIO_CAP2_IN   26   //Set GPIO 26 as  CAP2
#define GPIO_SYNC0_IN   2   //Set GPIO 02 as SYNC0
#define GPIO_SYNC1_IN   4   //Set GPIO 04 as SYNC1
#define GPIO_SYNC2_IN   5   //Set GPIO 05 as SYNC2
#define GPIO_FAULT0_IN 32   //Set GPIO 32 as FAULT0
#define GPIO_FAULT1_IN 34   //Set GPIO 34 as FAULT1
#define GPIO_FAULT2_IN 34   //Set GPIO 34 as FAULT2


extern uint32_t Zahl;



//DEBUGG
extern volatile uint32_t  Drag_mcpwm_intr_status;
extern volatile uint32_t  Drag_mcpwm_intr_clr;

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

#ifdef __cplusplus
extern "C" {
#endif


static void mcpwm_example_gpio_initialize();
static void gpio_test_signal(void *arg);

static void disp_captured_signal(void *arg);
static void mcpwm_example_config(void *arg);
void mcpwm_setup();
void start();
void L1();
void L3();


#ifdef __cplusplus
}
#endif


#endif