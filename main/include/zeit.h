#ifndef mcpwm
#define mcpwm
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "pin_def.h"

#define MCPWM_EN_CAPTURE 1   //Make this 1 to test capture submodule of mcpwm, measure time between rising/falling edge of captured signal
#define MCPWM_GPIO_INIT 1    //select which function to use to initialize gpio signals
#define CAP_SIG_NUM 3   //Three capture signals

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit


#define GPIO_CAP0_IN   DRAGRACE_PIN_LICHTSCHRANKE_L1_INPUT   //Set GPIO 23 as  CAP0
#define GPIO_CAP1_IN   DRAGRACE_PIN_LICHTSCHRANKE_L2_INPUT   //Set GPIO 19 as  CAP1
#define GPIO_CAP2_IN   DRAGRACE_PIN_LICHTSCHRANKE_L3_INPUT   //Set GPIO 22 as  CAP2

//SW interrupt
typedef union  {
    struct {
        uint16_t cap0_sw: 1;                    /*Edge of last capture trigger on channel 0  0: posedge  1: negedge*/
        uint16_t cap1_sw: 1;                    /*Edge of last capture trigger on channel 1  0: posedge  1: negedge*/
        uint16_t cap2_sw: 1;                    /*Edge of last capture trigger on channel 2  0: posedge  1: negedge*/
            };
    uint16_t val;
} cap_source_u;

typedef struct {
    union {
        cap_source_u mcpwm0;
        cap_source_u mcpwm1;
    };
    uint32_t val;
}cap_source_sw_t;
//
// Konstanten
//
typedef  enum {
    DR_LINKS,
    DR_RECHRS
}dr_Bahn_t;

typedef  enum{
    DR_LICHTSCHR1,
    DR_LICHTSCHR2,
    DR_LICHTSCHR3
}dr_Lichtschr_t;

typedef enum {
    NICHT_DURCHFAHREN,
    DURCHFAHREN
}dr_lschr_durchfahrt;

typedef  mcpwm_capture_signal_t dr_Start_Capture_Chanel_t ;

//
//Status
//
typedef union {
    struct {
        uint8_t Lichschr1:1;
        uint8_t Lichschr2:1;
        uint8_t Lichschr3:1;
        uint8_t Fruehstart:1;
        uint8_t Sieg:1;
    };
    uint8_t val;
}dr_Bahn_Status_t;

typedef struct {
    union {
        struct {
            uint8_t Gestartet: 1;
            uint8_t Laeuft: 1;
            uint8_t Ready: 1;
            uint8_t Fertig: 1;
        };
        uint8_t val;
    };
    dr_Bahn_Status_t LinkeBahn;
    dr_Bahn_Status_t RechteBahn;
} dr_Status_t;


//Zeiten
typedef struct {
    uint32_t Lichtschr1;
    uint32_t Lichtschr2;
    uint32_t Lichtschr3;
}dr_time_lichtschr_t;

typedef struct {
    uint32_t Time_Start;
    dr_time_lichtschr_t Links;
    dr_time_lichtschr_t Rechts;
}dr_Zeiten_t;

extern dr_Zeiten_t Zeiten;

typedef struct {
    dr_Status_t Status;
    dr_Zeiten_t Zeiten;
    char dragrace_Json_String[500];
}dr_Dragrace_t;

extern dr_Dragrace_t dragrace;
extern uint32_t Zahl;



//DEBUGG
extern volatile uint32_t  Drag_mcpwm_intr_status;
extern volatile uint32_t  Drag_mcpwm_intr_clr;

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;



static void mcpwm_example_gpio_initialize();
static void gpio_test_signal(void *arg);

static void disp_captured_signal(void *arg);
static void mcpwm_example_config(void *arg);
void mcpwm_setup();
void drag_start();
void neu();
void L1();
void L2();
void L3();


#ifdef __cplusplus
}
#endif


#endif