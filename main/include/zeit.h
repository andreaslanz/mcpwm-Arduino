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
#include "freertos/semphr.h"

///Programm im Test oder Produktionsmodus
#define PROGRAMM_MODUS_PRODUCTION  1

#define HARDWARE_START_NEU_BTN_ENABLE   PROGRAMM_MODUS_PRODUCTION
#define STARTSPERRE_BEI_LICHTSCHR_1     PROGRAMM_MODUS_PRODUCTION

#define MCPWM_GPIO_INIT 1    //select which function to use to initialize gpio signals
#define CAP_SIG_NUM 3   //Three capture signals

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit


#define GPIO_CAP0_L_IN   DRAGRACE_PIN_LICHTSCHRANKE_L1_INPUT
#define GPIO_CAP1_L_IN   DRAGRACE_PIN_LICHTSCHRANKE_L2_INPUT
#define GPIO_CAP2_L_IN   DRAGRACE_PIN_LICHTSCHRANKE_L3_INPUT

#define GPIO_CAP0_R_IN   DRAGRACE_PIN_LICHTSCHRANKE_R1_INPUT
#define GPIO_CAP1_R_IN   DRAGRACE_PIN_LICHTSCHRANKE_R2_INPUT
#define GPIO_CAP2_R_IN   DRAGRACE_PIN_LICHTSCHRANKE_R3_INPUT

#define GPIO_CAP_POS_EDGE 0  // Lichtschr.Eing. Positive Flanke
#define GPIO_CAP_NEG_EDGE 1  // Lichtschr.Eing. Negative Flanke
/**
 * @brief MCPWM select capture signal input
 */
typedef enum {
    MCPWM_UNIT0_SELECT_CAP0 = 0, /*!<Select CAP0 as input*/
    MCPWM_UNIT0_SELECT_CAP1,     /*!<Select CAP1 as input*/
    MCPWM_UNIT0_SELECT_CAP2,     /*!<Select CAP2 as input*/
    MCPWM_UNIT1_SELECT_CAP0,     /*!<Select CAP0 as input*/
    MCPWM_UNIT1_SELECT_CAP1,     /*!<Select CAP1 as input*/
    MCPWM_UNIT1_SELECT_CAP2,     /*!<Select CAP2 as input*/
} mcpwm_capture_signal_dragrace_t;

extern const char* mcpwm_capture_signal_string[];


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
    DR_RECHTS
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
        uint8_t Start:1;
        uint8_t Start_Ausgewertet:1;
        uint8_t Ziel_Ausgewertet:1;
        uint8_t Lichschr1:1;
        uint8_t Lichschr2:1;
        uint8_t Lichschr3:1;
        uint8_t Fruehstart:1;
        uint8_t Sieg:1;
    };
    uint8_t val;
}dr_Bahn_Status_t;

typedef union{
    struct {
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
    };
    uint32_t all;
}dr_Status_t;

//Eingänge
typedef struct {
    union {
        struct {
            uint8_t Position_L: 1;
            uint8_t Position_R: 1;
            uint8_t Lichtschranke_L1: 1;
            uint8_t Lichtschranke_L2: 1;
            uint8_t Lichtschranke_L3: 1;
            uint8_t Lichtschranke_R1: 1;
            uint8_t Lichtschranke_R2: 1;
            uint8_t Lichtschranke_R3: 1;
        };
        uint8_t all;
    };
}dr_eingaenge_status;

//Zeiten
typedef struct {
    uint32_t Lichtschr1;
    uint32_t Lichtschr2;
    uint32_t Lichtschr3;
    uint32_t Start;
}dr_time_lichtschr_t;

typedef struct {
    int32_t Time_Start;
    uint8_t Time_Random;
    dr_time_lichtschr_t Links;
    dr_time_lichtschr_t Rechts;
}dr_Zeiten_t;

//Bahnen new
typedef struct {
    uint32_t Zeit_L1;
    uint32_t Zeit_L2;
    uint32_t Zeit_L3;
    uint32_t Zeit_Start;
} dr_bahn_new_t;



//Status new
typedef struct {
    union  {
        struct {
            uint8_t Status_Start: 1; //Startzeit gespeichert bei "Grün"
            uint8_t Status_Laeuft: 1;
            uint8_t Status_Ziel_Ausgewertet: 1;
            uint8_t Status_L1: 1;
            uint8_t Status_L2: 1;
            uint8_t Status_L3: 1;
            uint8_t Status_Fruehstart: 1;
            uint8_t Status_Sieg: 1;
        };
        uint8_t bahn_status_all;
    };
} dr_bahn_status_new_t;

typedef union {
    struct{
        dr_eingaenge_status Eingaenge;
        union {
            struct {
                uint8_t Orange1: 1;
                uint8_t Orange2: 1;
                uint8_t Laeuft_NEW: 1;
                uint8_t Ready_NEW: 1;
                uint8_t Gestartet_NEW: 1;
                uint8_t Fertig_New: 1;
            };
            uint8_t race_status_all;
        };
        dr_bahn_status_new_t bahn_status_new[2] ;


    };
    uint32_t  Status_All;
}dr_status_new_t;

/* Status ist eine 32 - bit Zahl

Bedeutung der einzelnen bit's:


BitNr      Bedeutung
=======================

  Eingänge
0       Position_L
1       Position_R
2       Lichtschranke_L1
3       Lichtschranke_L2
4       Lichtschranke_L3
5       Lichtschranke_R1
6       Lichtschranke_R2
7       Lichtschranke_R3

 Rennen Status
8       Orange1
9       Orange2
10      Laeuft=Grün
11      Ready
12      Gestartet
13      Fertig_New
14      nicht gebraucht
15      nicht gebraucht

 Linke Bahn
16      L_Start
17      L_Laeuft
18      L_Ausgewertet
19      L_L1
20      L_L2
21      L_L3
22      L_Fruehstart
23      L_Sieg

 Rechte Bahn
24      R_Start
25      R_Laeuft
26      R_Ausgewertet
27      R_L1
28      R_L2
29      R_L3
30      R_Fruehstart
31      R_Sieg

*/






//dr_Dragrace_new_t DR;
//
//void f(void){
//    uint32_t  v=DR.Bahn[DR_LINKS].Zeit_L1;
//}

//Dragrace
typedef struct {
    dr_bahn_new_t Zeiten_new[2];
    dr_status_new_t Status_new;
    bool debugg;
    uint32_t randomstart;
    dr_Status_t Status_old;
    dr_Zeiten_t Zeiten;
    dr_eingaenge_status Eingaenge;
    char dragrace_Json_String[500];
}dr_Dragrace_t;

extern  dr_Dragrace_t dragrace;
extern uint32_t Zahl;
extern dr_Zeiten_t Zeiten;



//DEBUGG
extern volatile uint32_t  Drag_mcpwm_intr_status;
extern volatile uint32_t  Drag_mcpwm_intr_clr;

// Semaphore
extern SemaphoreHandle_t xSemaphore;

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_dragrace_t sel_cap_signal;
} capture;



static void mcpwm_example_gpio_initialize();
static void gpio_test_signal(void *arg);
//extern mcpwm_dev_t *MCPWM;
static void disp_captured_signal(void *arg);
static void mcpwm_example_config(void *arg);
void dragrace_show(const char* message);
void mcpwm_setup();
void drag_start();
void neu();
void fertig();
void L1();
void L2();
void L3();
void R1();
void R2();
void R3();
void action(void (*f)());


#ifdef __cplusplus
}
#endif


#endif