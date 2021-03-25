#include <esp_types.h>
#include "include/main.h"
#include "esp_timer.h"
#define  LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define  LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include "include/zeit.h"
#include "include/display.h"

xQueueHandle cap_queue;
#if MCPWM_EN_CAPTURE
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#endif

uint32_t Zahl;
dr_Dragrace_t Dragrace;

cap_source_sw_t soft_cap_intr;

static const char *TAG = "zeit.c";


//DEBUGG
static volatile IRAM_ATTR int interupt_count=0;
volatile uint32_t  Drag_mcpwm_intr_status;
volatile uint32_t  Drag_mcpwm_intr_clr;
static volatile uint32_t  time;

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
#if MCPWM_GPIO_INIT
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_0, GPIO_SYNC0_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_1, GPIO_SYNC1_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_SYNC_2, GPIO_SYNC2_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, GPIO_FAULT0_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_1, GPIO_FAULT1_IN);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_2, GPIO_FAULT2_IN);
#else
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
        .mcpwm1a_out_num = GPIO_PWM1A_OUT,
        .mcpwm1b_out_num = GPIO_PWM1B_OUT,
        .mcpwm2a_out_num = GPIO_PWM2A_OUT,
        .mcpwm2b_out_num = GPIO_PWM2B_OUT,
        .mcpwm_sync0_in_num  = GPIO_SYNC0_IN,
        .mcpwm_sync1_in_num  = GPIO_SYNC1_IN,
        .mcpwm_sync2_in_num  = GPIO_SYNC2_IN,
        .mcpwm_fault0_in_num = GPIO_FAULT0_IN,
        .mcpwm_fault1_in_num = GPIO_FAULT1_IN,
        .mcpwm_fault2_in_num = GPIO_FAULT2_IN,
        .mcpwm_cap0_in_num   = GPIO_CAP0_IN,
        .mcpwm_cap1_in_num   = GPIO_CAP1_IN,
        .mcpwm_cap2_in_num   = GPIO_CAP2_IN
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
#endif
    gpio_pulldown_en(GPIO_CAP0_IN);    //Enable pull down on CAP0   signal
//    gpio_pulldown_en(GPIO_CAP1_IN);    //Enable pull down on CAP1   signal
//    gpio_pulldown_en(GPIO_CAP2_IN);    //Enable pull down on CAP2   signal
//    gpio_pulldown_en(GPIO_SYNC0_IN);   //Enable pull down on SYNC0  signal
//    gpio_pulldown_en(GPIO_SYNC1_IN);   //Enable pull down on SYNC1  signal
//    gpio_pulldown_en(GPIO_SYNC2_IN);   //Enable pull down on SYNC2  signal
//    gpio_pulldown_en(GPIO_FAULT0_IN);  //Enable pull down on FAULT0 signal
//    gpio_pulldown_en(GPIO_FAULT1_IN);  //Enable pull down on FAULT1 signal
//    gpio_pulldown_en(GPIO_FAULT2_IN);  //Enable pull down on FAULT2 signal
}

/**
 * @brief Set gpio 12 as our test signal that generates high-low waveform continuously, connect this gpio to capture pin.
 */
_Noreturn static void gpio_test_signal(void *arg)
{
    printf("intializing test signal...\n");
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = GPIO_SEL_12;
    gpio_config(&gp);
    while (1) {
        //here the period of test signal is 20ms
        gpio_set_level(GPIO_NUM_12, 1); //Set high
        //printf("on\n");
        vTaskDelay(50);             //delay of 10ms
        gpio_set_level(GPIO_NUM_12, 0); //Set low
        //printf("off\n");
        vTaskDelay(50);         //delay of 10ms
    }
}

/**
 * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
 */
_Noreturn void IRAM_ATTR disp_captured_signal(void *arg)
{
    uint32_t *current_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t));
    uint32_t *previous_cap_value = (uint32_t *)malloc(CAP_SIG_NUM*sizeof(uint32_t));
    capture evt;
    static long count=0;


    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);

        // 1. Lichtschranke (Start)
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {

            Dragrace.Status.LinkeBahn.Lichschr1 = DURCHFAHREN;
            Dragrace.Zeiten.Links.Lichtschr1= evt.capture_signal;
            Dragrace.Status.LinkeBahn.Fruehstart=1;

            ESP_LOGD(TAG, "val:%d\n", current_cap_value[0]);
            Zahl = current_cap_value[0];
            /*current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] / 80);*/
        }

        // 2. Lichtschranke (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1) {
        }

        //  3. Lichtschranke (Ziel) / zugleich Renn-Start(Soft-Interrupt)
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP2) {

            // Start Signal (Software Interrupt)
            if(Dragrace.Status.Gestartet==true){
                Dragrace.Status.Gestartet=false;
                Dragrace.Status.Laeuft=true;
                Dragrace.Zeiten.Time_Start= evt.capture_signal;
                Dragrace.Status.Ready=false;

                //Fehlstart?
                if(Dragrace.Status.LinkeBahn.Lichschr1 == DURCHFAHREN){
                    Dragrace.Status.LinkeBahn.Fruehstart=true;
                }
            }

            // Lichtschranke 3
            if(Dragrace.Status.Laeuft==true){
                Dragrace.Zeiten.Links.Lichtschr3=evt.capture_signal;
                Dragrace.Status.LinkeBahn.Lichschr3=DURCHFAHREN;
            }



        }
    }
}

#if MCPWM_EN_CAPTURE
/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler()
{
    uint32_t mcpwm_intr_status;
    BaseType_t xHigherPriorityTaskWoken;
    capture evt;

    xHigherPriorityTaskWoken = pdFALSE;
    interupt_count++;

    /**Read interrupt status*/
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val;
    ESP_EARLY_LOGD(TAG,"sta %d\n",mcpwm_intr_status);  /**for Debuging in isr (#define LOG_LOCAL_LEVEL in (this) local file)  */

    /**Check for interrupt on rising edge on CAP0 signal original*/
    if (mcpwm_intr_status & CAP0_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP0\n");
    }

    /**Check for interrupt on rising edge on CAP1 signal original*/
    if (mcpwm_intr_status & CAP1_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP1]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP1\n");
    }

    /**Check for interrupt on rising edge on CAP2 signal original*/
    if (mcpwm_intr_status & CAP2_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP2]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP2\n");
    }

    /**Clear Interuppt*/
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR (); /*Gehe direkt zur Verarbeitung*/
    }

}
#endif

/**
 * @brief Configure whole MCPWM module
 */
static void mcpwm_example_config(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();


#if MCPWM_EN_CAPTURE
    //7. Capture configuration
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
#endif
    vTaskDelete(NULL);
}
/**Software Interrupts für Test*/
void start(){
    soft_cap_intr.mcpwm0.cap0_sw=1;
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[0].sw=1;
}
void L1(){
    soft_cap_intr.mcpwm0.cap1_sw=1;

    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[1].sw=1;
}
 void mcpwm_setup()
{
    printf("Testing MCPWM...\n ");
    cap_queue = xQueueCreate(50, sizeof(capture)); //comment if you don't want to use capture module
    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module
//    xTaskCreate(gpio_test_signal, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module
    xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
}

