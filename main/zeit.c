#include <esp_types.h>
#include "include/main.h"
#include "esp_timer.h"
#define  LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define  LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include "include/zeit.h"
#include "include/display.h"
#include <cJSON.h>

xQueueHandle cap_queue;
#if MCPWM_EN_CAPTURE
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#endif

uint32_t Zahl;
dr_Dragrace_t dragrace;

cap_source_sw_t soft_cap_intr;

static const char *TAG = "zeit.c";

//DEBUGG
static volatile IRAM_ATTR int interupt_count=0;
volatile uint32_t  Drag_mcpwm_intr_status;
volatile uint32_t  Drag_mcpwm_intr_clr;
static volatile uint32_t  time;
//extern void inline ets_delay_us(uint32_t t);

/** Convert to json*/
void convert_to_json() {
    cJSON *root = NULL;
    char *out = NULL;
    root =cJSON_CreateObject();
    cJSON_AddNumberToObject(root,"Status",dragrace.Status.val);
    cJSON_AddNumberToObject(root,"Status_Links",dragrace.Status.LinkeBahn.val);
    cJSON_AddNumberToObject(root,"Start",dragrace.Zeiten.Time_Start);
    cJSON_AddNumberToObject(root,"Zeit_L1",dragrace.Zeiten.Links.Lichtschr1);
    cJSON_AddNumberToObject(root,"Zeit_L2",dragrace.Zeiten.Links.Lichtschr2);
    cJSON_AddNumberToObject(root,"Zeit_L3",dragrace.Zeiten.Links.Lichtschr3);
    out = cJSON_Print(root);
    strcpy(dragrace.dragrace_Json_String,out);
    ESP_LOGD(TAG,"json:%s",out);
    free(out);
    cJSON_Delete(root);
}


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
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_IN);
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
    gpio_pulldown_en(GPIO_CAP2_IN);    //Enable pull down on CAP2   signal
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

void dragrace_show(){
    ESP_LOGI(TAG, "Status: L1:%d L2:%d L3:%d Frühstart-Links:%d", dragrace.Status.LinkeBahn.Lichschr1, dragrace.Status.LinkeBahn.Lichschr2, dragrace.Status.LinkeBahn.Lichschr3,dragrace.Status.LinkeBahn.Fruehstart);
    ESP_LOGI(TAG, "Status:  Ready:%d Gestartet:%d Läuft:%d Fertig:%d ", dragrace.Status.Ready, dragrace.Status.Gestartet, dragrace.Status.Laeuft, dragrace.Status.Fertig);
    ESP_LOGI(TAG, "Zeiten: Start:%u L1:%u L2:%u L3:%u  ", dragrace.Zeiten.Time_Start, dragrace.Zeiten.Links.Lichtschr1, dragrace.Zeiten.Links.Lichtschr2, dragrace.Zeiten.Links.Lichtschr3);
    convert_to_json();
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

            if(dragrace.Status.Gestartet || dragrace.Status.Laeuft || dragrace.Status.Ready){


                dragrace.Status.LinkeBahn.Lichschr1 = DURCHFAHREN;

                dragrace.Zeiten.Links.Lichtschr1= evt.capture_signal;

                ESP_LOGD(TAG, "val:%d", dragrace.Zeiten.Links.Lichtschr1);


                if(dragrace.Zeiten.Links.Lichtschr1 < dragrace.Zeiten.Time_Start || dragrace.Zeiten.Time_Start==0)

                    dragrace.Status.LinkeBahn.Fruehstart=1;

            }

            // Disabel Interrupt
//            mcpwm_capture_disable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);  // Eingang sperren

            Zahl = current_cap_value[0];
            /*current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] / 80);*/
        }

        // 2. Lichtschranke (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1) {
            if(dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
                dragrace.Zeiten.Links.Lichtschr2=evt.capture_signal;
                dragrace.Status.LinkeBahn.Lichschr2=DURCHFAHREN;

            }

        }

        //  3. Lichtschranke (Ziel) / zugleich Renn-Start(Soft-Interrupt)
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP2) {

            // Start Signal (Software Interrupt)
//            if(dragrace.Status.Laeuft == false && dragrace.Status.Gestartet==true){
            if(dragrace.Status.Ready){
                dragrace.Status.Gestartet=false;
                dragrace.Status.Laeuft=true;
                dragrace.Zeiten.Time_Start= evt.capture_signal;
                dragrace.Status.Ready=false;

                //Fehlstart?
                if(dragrace.Status.LinkeBahn.Lichschr1 == DURCHFAHREN){
                    if(dragrace.Zeiten.Links.Lichtschr1 < dragrace.Zeiten.Time_Start)
                        dragrace.Status.LinkeBahn.Fruehstart=true;
                }
            }else{

                // Lichtschranke 3
    //            if(dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
                if(dragrace.Status.Laeuft == true ){
                    dragrace.Zeiten.Links.Lichtschr3=evt.capture_signal;
                    dragrace.Status.LinkeBahn.Lichschr3=DURCHFAHREN;

                }
            }





        }

        dragrace_show();
    }
}

#if MCPWM_EN_CAPTURE
/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
 int first=1;
#define DRAGRACE_INTERRUPT_TEST 0
static void IRAM_ATTR isr_handler()
{
    uint32_t mcpwm_intr_status;
    BaseType_t xHigherPriorityTaskWoken;
    capture evt;
    uint32_t status;
    uint32_t pins;

    xHigherPriorityTaskWoken = pdFALSE;
    interupt_count++;


    /**Read interrupt status*/
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val;
    status=mcpwm_intr_status &(CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN);
    status=status>>27;
    ESP_EARLY_LOGD(TAG,"sta %d ct:%d",status,interupt_count );  /**for Debuging in isr (#define LOG_LOCAL_LEVEL in (this) local file)  */


#if DRAGRACE_INTERRUPT_TEST
    /**Test Interrupt während Interrupt auslösen*/
    if(first){
        first=0;
        pins=(BIT(DRAGRACE_PIN_TEST_L1_OUTPUT));
        *(uint32_t *)GPIO_OUT_W1TC_REG=pins; //Low
        ets_delay_us(10);
        *(uint32_t *)GPIO_OUT_W1TS_REG=pins; //High
        ets_delay_us(10);
        *(uint32_t *)GPIO_OUT_W1TC_REG=pins; //Low
    }
#endif

    /**Check for interrupt on rising edge on CAP0 signal original*/
    if (mcpwm_intr_status & CAP0_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP0");
    }

    /**Check for interrupt on rising edge on CAP1 signal original*/
    if (mcpwm_intr_status & CAP1_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP1]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP1");
    }

    /**Check for interrupt on rising edge on CAP2 signal original*/
    if (mcpwm_intr_status & CAP2_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP2]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP2");
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
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
#endif
    vTaskDelete(NULL);
}
/**Software Interrupts für Test*/
void neu(){
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second

    dragrace.Status.val=0;
    dragrace.Status.Ready=1;
    dragrace.Status.LinkeBahn.val=0;
    dragrace.Status.RechteBahn.val=0;
    dragrace.Zeiten.Links.Lichtschr1=0;
    dragrace.Zeiten.Links.Lichtschr2=0;
    dragrace.Zeiten.Links.Lichtschr3=0;
    dragrace.Zeiten.Time_Start=0;
    dragrace_show();
}
void drag_start(){
    if(!dragrace.Status.Laeuft){
        dragrace.Status.Gestartet=true;
    }
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[2].sw=1;
    soft_cap_intr.mcpwm0.cap0_sw=1;
}
void L1(){
    soft_cap_intr.mcpwm0.cap0_sw=1;
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[MCPWM_SELECT_CAP0].sw=1;
}
void L2(){
    soft_cap_intr.mcpwm0.cap1_sw=1;
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[MCPWM_SELECT_CAP1].sw=1;
}
void L3(){
    soft_cap_intr.mcpwm0.cap2_sw=1;
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[MCPWM_SELECT_CAP2].sw=1;
}
 void mcpwm_setup()
{
    printf("Testing MCPWM...\n ");
    cap_queue = xQueueCreate(50, sizeof(capture)); //comment if you don't want to use capture module
    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module
//    xTaskCreate(gpio_test_signal, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module
    xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);

    //neues Rennen
    neu();
}

