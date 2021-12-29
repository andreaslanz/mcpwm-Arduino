#include <esp_types.h>
#include "include/main.h"
#include "esp_timer.h"
#include <esp_log.h>
#define  LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define  LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "include/zeit.h"
#include "include/display.h"
#include "include/utility.h"
#include <cJSON.h>

xQueueHandle cap_queue;
#if MCPWM_EN_CAPTURE
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#endif

// Interrupt Test
#define DRAGRACE_INTERRUPT_TEST 0


uint32_t Zahl;
dr_Dragrace_t dragrace;

cap_source_sw_t soft_cap_intr;

static const char *TAG = "zeit.c";
static const char *DRAG = "\033[0;31m\t....Dragrace...";
const char* mcpwm_capture_signal_string[]={"L1","L2","L3","R1","R2","R3"};

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
    cJSON_AddNumberToObject(root,"Status",dragrace.Status.all);
    cJSON_AddNumberToObject(root,"Start",dragrace.Zeiten.Time_Start);
    cJSON_AddNumberToObject(root,"Zeit_L1",dragrace.Zeiten.Links.Lichtschr1);
    cJSON_AddNumberToObject(root,"Zeit_L2",dragrace.Zeiten.Links.Lichtschr2);
    cJSON_AddNumberToObject(root,"Zeit_L3",dragrace.Zeiten.Links.Lichtschr3);
    out = cJSON_Print(root);
    strcpy(dragrace.dragrace_Json_String,out);
    //ESP_LOGI(TAG,"json:%s",out);
    free(out);
    cJSON_Delete(root);
}


static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    ///Interrupt Test Ausgang
#if DRAGRACE_INTERRUPT_TEST
    dragrace_set_Test_Pin_as_Output(DRAGRACE_PIN_TEST_L1_OUTPUT);
#endif
#if MCPWM_GPIO_INIT
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_0, GPIO_CAP0_R_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, GPIO_CAP1_R_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_2, GPIO_CAP2_R_IN);

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
        .mcpwm_cap2_in_num   = GPIO_CAP2_L_IN
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
#endif
    gpio_pulldown_en(GPIO_CAP0_L_IN);    //Enable pull down on CAP0   signal
    gpio_pulldown_en(GPIO_CAP1_L_IN);    //Enable pull down on CAP1   signal
    gpio_pulldown_en(GPIO_CAP2_L_IN);    //Enable pull down on CAP2   signal
    gpio_pulldown_en(GPIO_CAP0_R_IN);    //Enable pull down on CAP0   signal
    gpio_pulldown_en(GPIO_CAP1_R_IN);    //Enable pull down on CAP1   signal
    gpio_pulldown_en(GPIO_CAP2_R_IN);    //Enable pull down on CAP2   signal
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
#define LEN 9
void dragrace_show(){
    ESP_LOGI(TAG, "Status: L1:%d L2:%d L3:%d Frühstart-Links:%d", dragrace.Status.LinkeBahn.Lichschr1, dragrace.Status.LinkeBahn.Lichschr2, dragrace.Status.LinkeBahn.Lichschr3,dragrace.Status.LinkeBahn.Fruehstart);
    ESP_LOGI(TAG, "Status: R1:%d R2:%d R3:%d Frühstart-Rechts:%d", dragrace.Status.RechteBahn.Lichschr1, dragrace.Status.RechteBahn.Lichschr2, dragrace.Status.RechteBahn.Lichschr3,dragrace.Status.RechteBahn.Fruehstart);
    ESP_LOGI(TAG, "Status:  Ready:%d Gestartet:%d Läuft:%d Fertig:%d ", dragrace.Status.Ready, dragrace.Status.Gestartet, dragrace.Status.Laeuft, dragrace.Status.Fertig);
    ESP_LOGI(TAG, "Zeiten: Start:%*u L1:%u L2:%u L3:%u  ",LEN, dragrace.Zeiten.Time_Start, dragrace.Zeiten.Links.Lichtschr1, dragrace.Zeiten.Links.Lichtschr2, dragrace.Zeiten.Links.Lichtschr3);
    ESP_LOGI(TAG, "Zeiten:           R1:%u R2:%u R3:%u  ",  dragrace.Zeiten.Rechts.Lichtschr1, dragrace.Zeiten.Rechts.Lichtschr2, dragrace.Zeiten.Rechts.Lichtschr3);
    ESP_LOGI(TAG, "Div: %u ",  dragrace.Zeiten.Links.Lichtschr1 -dragrace.Zeiten.Rechts.Lichtschr1);
    convert_to_json();
}

/**
 *
 * @brief Auswertung des Interrupts
 *
 */
_Noreturn void IRAM_ATTR disp_captured_signal(void *arg)
{
    capture evt;
    static long count=0;


    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);

        /**
         *
         * Rechte BAHN
         *
         * */
        //! 1. Lichtschranke Rechts (Start)
        if (evt.sel_cap_signal == MCPWM_UNIT1_SELECT_CAP0) {

            if(dragrace.Status.Gestartet || dragrace.Status.Laeuft || dragrace.Status.Ready){
                dragrace.Status.RechteBahn.Lichschr1 = DURCHFAHREN;
                dragrace.Zeiten.Rechts.Lichtschr1= evt.capture_signal;
                ///            "----------------|-------------|------------"
                ESP_LOGI(DRAG, "Lichschr. %s    |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);

//                if(dragrace.Zeiten.Links.Lichtschr1 < dragrace.Zeiten.Time_Start || dragrace.Zeiten.Time_Start==0)
//                    dragrace.Status.LinkeBahn.Fruehstart=1;

                if(Zeiten.Rechts.Lichtschr1-Zeiten.Time_Start<0)
                    dragrace.Status.RechteBahn.Fruehstart=1;
            }
        }
        //! 2. Lichtschranke Rechts (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_UNIT1_SELECT_CAP1) {
            if(dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
                dragrace.Zeiten.Rechts.Lichtschr2=evt.capture_signal;
                dragrace.Status.RechteBahn.Lichschr2=DURCHFAHREN;
                ESP_LOGI(DRAG, "Lichschr. %s    |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);

            }
        }

        //!  3. Lichtschranke Rechts (Ziel) / zugleich Renn-Start
        if (evt.sel_cap_signal == MCPWM_UNIT1_SELECT_CAP2) {
            dragrace.Zeiten.Rechts.Lichtschr3=evt.capture_signal;
            dragrace.Status.RechteBahn.Lichschr3=DURCHFAHREN;
            ESP_LOGI(DRAG, "Lichschr. %s    |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);

        }


        /**
         *
         * LINKE BAHN
         *
         * */

        //! 1. Lichtschranke Links (Start)
        if (evt.sel_cap_signal == MCPWM_UNIT0_SELECT_CAP0) {

            if(dragrace.Status.Gestartet || dragrace.Status.Laeuft || dragrace.Status.Ready){
                dragrace.Status.LinkeBahn.Lichschr1 = DURCHFAHREN;
                dragrace.Zeiten.Links.Lichtschr1= evt.capture_signal;
                ESP_LOGI(DRAG, "Lichschr. %s    |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);

//                if(dragrace.Zeiten.Links.Lichtschr1 < dragrace.Zeiten.Time_Start || dragrace.Zeiten.Time_Start==0)
//                    dragrace.Status.LinkeBahn.Fruehstart=1;

                if(Zeiten.Links.Lichtschr1-Zeiten.Time_Start<0)
                    dragrace.Status.LinkeBahn.Fruehstart=1;
            }
        }

        //! 2. Lichtschranke Links (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_UNIT0_SELECT_CAP1) {
            if(dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
                dragrace.Zeiten.Links.Lichtschr2=evt.capture_signal;
                dragrace.Status.LinkeBahn.Lichschr2=DURCHFAHREN;
                ESP_LOGI(DRAG, "Lichschr. %s   |  %*u  |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);

            }
        }

        //!  3. Lichtschranke Links (Ziel) / zugleich Renn-Start()
        if (evt.sel_cap_signal == MCPWM_UNIT0_SELECT_CAP2) {

            if ( dragrace.Status.LinkeBahn.Start){
                // Lichtschranke 3 Ziel
                dragrace.Zeiten.Links.Lichtschr3=evt.capture_signal;
                dragrace.Status.LinkeBahn.Lichschr3=DURCHFAHREN;
                ESP_LOGI(DRAG, "Lichschr. %s    |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);

            }else{

                // Start Signal (Software Interrupt)
                dragrace.Status.Gestartet=false;
                dragrace.Status.Laeuft=true;
                dragrace.Zeiten.Links.Start= evt.capture_signal;
                dragrace.Status.LinkeBahn.Start=1;
                dragrace.Status.Ready=false;
                ESP_LOGI(DRAG, "Start     %s    |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);


                //Fehlstart?
                if(dragrace.Status.LinkeBahn.Lichschr1 == DURCHFAHREN){
                    if(dragrace.Zeiten.Links.Lichtschr1 < dragrace.Zeiten.Time_Start)
                        dragrace.Status.LinkeBahn.Fruehstart=true;
                    ESP_LOGI(DRAG, "ReaktionsZeit%s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal-dragrace.Zeiten.Links.Start);
                }



            }
        }

        //dragrace_show();
    }
}

#if MCPWM_EN_CAPTURE
/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
 int first=1;
static void IRAM_ATTR isr_handler()
{
    uint32_t mcpwm_unit0_intr_status;
    uint32_t mcpwm_unit1_intr_status;
    BaseType_t xHigherPriorityTaskWoken;
    capture evt;
    uint32_t status;
    uint32_t pins;

    xHigherPriorityTaskWoken = pdFALSE;
    interupt_count++;


    /**Read interrupt status Links*/
    mcpwm_unit0_intr_status =  MCPWM[MCPWM_UNIT_0]->int_st.val;
    status=mcpwm_unit0_intr_status &(CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN);
    status=status>>27;
    ESP_EARLY_LOGD(TAG,"staL %d ct:%d",status,interupt_count );  /**for Debuging in isr (#define LOG_LOCAL_LEVEL in (this) local file)  */

    /**Read interrupt status Rechts*/
    mcpwm_unit1_intr_status = MCPWM[MCPWM_UNIT_1]->int_st.val;
    status=mcpwm_unit1_intr_status &(CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN);
    status=status>>27;
    ESP_EARLY_LOGD(TAG,"staR %d ct:%d",status,interupt_count );  /**for Debuging in isr (#define LOG_LOCAL_LEVEL in (this) local file)  */


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

    /*!
     * Linke Bahn
     * */
    /**Check for interrupt on rising edge on CAP0 signal original*/
    if (mcpwm_unit0_intr_status & CAP0_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP0-L");
    }
    /**Check for interrupt on rising edge on CAP1 signal original*/
    if (mcpwm_unit0_intr_status & CAP1_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP1]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP1-L");
    }
    /**Check for interrupt on rising edge on CAP2 signal original*/
    if (mcpwm_unit0_intr_status & CAP2_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP2]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP2-L");
    }

    /*!
     * Rechte Bahn
     * */
    /**Check for interrupt on rising edge on CAP0 signal original*/
    if (mcpwm_unit1_intr_status & CAP0_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP0-R");
    }
    /**Check for interrupt on rising edge on CAP1 signal original*/
    if (mcpwm_unit1_intr_status & CAP1_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_val_ch[MCPWM_SELECT_CAP1]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP1-R");
    }
    /**Check for interrupt on rising edge on CAP2 signal original*/
    if (mcpwm_unit1_intr_status & CAP2_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_val_ch[MCPWM_SELECT_CAP2]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP2-R");
    }

    /**Clear Interuppt*/
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_unit0_intr_status;
    MCPWM[MCPWM_UNIT_1]->int_clr.val = mcpwm_unit1_intr_status;
    //return xHigherPriorityTaskWoken == pdTRUE;

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
    //Capture configuration
    mcpwm
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    MCPWM[MCPWM_UNIT_1]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
    mcpwm_isr_register(MCPWM_UNIT_1, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
#endif
    vTaskDelete(NULL);
}
/**Software Interrupts für Test*/
void neu(){
    ESP_LOGI(DRAG,"************** NEU **********************");
    ESP_LOGI(DRAG,"Ereignis        |  Linke Bahn | Rechte Bahn");
    ESP_LOGI(DRAG,"----------------|-------------|------------");

//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second

    dragrace.Status.all=0;
    dragrace.Status.Ready=1;
    dragrace.Zeiten.Time_Start=0;
    dragrace.Zeiten.Links.Start=0;
    dragrace.Zeiten.Links.Lichtschr1=0;
    dragrace.Zeiten.Links.Lichtschr2=0;
    dragrace.Zeiten.Links.Lichtschr3=0;
    dragrace.Zeiten.Rechts.Start=0;
    dragrace.Zeiten.Rechts.Lichtschr1=0;
    dragrace.Zeiten.Rechts.Lichtschr2=0;
    dragrace.Zeiten.Rechts.Lichtschr3=0;

    //dragrace_show();
}
void drag_start(){
    if(!dragrace.Status.Laeuft){
        dragrace.Status.Gestartet=true;
    }
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[2].sw=1;
//    dragrace_show();
}
void L1(){
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[MCPWM_SELECT_CAP0].sw=1;
//    dragrace_show();
}
void L2(){
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[MCPWM_SELECT_CAP1].sw=1;
//    dragrace_show();
}
void L3(){
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[MCPWM_SELECT_CAP2].sw=1;
//    dragrace_show();
}
void R1(){
    MCPWM[MCPWM_UNIT_1]->cap_cfg_ch[MCPWM_SELECT_CAP0].sw=1;
//    dragrace_show();
}
void R2(){
    MCPWM[MCPWM_UNIT_1]->cap_cfg_ch[MCPWM_SELECT_CAP1].sw=1;
//    dragrace_show();
}
void R3(){
    MCPWM[MCPWM_UNIT_1]->cap_cfg_ch[MCPWM_SELECT_CAP2].sw=1;
//    dragrace_show();
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

