#include <esp_types.h>
#include "include/main.h"
#include "esp_timer.h"
#include <esp_log.h>
#define  LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define  LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "include/zeit.h"
#include "include/display.h"
#include "include/utility.h"
#include "freertos/semphr.h"
#include <cJSON.h>

xQueueHandle cap_queue;
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

// Interrupt Test
#define DRAGRACE_INTERRUPT_TEST 0

// Semaphore
SemaphoreHandle_t xSemaphore = NULL;

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

void convert_to_json() {
    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
    {
        /* We were able to obtain the semaphore and can now access the
        shared resource. */

        cJSON *root = NULL;
        char *out = NULL;
        root =cJSON_CreateObject();
        cJSON_AddNumberToObject(root,"Status",dragrace.Status.all);
        cJSON_AddNumberToObject(root,"Start_Links",dragrace.Zeiten.Links.Start);
        cJSON_AddNumberToObject(root,"Zeit_L1",dragrace.Zeiten.Links.Lichtschr1);
        cJSON_AddNumberToObject(root,"Zeit_L2",dragrace.Zeiten.Links.Lichtschr2);
        cJSON_AddNumberToObject(root,"Zeit_L3",dragrace.Zeiten.Links.Lichtschr3);
        cJSON_AddNumberToObject(root,"Zeit_R1",dragrace.Zeiten.Rechts.Lichtschr1);
        cJSON_AddNumberToObject(root,"Zeit_R2",dragrace.Zeiten.Rechts.Lichtschr2);
        cJSON_AddNumberToObject(root,"Zeit_R3",dragrace.Zeiten.Rechts.Lichtschr3);
        cJSON_AddNumberToObject(root,"rand",dragrace.Zeiten.Time_Random);
        out = cJSON_Print(root);
        strcpy(dragrace.dragrace_Json_String,out);
        //ESP_LOGI(TAG,"json:%s",out);
        free(out);
        cJSON_Delete(root);

        /* We have finished accessing the shared resource.  Release the
        semaphore. */
        xSemaphoreGive( xSemaphore );
    }

}
void action(void (*f)()){
    f();
    convert_to_json();
}

static void mcpwm_example_gpio_initialize(){
    printf("initializing mcpwm gpio...\n");
    ///Interrupt Test Ausgang
#if DRAGRACE_INTERRUPT_TEST
    dragrace_set_Test_Pin_as_Output(DRAGRACE_PIN_TEST_L1_OUTPUT);
#endif
#if MCPWM_GPIO_INIT
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP2_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_L_IN);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_0, GPIO_CAP0_R_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, GPIO_CAP1_R_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_2, GPIO_CAP2_R_IN);

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
}

/**
 * @brief Set gpio 12 as our test signal that generates high-low waveform continuously, connect this gpio to capture pin.
 */
_Noreturn static void gpio_test_signal(void *arg){
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
#define LEN 10

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
_Noreturn void IRAM_ATTR disp_captured_signal(void *arg){
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
                ESP_LOGI(DRAG, "Lichschr.    %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);
            }
            // Rennen noch nicht gestartet = Frühstart
            if(!dragrace.Status.RechteBahn.Start && evt.sel_cap_signal != MCPWM_UNIT1_SELECT_CAP2) {
                dragrace.Status.RechteBahn.Fruehstart = 1;
                ESP_LOGI(DRAG, "FrühstartRechts |             |");
            }
            // Reaktionszeit
            if( dragrace.Status.RechteBahn.Start == true) {
                ESP_LOGI(DRAG, "ReaktionsZeit R |             |  %*u |",10,evt.capture_signal-dragrace.Zeiten.Rechts.Start);
            }

        }
        //! 2. Lichtschranke Rechts (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_UNIT1_SELECT_CAP1) {
            if(dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
                dragrace.Zeiten.Rechts.Lichtschr2=evt.capture_signal;
                dragrace.Status.RechteBahn.Lichschr2=DURCHFAHREN;
                ESP_LOGI(DRAG, "Lichschr.    %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);

            }
        }

        //!  3. Lichtschranke Rechts (Ziel) / zugleich Renn-Start
        if (evt.sel_cap_signal == MCPWM_UNIT1_SELECT_CAP2) {

            if (dragrace.Status.RechteBahn.Start) {
                // Lichtschranke 3 Ziel
                dragrace.Zeiten.Rechts.Lichtschr3 = evt.capture_signal;
                dragrace.Status.RechteBahn.Lichschr3 = DURCHFAHREN;
                ESP_LOGI(DRAG, "ZielRechts   %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],
                         evt.capture_signal);
                // fertig
                if(dragrace.Status.LinkeBahn.Lichschr3){
                    action(fertig);
                }

            } else {

                // Start Signal (Software Interrupt)
                dragrace.Zeiten.Rechts.Start = evt.capture_signal;
                dragrace.Status.RechteBahn.Start = 1;
                dragrace.Status.Gestartet = false;
                dragrace.Status.Laeuft = true;
                dragrace.Status.Ready = false;
                ESP_LOGI(DRAG, "Start Rechts    |             | %u",  evt.capture_signal);


                //Fehlstart?
                if (dragrace.Status.RechteBahn.Lichschr1 == DURCHFAHREN) {
                    if (dragrace.Zeiten.Rechts.Lichtschr1 < dragrace.Zeiten.Rechts.Start)
                        dragrace.Status.RechteBahn.Fruehstart = true;
                    ESP_LOGI(DRAG, "ReaktionsZeit%s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],
                             evt.capture_signal - dragrace.Zeiten.Rechts. Lichtschr1);
                }
            }
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
                ESP_LOGI(DRAG, "Lichschr.    %s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);

                // Rennen noch nicht gestartet = Frühstart
                if(!dragrace.Status.LinkeBahn.Start && evt.sel_cap_signal != MCPWM_UNIT0_SELECT_CAP2) {
                    dragrace.Status.LinkeBahn.Fruehstart = 1;
                    ESP_LOGI(DRAG, "FrühstartLinks  |             |");
                }
                // Reaktionszeit
                if( dragrace.Status.LinkeBahn.Start == true) {
                    ESP_LOGI(DRAG, "ReaktionsZeit L |  %*u |",10,evt.capture_signal-dragrace.Zeiten.Links.Start);
                }
            }
        }

        //! 2. Lichtschranke Links (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_UNIT0_SELECT_CAP1) {
            if(dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
                dragrace.Zeiten.Links.Lichtschr2=evt.capture_signal;
                dragrace.Status.LinkeBahn.Lichschr2=DURCHFAHREN;
                ESP_LOGI(DRAG, "Lichschr.    %s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);

            }
        }

        //!  3. Lichtschranke Links (Ziel) / zugleich Renn-Start()
        if (evt.sel_cap_signal == MCPWM_UNIT0_SELECT_CAP2) {

            if ( dragrace.Status.LinkeBahn.Start){
                // Lichtschranke 3 Ziel
                dragrace.Zeiten.Links.Lichtschr3=evt.capture_signal;
                dragrace.Status.LinkeBahn.Lichschr3=DURCHFAHREN;
                ESP_LOGI(DRAG, "Ziel Links      |  %*u |", 10,evt.capture_signal);
                // fertig
                if(dragrace.Status.RechteBahn.Lichschr3){
                    action(fertig);
                }

            }else{

                // Start Signal (Software Interrupt)
                dragrace.Zeiten.Links.Start= evt.capture_signal;
                dragrace.Status.LinkeBahn.Start=1;
                dragrace.Status.Gestartet=false;
                dragrace.Status.Laeuft=true;
                dragrace.Status.Ready=false;
                ESP_LOGI(DRAG, "Start Links  %s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal], 10,evt.capture_signal);


                //Fehlstart?
                if(dragrace.Status.LinkeBahn.Lichschr1 == DURCHFAHREN){
                    if(dragrace.Zeiten.Links.Lichtschr1 < dragrace.Zeiten.Links.Start){
                        dragrace.Status.LinkeBahn.Fruehstart=true;
                    }
                    ESP_LOGI(DRAG, "Reaktionszeit%s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal-dragrace.Zeiten.Links.Lichtschr1);
                }
            }
        }

        if(!dragrace.Status.LinkeBahn.Start_Ausgewertet && dragrace.Status.LinkeBahn.Start && dragrace.Status.LinkeBahn.Lichschr1){
            dragrace.Status.LinkeBahn.Start_Ausgewertet=true;
        }

        //dragrace_show();
        convert_to_json();
    }
}

/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP signal and according take action
 */

static void IRAM_ATTR isr_handler(){
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
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP0].capn_value; //get capture signal counter value
//        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP0-L");
    }
    /**Check for interrupt on rising edge on CAP1 signal original*/
    if (mcpwm_unit0_intr_status & CAP1_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP1].capn_value; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP1-L");
    }
    /**Check for interrupt on rising edge on CAP2 signal original*/
    if (mcpwm_unit0_intr_status & CAP2_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP2].capn_value; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP2-L");
    }

    /*!
     * Rechte Bahn
     * */
    /**Check for interrupt on rising edge on CAP0 signal original*/
    if (mcpwm_unit1_intr_status & CAP0_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP0].capn_value; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP0-R");
    }
    /**Check for interrupt on rising edge on CAP1 signal original*/
    if (mcpwm_unit1_intr_status & CAP1_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP1].capn_value; //get capture signal counter value
        evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        ESP_EARLY_LOGD(TAG,"CAP1-R");
    }
    /**Check for interrupt on rising edge on CAP2 signal original*/
    if (mcpwm_unit1_intr_status & CAP2_INT_EN) {
        evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP2].capn_value; //get capture signal counter value
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

/**
 * @brief Configure whole MCPWM module
 */
static void mcpwm_example_config(void *arg){

    // Semaphore
    xSemaphore = xSemaphoreCreateMutex();

    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();


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
    vTaskDelete(NULL);
}
void neu(){
    ESP_LOGI(DRAG,"************** NEU **********************");
    ESP_LOGI(DRAG,"Ereignis        |  Linke Bahn | Rechte Bahn");
    ESP_LOGI(DRAG,"----------------|-------------|------------");

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
    uint32_t r_max =254; //Zufallszahl Maximum
    uint32_t r_min = 100;//Minimum
    uint32_t r = esp_random()%(r_max-r_min);
    r = r + r_min;
    ESP_LOGI(DRAG,"Zufälliger Start in %d.%d Sekunden\n",r/100,r%100);
    dragrace.Zeiten.Time_Random= r;

}
void fertig(){
    dragrace.Status.Fertig=true;
    dragrace.Status.Laeuft=false;
    dragrace_show();
    ESP_LOGI(DRAG,"\n%s\n",dragrace.dragrace_Json_String);

}
void drag_start(){
    if(!dragrace.Status.Laeuft){
        dragrace.Status.Gestartet=true;
        dragrace.Status.Ready=false;
        dragrace_impulse(NULL,0);
    }
}
/**
 * Software Interrupts für Test
 */
void L1(){
    MCPWM[MCPWM_UNIT_0]->cap_chn_cfg[MCPWM_SELECT_CAP0].capn_sw=1;
}
void L2(){
    MCPWM[MCPWM_UNIT_0]->cap_chn_cfg[MCPWM_SELECT_CAP1].capn_sw=1;
}
void L3(){
    MCPWM[MCPWM_UNIT_0]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;
}
void R1(){
    MCPWM[MCPWM_UNIT_1]->cap_chn_cfg[MCPWM_SELECT_CAP0].capn_sw=1;
}
void R2(){
    MCPWM[MCPWM_UNIT_1]->cap_chn_cfg[MCPWM_SELECT_CAP1].capn_sw=1;
}
void R3(){
    MCPWM[MCPWM_UNIT_1]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;
}

void mcpwm_setup() {
    printf("Testing MCPWM...\n ");
    cap_queue = xQueueCreate(50, sizeof(capture)); //comment if you don't want to use capture module
    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5,NULL);
    xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
//     char buf[500];
//     vTaskList(buf);
//     ESP_LOGD(TAG,"*********************************");
//     ESP_LOGD(TAG,"Task         State Prio Stack Num");
//     ESP_LOGD(TAG,"*********************************");
//     ESP_LOGD(TAG,"%s",buf);

    //neues Rennen
    action(neu);
//    neu();
}

