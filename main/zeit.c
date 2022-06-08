/*
 *
 *
 *
 * Dragrace Zeitmessung
 *
 *
 *
 * */


#include <esp_types.h>
#include "include/main.h"
#include "esp_timer.h"
#include <esp_log.h>
//#define  LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define  LOG_LOCAL_LEVEL ESP_LOG_WARN
//#define  LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "include/zeit.h"
#include "include/utility.h"
#include "freertos/semphr.h"
#include <cJSON.h>
#include "Arduino.h"

xQueueHandle cap_queue;  //Ereignisse
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1}; //MCPWM Register

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
const char* mcpwm_capture_fehlstart_string[]={"Fehlstart Links", "Fehlstart Rechts"};
const char* mcpwm_capture_sieg_string[]={     "Sieg Links",      "Sieg Rechts"};
const char* mcpwm_capture_start_string[]={     "Start Links",      "Start Rechts"};
const char* mcpwm_capture_fertig_string[]={   "Fertig",          "Fertig"};

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
        cJSON_AddNumberToObject(root,"Status",dragrace.Status_old.all);
        cJSON_AddNumberToObject(root,"Start_Links",dragrace.Zeiten.Links.Start);
        cJSON_AddNumberToObject(root,"Start_Rechts",dragrace.Zeiten.Rechts.Start);
        cJSON_AddNumberToObject(root,"Zeit_L1",dragrace.Zeiten.Links.Lichtschr1);
        cJSON_AddNumberToObject(root,"Zeit_L2",dragrace.Zeiten.Links.Lichtschr2);
        cJSON_AddNumberToObject(root,"Zeit_L3",dragrace.Zeiten.Links.Lichtschr3);
        cJSON_AddNumberToObject(root,"Zeit_R1",dragrace.Zeiten.Rechts.Lichtschr1);
        cJSON_AddNumberToObject(root,"Zeit_R2",dragrace.Zeiten.Rechts.Lichtschr2);
        cJSON_AddNumberToObject(root,"Zeit_R3",dragrace.Zeiten.Rechts.Lichtschr3);
        cJSON_AddNumberToObject(root,"Eingaenge",dragrace.Eingaenge.all);
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

static void mcpwm_example_gpio_initialize() {
    printf("initializing mcpwm gpio...\n");
    // Log Level
    esp_log_level_set("gpio", ESP_LOG_INFO);

    ///Interrupt Test Ausgang
#if DRAGRACE_INTERRUPT_TEST
    dragrace_set_Test_Pin_as_Output(DRAGRACE_PIN_TEST_L1_OUTPUT);
#endif
#if MCPWM_GPIO_INIT
//    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP2_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_0, GPIO_CAP0_R_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_L_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, GPIO_CAP1_R_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_L_IN);
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
#if GPIO_CAP_POS_EDGE || GPIO_CAP_NEG_EDGE
    gpio_pulldown_en(GPIO_CAP0_L_IN);    //Enable pull down on CAP0   signal
    gpio_pulldown_en(GPIO_CAP1_L_IN);    //Enable pull down on CAP1   signal
    gpio_pulldown_en(GPIO_CAP2_L_IN);    //Enable pull down on CAP2   signal
    gpio_pulldown_en(GPIO_CAP0_R_IN);    //Enable pull down on CAP0   signal
    gpio_pulldown_en(GPIO_CAP1_R_IN);    //Enable pull down on CAP1   signal
    gpio_pulldown_en(GPIO_CAP2_R_IN);    //Enable pull down on CAP2   signal
#endif
//#if GPIO_CAP_NEG_EDGE
//    gpio_pullup_en(GPIO_CAP0_L_IN);    //Enable pull down on CAP0   signal
//    gpio_pullup_en(GPIO_CAP1_L_IN);    //Enable pull down on CAP1   signal
//    gpio_pullup_en(GPIO_CAP2_L_IN);    //Enable pull down on CAP2   signal
//    gpio_pullup_en(GPIO_CAP0_R_IN);    //Enable pull down on CAP0   signal
//    gpio_pullup_en(GPIO_CAP1_R_IN);    //Enable pull down on CAP1   signal
//    gpio_pullup_en(GPIO_CAP2_R_IN);    //Enable pull down on CAP2   signal
//#endif

/// Ein- Aus-Gänge Initialisieren

    ///EINGAENGE
    dragrace_set_Test_Pin_as_Input(1ULL<<DRAGRACE_PIN_POSITION_LINKS_INPUT);        //Position in Links
    dragrace_set_Test_Pin_as_Input(1ULL<<DRAGRACE_PIN_POSITION_RECHTS_INPUT);       //Position in Rechts
    dragrace_set_Test_Pin_as_Input(1ULL<<DRAGRACE_PIN_NEU_INPUT);                   //Neu in
    dragrace_set_Test_Pin_as_Input(1ULL<<DRAGRACE_PIN_START_INPUT);                 //Start in

    ///AUSGAENGE
    dragrace_set_Test_Pin_as_Output(1ULL << DRAGRACE_PIN_POSITION_LAMPE_L_OUTPUT);  //Positions out Links
    dragrace_set_Test_Pin_as_Output(1ULL << DRAGRACE_PIN_POSITION_LAMPE_R_OUTPUT);  //Positions out Rechts
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_FEHLSTART_LINKS_OUTPUT);     //FrühStart out Links
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_FEHLSTART_RECHTS_OUTPUT);    //FrühStart out Rechts
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_SIEG_LAMPE_R_OUTPUT);        //Sieg out Rechts
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_SIEG_LAMPE_L_OUTPUT);        //Sieg out Links
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_ORANG1_LAMPE_OUTPUT);        //Orange1 out
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_ORANG2_LAMPE_OUTPUT);        //Orange2 out
    dragrace_set_Test_Pin_as_Output(1ULL<<DRAGRACE_PIN_GRUEN_LAMPE_OUTPUT);         //Grün out
    gpio_set_level(DRAGRACE_PIN_FEHLSTART_LINKS_OUTPUT,0);
}

/**
 * @brief Set gpio 12 as our test signal that generates high-low waveform continuously, connect this gpio to capture pin.
 */
_Noreturn static void gpio_test_signal(void *arg){
    printf("initializing test signal...\n");
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

void dragrace_show(const char* message){
    printf("%*s,%*u,%*u,%*u,%*u,%*u,%*u,%*u,%*u,%*u,%*u\n",
           16,message,
           10,dragrace.Status_new.Status_All,
           10,dragrace.Zeiten_new[DR_LINKS].Zeit_Start,
           10,dragrace.Zeiten_new[DR_LINKS].Zeit_L1,
           10,dragrace.Zeiten_new[DR_LINKS].Zeit_L2,
           10,dragrace.Zeiten_new[DR_LINKS].Zeit_L3,
           10,dragrace.Zeiten_new[DR_RECHTS].Zeit_Start,
           10,dragrace.Zeiten_new[DR_RECHTS].Zeit_L1,
           10,dragrace.Zeiten_new[DR_RECHTS].Zeit_L2,
           10,dragrace.Zeiten_new[DR_RECHTS].Zeit_L3,
           10,interupt_count);
//    ESP_LOGI(TAG, "Status: L1:%d L2:%d L3:%d Frühstart-Links:%d", dragrace.Status.LinkeBahn.Lichschr1, dragrace.Status.LinkeBahn.Lichschr2, dragrace.Status.LinkeBahn.Lichschr3,dragrace.Status.LinkeBahn.Fruehstart);
//    ESP_LOGI(TAG, "Status: R1:%d R2:%d R3:%d Frühstart-Rechts:%d", dragrace.Status.RechteBahn.Lichschr1, dragrace.Status.RechteBahn.Lichschr2, dragrace.Status.RechteBahn.Lichschr3,dragrace.Status.RechteBahn.Fruehstart);
//    ESP_LOGI(TAG, "Status:  Ready:%d Gestartet:%d Läuft:%d Fertig:%d ", dragrace.Status.Ready, dragrace.Status.Gestartet, dragrace.Status.Laeuft, dragrace.Status.Fertig);
//    ESP_LOGI(TAG, "Zeiten: Start:%*u L1:%u L2:%u L3:%u  ",LEN, dragrace.Zeiten.Time_Start, dragrace.Zeiten.Links.Lichtschr1, dragrace.Zeiten.Links.Lichtschr2, dragrace.Zeiten.Links.Lichtschr3);
//    ESP_LOGI(TAG, "Zeiten:           R1:%u R2:%u R3:%u  ",  dragrace.Zeiten.Rechts.Lichtschr1, dragrace.Zeiten.Rechts.Lichtschr2, dragrace.Zeiten.Rechts.Lichtschr3);
//    ESP_LOGI(TAG, "Div: %u ",  dragrace.Zeiten.Links.Lichtschr1 -dragrace.Zeiten.Rechts.Lichtschr1);
//    convert_to_json();
}

/**
 *
 * @brief Auswertung des Interrupts
 *
 */
_Noreturn void IRAM_ATTR disp_captured_signal(void *arg){
    capture evt;
    static long count=0;
    const char *message;
    static dr_bahn_status_new_t * bs []= { &dragrace.Status_new.bahn_status_new[DR_LINKS],
                                           &dragrace.Status_new.bahn_status_new[DR_RECHTS]};

    while (1) {
        //esp_log_level_set(DRAG, ESP_LOG_ERROR);

        xQueueReceive(cap_queue, &evt, portMAX_DELAY);


        /// New

        uint32_t mcpwm_unit=       evt.sel_cap_signal/3; /// Linker oder Rechter Kanal
        uint32_t mcpwm_cap_int_no= evt.sel_cap_signal%3; /// Lichtschr. Nr 1-3
        //ESP_LOGI(DRAG, "Unit.    %d Kanal %d\n",mcpwm_unit,mcpwm_cap_int_no);


        /// Nur wenn Rennen gestartet wurde!!
        if( !(dragrace.Status_new.Gestartet_NEW || dragrace.Status_new.Orange1 || dragrace.Status_new.Orange2 || dragrace.Status_new.Laeuft_NEW) )
            continue;



        /// Lichtschranke 1 --------------------------------------------------------------------------------------------
        if(mcpwm_cap_int_no == 0){

            if(  !bs[mcpwm_unit]->Status_L1){
//            if(  !dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L1){

                dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L1 = 1;
                dragrace.Zeiten_new[mcpwm_unit].Zeit_L1 =   evt.capture_signal;
                //message
                dragrace_show(mcpwm_capture_signal_string[evt.sel_cap_signal]) ;

                //                 "----------------|-------------|------------"
                //                 "Lichschr.    %s |  %*u |"
                //                 "Lichschr.    %s |             | %u"
                if(mcpwm_unit==DR_LINKS)
                    ESP_LOGI(DRAG, "Lichschr.    %s |  %*u |",           mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);
                if(mcpwm_unit==DR_RECHTS)
                    ESP_LOGI(DRAG, "Lichschr.    %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);

                // Rennen noch nicht gestartet = Frühstart
                if(dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Start == 0) {
                    dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Fruehstart = 1;
                    if(mcpwm_unit==DR_LINKS)
                        ESP_LOGI(DRAG, "FrühstartLinks  |             |");
                    if(mcpwm_unit==DR_RECHTS)
                        ESP_LOGI(DRAG, "FrühstartRechts |             |");
                    //message
                    dragrace_show(mcpwm_capture_fehlstart_string[mcpwm_unit]);
                }

            }

        }
        /// Lichtschranke 2 --------------------------------------------------------------------------------------------
        if(mcpwm_cap_int_no == 1){
            if(!dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L2 && dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Start){
                dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L2 = 1;
                dragrace.Zeiten_new[mcpwm_unit].Zeit_L2 =   evt.capture_signal;
                //             "----------------|-------------|------------"
                //             "Lichschr.    %s |  %*u |"
                //             "Lichschr.    %s |             | %u"
                if(mcpwm_unit==DR_LINKS)
                ESP_LOGI(DRAG, "Lichschr.    %s |  %*u |",           mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);
                if(mcpwm_unit==DR_RECHTS)
                ESP_LOGI(DRAG, "Lichschr.    %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);

                //message
                dragrace_show(mcpwm_capture_signal_string[evt.sel_cap_signal]) ;

            }
        }


        /// Lichtschranke 3 --------------------------------------------------------------------------------------------
        if(mcpwm_cap_int_no == 2){

            /// Lichtschranke 3 Ziel
            if (!dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L3 && dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Start) {
                dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L3 = 1;
                dragrace.Zeiten_new[mcpwm_unit].Zeit_L3 =   evt.capture_signal;
                //message
                dragrace_show(mcpwm_capture_signal_string[evt.sel_cap_signal]) ;

                // Sieg
                if ( !bs[mcpwm_unit]->Status_Fruehstart   ){ //kein Frühstart
                    // anderer Fahrer noch nicht im Ziel oder hat Fehlstart
                    if(   !bs[!mcpwm_unit]->Status_L3 ||  bs[!mcpwm_unit]->Status_Fruehstart ){
                        bs[mcpwm_unit]->Status_Sieg = 1;
                        //message
                        dragrace_show(mcpwm_capture_sieg_string[mcpwm_unit]) ;
                    }

                }

                // fertig
                if(dragrace.Status_new.bahn_status_new[!mcpwm_unit].Status_L3){
                    dragrace.Status_new.Fertig_New=1;
                   fertig();
                }

                //             "----------------|-------------|------------"
                //             "Lichschr.    %s |  %*u |"
                //             "Lichschr.    %s |             | %u"
                if(mcpwm_unit==DR_LINKS)
                    ESP_LOGI(DRAG, "ZielRechts   %s |  %*u  |",          mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal);
                if(mcpwm_unit==DR_RECHTS)
                    ESP_LOGI(DRAG, "ZielRechts   %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal]
                    , evt.capture_signal);

            }

            /// Lichtschranke 3 Start (Lichtschranke 3 wird zugleich als Start durch Hardware oder Software Interrupt benutzt)
            if(!dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Start){
                dragrace.Zeiten_new[mcpwm_unit].Zeit_Start =   evt.capture_signal;
                dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Start = true;
                if(dragrace.Status_new.bahn_status_new[ ! mcpwm_unit].Status_Start == true){ //andere Bahn schon gestartet
                    ///Grüne Lampe ein
                    gpio_set_level(DRAGRACE_PIN_GRUEN_LAMPE_OUTPUT, 1);

                    //differenz zwischen Startzeiten
                    dragrace.Zeiten_new[DR_RECHTS].Zeit_L3 =  (dragrace.Zeiten_new[mcpwm_unit].Zeit_Start-dragrace.Zeiten_new[  !mcpwm_unit].Zeit_Start)  ;
                    //message
                    dragrace_show("Gestartet GRUEN") ;

                }

                ESP_LOGI(DRAG, "Start Rechts %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal]
                                                                   , evt.capture_signal);


                //Fehlstart?
                if (dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_L1 == DURCHFAHREN) {
                    if (dragrace.Zeiten_new[mcpwm_unit].Zeit_L1 < dragrace.Zeiten_new[mcpwm_unit].Zeit_Start) {
                        dragrace.Status_new.bahn_status_new[mcpwm_unit].Status_Fruehstart = true;
                        //message
                        dragrace_show(mcpwm_capture_fehlstart_string[mcpwm_unit]) ;
                    }
                    ESP_LOGI(DRAG, "ReaktionsZeit%s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],
                             evt.capture_signal - dragrace.Zeiten.Rechts. Lichtschr1);

                }
            }



        }


        /// Old
#define OLD 0
#if OLD==1

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
                ESP_LOGI(DRAG, "Lichschr.    %s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],evt.capture_signal);
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
            if( dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false){
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
                ESP_LOGI(DRAG, "Start Rechts %s |             | %u", mcpwm_capture_signal_string[evt.sel_cap_signal],  evt.capture_signal);


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
                    ESP_LOGI(DRAG, "Frühstart    %s |             |", mcpwm_capture_signal_string[evt.sel_cap_signal]);
                }
                // Reaktionszeit
                if( dragrace.Status.LinkeBahn.Start == true) {
                    ESP_LOGI(DRAG, "Reak.Zeit    %s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal],10,evt.capture_signal-dragrace.Zeiten.Links.Start);
                }
            }
        }

        //! 2. Lichtschranke Links (Geschwindikeitsmessung)
        if (evt.sel_cap_signal == MCPWM_UNIT0_SELECT_CAP1) {
            if(1/*dragrace.Status.Laeuft == true && dragrace.Status.Gestartet==false*/){
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
                ESP_LOGI(DRAG, "Ziel Links   %s |  %*u |", mcpwm_capture_signal_string[evt.sel_cap_signal], 10,evt.capture_signal);
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
#endif

        if(!dragrace.Status_old.LinkeBahn.Start_Ausgewertet && dragrace.Status_old.LinkeBahn.Start && dragrace.Status_old.LinkeBahn.Lichschr1){
            dragrace.Status_old.LinkeBahn.Start_Ausgewertet=true;
        }

        //dragrace_show(message);
        //convert_to_json();
    }
}

/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP signal and according take action
 */

#define NEW_isr  0
#define NEW2_isr  1
#define OLD_isr  0

static void IRAM_ATTR isr_handler(void *u){
//static void IRAM_ATTR isr_handler(const int *unit){
    uint32_t mcpwm_unit0_intr_status;
    uint32_t mcpwm_unit1_intr_status;
    BaseType_t xHigherPriorityTaskWoken;
    capture evt;
    uint32_t status;
    uint32_t pins;
    int unit =*(int*)u; ///MCPWM_UNIT_0 oder MCPWM_UNIT_1 = Linke oder Rechte Bahn

    xHigherPriorityTaskWoken = pdFALSE;
    interupt_count++;

#if NEW_isr==1
    ///interrupt status
    uint32_t mcpwm_intr_status =  MCPWM[unit]->int_st.val;
#endif
#if OLD_isr
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

#endif
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

#if NEW2_isr==1
    /**Read interrupt status */
    while(MCPWM[MCPWM_UNIT_0]->int_st.val &(CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN) || MCPWM[MCPWM_UNIT_1]->int_st.val &(CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN)){

        /// Lichtschr. 3 Links
        if(MCPWM[MCPWM_UNIT_0]->int_st.cap2_int_st){
            evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP2].capn_value; //get capture signal counter value
            evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP2;                                    //get capture signal channel
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);                   //put in "Pipeline"
            MCPWM[MCPWM_UNIT_0]->int_clr.cap2_int_clr=1;                                     //clear interrupt
        }
        /// Lichtschr. 3 Rechts
        if(MCPWM[MCPWM_UNIT_1]->int_st.cap2_int_st){
            evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP2].capn_value;
            evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP2;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            MCPWM[MCPWM_UNIT_1]->int_clr.cap2_int_clr=1;
        }
        /// Lichtschr. 1 Links
        if(MCPWM[MCPWM_UNIT_0]->int_st.cap0_int_st){
            evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP0].capn_value;
            evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP0;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            MCPWM[MCPWM_UNIT_0]->int_clr.cap0_int_clr=1;
        }
        /// Lichtschr. 1 Rechts
        if(MCPWM[MCPWM_UNIT_1]->int_st.cap0_int_st){
            evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP0].capn_value;
            evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP0;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            MCPWM[MCPWM_UNIT_1]->int_clr.cap0_int_clr=1;
        }
        /// Lichtschr. 2 Links
        if(MCPWM[MCPWM_UNIT_0]->int_st.cap1_int_st){
            MCPWM[0]->int_ena.cap1_int_ena=0;       /// disable interrupt
            evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP1].capn_value;
            evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP1;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            MCPWM[MCPWM_UNIT_0]->int_clr.cap1_int_clr=1;
        }
        /// Lichtschr. 2 Rechts
        if(MCPWM[MCPWM_UNIT_1]->int_st.cap1_int_st){
            MCPWM[1]->int_ena.cap1_int_ena=0;       /// disable interrupt
            evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP1].capn_value;
            evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP1;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            MCPWM[MCPWM_UNIT_1]->int_clr.cap1_int_clr=1;
        }

    }
#endif


#if NEW_isr
    /*!
     * Neu Beide Bahnen (Bahn steht in "unit")
     * */
    uint32_t offset=unit*3;

    /// 3. Lichtschranke
    if (mcpwm_intr_status & CAP2_INT_EN) {
//        if(dragrace.Status_new.bahn_status_new[unit].Status_Start){
//            MCPWM[unit]->int_ena.cap2_int_ena=0;} ///disable interrupt
        evt.capture_signal = MCPWM[unit]->cap_chn[MCPWM_SELECT_CAP2].capn_value; //get capture signal counter value
        evt.sel_cap_signal = offset+MCPWM_SELECT_CAP2; //welche Lichtstranke (3 oder 6)
        xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
        MCPWM[unit]->int_clr.val = CAP2_INT_EN;
    }
    /// 1. Lichtschranke
    if (mcpwm_intr_status & CAP0_INT_EN) {
//        MCPWM[unit]->int_ena.cap0_int_ena=0; ///disable interrupt
        evt.capture_signal = MCPWM[unit]->cap_chn[MCPWM_SELECT_CAP0].capn_value; //get capture signal counter value
        evt.sel_cap_signal = offset+MCPWM_SELECT_CAP0; //welche Lichtstranke (1 oder 4)
        xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
        MCPWM[unit]->int_clr.val = CAP0_INT_EN;

    }
    /// 2. Lichtschranke
    if (mcpwm_intr_status & CAP1_INT_EN) {
//        MCPWM[unit]->int_ena.cap1_int_ena=0;
        evt.capture_signal = MCPWM[unit]->cap_chn[MCPWM_SELECT_CAP1].capn_value; //get capture signal counter value
        evt.sel_cap_signal = offset+MCPWM_SELECT_CAP1; //welche Lichtstranke (2 oder 5)
        xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
        MCPWM[unit]->int_clr.val = CAP1_INT_EN;

    }
    ///clear interrupt status
    //MCPWM[unit]->int_clr.val = mcpwm_intr_status;
#endif

#if OLD_isr
    /*!
     * Linke Bahn
     * */
    /**Check for interrupt on rising edge on CAP0 signal original*/


    if(unit==MCPWM_UNIT_0) {

        if (mcpwm_unit0_intr_status & CAP0_INT_EN) {
            evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP0].capn_value; //get capture signal counter value
//        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
            evt.sel_cap_signal = unit*3+MCPWM_SELECT_CAP0;
//            evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP0;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            ESP_EARLY_LOGD(TAG, "CAP0-L");
        }
        /**Check for interrupt on rising edge on CAP1 signal original*/
        if (mcpwm_unit0_intr_status & CAP1_INT_EN) {
            evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP1].capn_value; //get capture signal counter value
            evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP1;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            ESP_EARLY_LOGD(TAG, "CAP1-L");
        }
        /**Check for interrupt on rising edge on CAP2 signal original*/
        if (mcpwm_unit0_intr_status & CAP2_INT_EN) {
            evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_chn[MCPWM_SELECT_CAP2].capn_value; //get capture signal counter value
            evt.sel_cap_signal = MCPWM_UNIT0_SELECT_CAP2;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
//        MCPWM[MCPWM_UNIT_1]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;// Starte Rechts auch gleich

            ESP_EARLY_LOGD(TAG, "CAP2-L");
        }
    }

    /*!
     * Rechte Bahn
     * */
    /**Check for interrupt on rising edge on CAP0 signal original*/
    if(unit==MCPWM_UNIT_1) {

        if (mcpwm_unit1_intr_status & CAP0_INT_EN) {
            evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP0].capn_value; //get capture signal counter value
            evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP0;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            ESP_EARLY_LOGD(TAG, "CAP0-R");
        }
        /**Check for interrupt on rising edge on CAP1 signal original*/
        if (mcpwm_unit1_intr_status & CAP1_INT_EN) {
            evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP1].capn_value; //get capture signal counter value
            evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP1;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
            ESP_EARLY_LOGD(TAG, "CAP1-R");
        }
        /**Check for interrupt on rising edge on CAP2 signal original*/
        if (mcpwm_unit1_intr_status & CAP2_INT_EN) {
            evt.capture_signal = MCPWM[MCPWM_UNIT_1]->cap_chn[MCPWM_SELECT_CAP2].capn_value; //get capture signal counter value
            evt.sel_cap_signal = MCPWM_UNIT1_SELECT_CAP2;
            xQueueSendFromISR(cap_queue, &evt, &xHigherPriorityTaskWoken);
//        MCPWM[MCPWM_UNIT_0]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;// Starte Links auch gleich

            ESP_EARLY_LOGD(TAG, "CAP2-R");
        }
    }

    /**Clear Interuppt*/
    if(unit==MCPWM_UNIT_0){

    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_unit0_intr_status;
    }

    if(unit==MCPWM_UNIT_1){

    MCPWM[MCPWM_UNIT_1]->int_clr.val = mcpwm_unit1_intr_status;
    }
#endif
    if (xHigherPriorityTaskWoken) {
        //portYIELD_FROM_ISR (); /*Gehe direkt zur Verarbeitung*/
    }
    return xHigherPriorityTaskWoken == pdTRUE;

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
#if GPIO_CAP_POS_EDGE   //TRIGER EDGE defined in zeit.h
#define GPIO_CAP_EDGE MCPWM_POS_EDGE
#endif
#if GPIO_CAP_NEG_EDGE
#define GPIO_CAP_EDGE MCPWM_NEG_EDGE
#endif

#define PRESCALE 0L

    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, GPIO_CAP_EDGE, PRESCALE);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, GPIO_CAP_EDGE, PRESCALE);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, GPIO_CAP_EDGE, PRESCALE);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP1, GPIO_CAP_EDGE, PRESCALE);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, GPIO_CAP_EDGE, PRESCALE);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP2, GPIO_CAP_EDGE, PRESCALE);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    static int unit0 = MCPWM_UNIT_0;
    static int unit1 = MCPWM_UNIT_1;
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    MCPWM[MCPWM_UNIT_1]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal

    MCPWM[MCPWM_UNIT_0]->cap_timer_cfg.cap_synci_en=1;
    MCPWM[MCPWM_UNIT_1]->cap_timer_cfg.cap_synci_en=1;
    MCPWM[MCPWM_UNIT_0]->cap_timer_cfg.cap_sync_sw=1;
    MCPWM[MCPWM_UNIT_1]->cap_timer_cfg.cap_sync_sw=1;

    mcpwm_isr_register(MCPWM_UNIT_0,  isr_handler, &unit0, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
    mcpwm_isr_register(MCPWM_UNIT_1,  isr_handler, &unit1, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

    Serial_Start();

    //neues Rennen
    //action(neu);
    neu();

    vTaskDelete(NULL);
}

void Ausgaenge_Ansteuern_Eingaenge_Abfragen() {
    uint32_t tickCount = 0;
    int neuBtnPreviousState = LOW;
    int startBtnPreviousState = LOW;
    static dr_eingaenge_status *e = &dragrace.Status_new.Eingaenge;
//    static dr_eingaenge_status *e = &dragrace.Eingaenge;
    TickType_t blink;

    while (1) {
        tickCount++;
        vTaskDelay(10);

        blink = (xTaskGetTickCount()/25)&1L;


        ///EINGAENGE abfragen
        int bt_neu_level    = gpio_get_level(DRAGRACE_PIN_NEU_INPUT);
        int bt_start_level  = gpio_get_level(DRAGRACE_PIN_START_INPUT);
        e->Position_L       = gpio_get_level(DRAGRACE_PIN_POSITION_LINKS_INPUT);
        e->Position_R       = gpio_get_level(DRAGRACE_PIN_POSITION_RECHTS_INPUT);
        e->Lichtschranke_L1 = gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_L1_INPUT);
        e->Lichtschranke_L2 = gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_L2_INPUT);
        e->Lichtschranke_L3 = gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_L3_INPUT);
        e->Lichtschranke_R1 = gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_R1_INPUT);
        e->Lichtschranke_R2 = gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_R2_INPUT);
        e->Lichtschranke_R3 = gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_R3_INPUT);

        ///START
        gpio_set_level(DRAGRACE_PIN_ORANG1_LAMPE_OUTPUT, dragrace.Status_new.Orange1);
        gpio_set_level(DRAGRACE_PIN_ORANG2_LAMPE_OUTPUT, dragrace.Status_new.Orange2);
        gpio_set_level(DRAGRACE_PIN_GRUEN_LAMPE_OUTPUT, dragrace.Status_new.Laeuft_NEW);

        // bls und bsr sind variablen die den aktuellen Bahnstatus enthalten
        dr_bahn_status_new_t bsl= dragrace.Status_new.bahn_status_new[DR_LINKS];
        dr_bahn_status_new_t bsr= dragrace.Status_new.bahn_status_new[DR_RECHTS];

        ///Frühstart blinkt
        gpio_set_level(DRAGRACE_PIN_FEHLSTART_LINKS_OUTPUT,  bsl.Status_Fruehstart  && blink);
        gpio_set_level(DRAGRACE_PIN_FEHLSTART_RECHTS_OUTPUT, bsr.Status_Fruehstart  && blink);

        ///Sieg blinkt
        gpio_set_level(DRAGRACE_PIN_SIEG_LAMPE_R_OUTPUT, bsr.Status_Sieg && blink);
        gpio_set_level(DRAGRACE_PIN_SIEG_LAMPE_L_OUTPUT, bsl.Status_Sieg  && blink);

        ///Positions Lampe L
        gpio_set_level(DRAGRACE_PIN_POSITION_LAMPE_L_OUTPUT,
                //Brennt
           ( !e->Position_L && e->Lichtschranke_L1 && !dragrace.Status_new.Gestartet_NEW)
                //Blinkt
                || (!e->Lichtschranke_L1 && blink)  );

        ///Positions Lampe R
        gpio_set_level(DRAGRACE_PIN_POSITION_LAMPE_R_OUTPUT,
                //Brennt
              (!e->Position_R && e->Lichtschranke_R1 && !dragrace.Status_new.Gestartet_NEW)
                //Blinkt
                || (!e->Lichtschranke_R1 && blink ) );

        ///Zustand Lichtschranke vor Start ausgeben
        if (dragrace.Status_new.Ready_NEW && dragrace.debugg){
            printf("%*s,%*u\n",
                   16,"Lichschranken",
                   10,dragrace.Status_new.Eingaenge.all);

        }


#if HARDWARE_START_NEU_BTN_ENABLE == 1

        ///Neues Rennen Schalter
        int neuBtnCurrentState = bt_neu_level;
        if (neuBtnCurrentState != neuBtnPreviousState) {
            if (neuBtnCurrentState == HIGH && neuBtnPreviousState == LOW) {
                neu();
            }
            neuBtnPreviousState = neuBtnCurrentState;
        }

        ///Starte Rennen Schalter
        int startBtnCurrentState = bt_start_level;
        if (startBtnCurrentState != startBtnPreviousState) {
            if (startBtnCurrentState == HIGH && startBtnPreviousState == LOW) {
                drag_start();
            }
            startBtnPreviousState = startBtnCurrentState;
        }
#endif
    }

}

void neu(){
    ESP_LOGI(DRAG,"************** NEU **********************");
    ESP_LOGI(DRAG,"Ereignis        |  Linke Bahn | Rechte Bahn");
    ESP_LOGI(DRAG,"----------------|-------------|------------");

    //delete impulse_task
    if(impulse_task_handle != NULL && eTaskGetState(impulse_task_handle)!=eDeleted)  {
        vTaskDelete(impulse_task_handle);
    }


    //new
    dragrace.Status_new.Status_All=           0;
    dragrace.Status_new.bahn_status_new[0].bahn_status_all=0;
    dragrace.Status_new.bahn_status_new[1].bahn_status_all=0;
    dragrace.Status_new.race_status_all=0;
    dragrace.debugg=0;
    dragrace.Status_new.Ready_NEW=            1;
    dragrace.Zeiten_new[DR_LINKS]. Zeit_Start=0;
    dragrace.Zeiten_new[DR_LINKS]. Zeit_L1=   0;
    dragrace.Zeiten_new[DR_LINKS]. Zeit_L2=   0;
    dragrace.Zeiten_new[DR_LINKS]. Zeit_L3=   0;
    dragrace.Zeiten_new[DR_RECHTS].Zeit_Start=0;
    dragrace.Zeiten_new[DR_RECHTS].Zeit_L1=   0;
    dragrace.Zeiten_new[DR_RECHTS].Zeit_L2=   0;
    dragrace.Zeiten_new[DR_RECHTS].Zeit_L3=   0;
    uint32_t r_max =254; //Zufallszahl Maximum
    uint32_t r_min = 100;//Minimum
    uint32_t r = esp_random()%(r_max-r_min);
    r = r + r_min;
    ESP_LOGI(DRAG,"Zufälliger Start in %d.%d Sekunden\n",r/100,r%100);
    dragrace.randomstart= r;
    printf("--------------------------------------------------------------------------------------------------------------------\n");
    printf("%*s,%*s,%*s,%*s,%*s,%*s,%*s,%*s,%*s,%*s\n",
           16,"Meldung",
           10,"Status",
           10,"L Start",
           10,"L 1",
           10,"L 2",
           10,"L 3",
           10,"R Start",
           10,"R 1",
           10,"R 2",
           10,"R 3");
    printf("--------------------------------------------------------------------------------------------------------------------\n");

    char buf[50];
    sprintf(buf,"Neu in %d.%d Sek.",r/100+1,r%100);
    dragrace_show(buf);

    //old
//    dragrace.Status_old.all=0;
//    dragrace.Status_old.Ready=1;
//    dragrace.Zeiten.Time_Start=0;
//    dragrace.Zeiten.Links.Start=0;
//    dragrace.Zeiten.Links.Lichtschr1=0;
//    dragrace.Zeiten.Links.Lichtschr2=0;
//    dragrace.Zeiten.Links.Lichtschr3=0;
//    dragrace.Zeiten.Rechts.Start=0;
//    dragrace.Zeiten.Rechts.Lichtschr1=0;
//    dragrace.Zeiten.Rechts.Lichtschr2=0;
//    dragrace.Zeiten.Rechts.Lichtschr3=0;
//    uint32_t r_max =254; //Zufallszahl Maximum
//    uint32_t r_min = 100;//Minimum
//    uint32_t r = esp_random()%(r_max-r_min);
//    r = r + r_min;
//    ESP_LOGI(DRAG,"Zufälliger Start in %d.%d Sekunden\n",r/100,r%100);
//    dragrace.Zeiten.Time_Random= r;
//    dragrace_show("Neu");

}
void fertig(){
    dragrace.Status_new.Fertig_New=true;
    dragrace.Status_new.Laeuft_NEW=false;
    dragrace_show("Fertig");
//    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
//    {
//        ESP_LOGI(DRAG,"\n%s\n",dragrace.dragrace_Json_String);
//        xSemaphoreGive( xSemaphore );
//    }


}
void drag_start(){
    //new
    int flag=0;
    if(dragrace.Status_new.Ready_NEW){
        dragrace.debugg=0;
        MCPWM[MCPWM_UNIT_0]->cap_timer_cfg.cap_sync_sw=1;///Zähler auf 0
        MCPWM[MCPWM_UNIT_1]->cap_timer_cfg.cap_sync_sw=1;

        ///clear interrupt
        MCPWM[0]->int_clr.val = CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN;
        MCPWM[1]->int_clr.val = CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN;
        ///enable interrupt
        MCPWM[0]->int_ena.val |= CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN;
        MCPWM[1]->int_ena.val |= CAP0_INT_EN|CAP1_INT_EN|CAP2_INT_EN;

#if STARTSPERRE_BEI_LICHTSCHR_1
        if(!gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_L1_INPUT)){
            dragrace_show("L1 belegt");
            flag=1;
        }
        if(!gpio_get_level(DRAGRACE_PIN_LICHTSCHRANKE_R1_INPUT)){
            dragrace_show("R1 belegt");
            flag=1;
        }
        if(flag){return;}
#endif
        ///enable interrupt again / disabled in isr-handler routine
//        MCPWM[MCPWM_UNIT_0]->int_clr.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //clear interrupt
//        MCPWM[MCPWM_UNIT_1]->int_clr.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //clear interrupt
//        MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
//        MCPWM[MCPWM_UNIT_1]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal


        dragrace_impulse(NULL,dragrace.randomstart);
    }
    //old
    if(!dragrace.Status_old.Laeuft){
//        dragrace.Status_old.Gestartet=true;
//        dragrace.Status_old.Ready=false;
//        dragrace_impulse(NULL,dragrace.Zeiten.Time_Random);
        //vTaskDelay(dragrace.Zeiten.Time_Random);
        //MCPWM[MCPWM_UNIT_1]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;
        //MCPWM[MCPWM_UNIT_0]->cap_chn_cfg[MCPWM_SELECT_CAP2].capn_sw=1;
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
    xTaskCreate(Ausgaenge_Ansteuern_Eingaenge_Abfragen, "ein-aus", 4096, NULL, 2, NULL);
//     char buf[500];
//     vTaskList(buf);
//     ESP_LOGD(TAG,"*********************************");
//     ESP_LOGD(TAG,"Task         State Prio Stack Num");
//     ESP_LOGD(TAG,"*********************************");
//     ESP_LOGD(TAG,"%s",buf);

}

