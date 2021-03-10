#include <esp_types.h>


#include "include/zeit.h"
#include "include/main.h"
#include "esp_timer.h"

xQueueHandle cap_queue;
#if MCPWM_EN_CAPTURE
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#endif

uint32_t Zahl;


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
    uint32_t t;
    while (1) {


        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
//        t=esp_timer_get_time();

        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] /80);
//            current_cap_value[0] = (current_cap_value[0] / 1000) * (100000000000 / rtc_clk_apb_freq_get());
//            current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
//            printf("CAP0 : %d \265s %ld interupt: %d intr-raw %x intr_sta %x edge %u \n",
//                   current_cap_value[0],count++,interupt_count,evt.intr_raw_stat,evt.intr_stat,evt.edge);
//            t=esp_timer_get_time()-time;
//            if(!(count++ % 200) ){

//                xQueueReset(cap_queue);

                printf("%d\n",
                       current_cap_value[0]);
                Zahl=current_cap_value[0];
//            }
            //printf("%lld",esp_timer_get_time()-time);
        }
/*
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1) {
            current_cap_value[1] = evt.capture_signal - previous_cap_value[1];
            previous_cap_value[1] = evt.capture_signal;
            current_cap_value[1] = (current_cap_value[1] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP1 : %d us\n", current_cap_value[1]);
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP2) {
            current_cap_value[2] = evt.capture_signal -  previous_cap_value[2];
            previous_cap_value[2] = evt.capture_signal;
            current_cap_value[2] = (current_cap_value[2] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP2 : %d us\n", current_cap_value[2]);
        }
*/
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
    xHigherPriorityTaskWoken = pdFALSE;
    interupt_count++;
    capture evt;
    uint32_t t;


    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status Original
    Drag_mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
//    evt.intr_raw_stat = MCPWM[MCPWM_UNIT_0]->int_raw.val; //Read interrupt status
//    evt.intr_stat = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
//    evt.edge = MCPWM[MCPWM_UNIT_0]->cap_status.cap0_edge; //Read interrupt status

    if (MCPWM[MCPWM_UNIT_0]->int_st.cap0_int_st) { //Check for interrupt on rising edge on CAP0 signal
//    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal //original
        evt.capture_signal = MCPWM[MCPWM_UNIT_0]->cap_val_ch[MCPWM_SELECT_CAP0]; //get capture signal counter value
//        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
//        MCPWM[MCPWM_UNIT_0]->int_st.val=0; //a.l.

//        t=esp_timer_get_time();
//        time=esp_timer_get_time();
        xQueueSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken );
        MCPWM[MCPWM_UNIT_0]->cap_cfg_ch->sw=0;
//        xQueueGenericSendFromISR(cap_queue, &evt,&xHigherPriorityTaskWoken,queueOVERWRITE );
//        MCPWM[MCPWM_UNIT_0]->int_clr.cap0_int_clr = 1;

    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;  //Original
    if (xHigherPriorityTaskWoken) {
        // Actual macro used here is port specific.
        portYIELD_FROM_ISR ();
    }



    //    if (mcpwm_intr_status & CAP1_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
//        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
//        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
//        xQueueSendFromISR(cap_queue, &evt, NULL);
//    }
//    if (mcpwm_intr_status & CAP2_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
//        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP2); //get capture signal counter value
//        evt.sel_cap_signal = MCPWM_SELECT_CAP2;
//        xQueueSendFromISR(cap_queue, &evt, NULL);
//    }
//    MCPWM[MCPWM_UNIT_0]->int_clr.val = 0;
//    MCPWM[MCPWM_UNIT_0]->int_clr.cap0_int_clr = 1;
}
#endif

/**
 * @brief Configure whole MCPWM module
 */
static void mcpwm_example_config(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initialize mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;

//    pwm_config.frequency = 1000;    //frequency = 1000Hz
//    pwm_config.cmpr_a = 60.0;       //duty cycle of PWMxA = 60.0%
//    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
//    pwm_config.counter_mode = MCPWM_UP_COUNTER;
//    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
//    pwm_config.frequency = 1000;     //frequency = 500Hz
//    pwm_config.cmpr_a = 50;       //duty cycle of PWMxA = 45.9%
//    pwm_config.cmpr_b = 50;    //duty cycle of PWMxb = 07.0%
//    pwm_config.counter_mode = MCPWM_UP_COUNTER;
//    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
//    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
//    pwm_config.frequency = 400;     //frequency = 400Hz
//    pwm_config.cmpr_a = 23.2;       //duty cycle of PWMxA = 23.2%
//    pwm_config.cmpr_b = 97.0;       //duty cycle of PWMxb = 97.0%
//    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; //frequency is half when up down count mode is set i.e. SYMMETRIC PWM
//    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
//    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM2A & PWM2B with above settings

#if MCPWM_EN_CARRIER
    //3. carrier configuration
    //comment if you don't want to use carrier mode
    //in carrier mode very high frequency carrier signal is generated at mcpwm high level signal
    mcpwm_carrier_config_t chop_config;
    chop_config.carrier_period = 6;         //carrier period = (6 + 1)*800ns
    chop_config.carrier_duty = 3;           //carrier duty = (3)*12.5%
    chop_config.carrier_os_mode = MCPWM_ONESHOT_MODE_EN; //If one shot mode is enabled then set pulse width, if disabled no need to set pulse width
    chop_config.pulse_width_in_os = 3;      //first pulse width = (3 + 1)*carrier_period
    chop_config.carrier_ivt_mode = MCPWM_CARRIER_OUT_IVT_EN; //output signal inversion enable
    mcpwm_carrier_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &chop_config);  //Enable carrier on PWM2A and PWM2B with above settings
    //use mcpwm_carrier_disable function to disable carrier on mcpwm timer on which it was enabled
#endif

#if MCPWM_EN_DEADTIME
    //4. deadtime configuration
    //comment if you don't want to use deadtime submodule
    //add rising edge delay or falling edge delay. There are 8 different types, each explained in mcpwm_deadtime_type_t in mcpwm.h
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 1000, 1000);   //Enable deadtime on PWM2A and PWM2B with red = (1000)*100ns on PWM2A
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_RED, 300, 2000);        //Enable deadtime on PWM1A and PWM1B with fed = (2000)*100ns on PWM1B
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_RED_FED_FROM_PWMXA, 656, 67);  //Enable deadtime on PWM0A and PWM0B with red = (656)*100ns & fed = (67)*100ns on PWM0A and PWM0B generated from PWM0A
    //use mcpwm_deadtime_disable function to disable deadtime on mcpwm timer on which it was enabled
#endif

#if MCPWM_EN_FAULT
    //5. enable fault condition
    //comment if you don't want to use fault submodule, also u can comment the fault gpio signals
    //whenever fault occurs you can configure mcpwm signal to either force low, force high or toggle.
    //in cycmode, as soon as fault condition is over, the mcpwm signal is resumed, whereas in oneshot mode you need to reset.
    mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0); //Enable FAULT, when high level occurs on FAULT0 signal
    mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F1); //Enable FAULT, when high level occurs on FAULT1 signal
    mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F2); //Enable FAULT, when high level occurs on FAULT2 signal
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_F0, MCPWM_FORCE_MCPWMXA_HIGH, MCPWM_FORCE_MCPWMXB_LOW); //Action taken on PWM1A and PWM1B, when FAULT0 occurs
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_F1, MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_HIGH); //Action taken on PWM1A and PWM1B, when FAULT1 occurs
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F2, MCPWM_FORCE_MCPWMXA_HIGH, MCPWM_FORCE_MCPWMXB_LOW); //Action taken on PWM0A and PWM0B, when FAULT2 occurs
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F1, MCPWM_FORCE_MCPWMXA_LOW, MCPWM_FORCE_MCPWMXB_HIGH); //Action taken on PWM0A and PWM0B, when FAULT1 occurs
#endif

#if MCPWM_EN_SYNC
    //6. Syncronization configuration
    //comment if you don't want to use sync submodule, also u can comment the sync gpio signals
    //here synchronization occurs on PWM1A and PWM1B
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 200);    //Load counter value with 20% of period counter of mcpwm timer 1 when sync 0 occurs
#endif

#if MCPWM_EN_CAPTURE
    //7. Capture configuration
    //comment if you don't want to use capture submodule, also u can comment the capture gpio signals
    //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
    //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
//    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    //MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN ;  //Enable interrupt on  CAP0 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
#endif
    vTaskDelete(NULL);
}
void start(){
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch->sw=1;
}
 void mcpwm_setup()
{
    printf("Testing MCPWM...\n ");
    cap_queue = xQueueCreate(50, sizeof(capture)); //comment if you don't want to use capture module
    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module
//    xTaskCreate(gpio_test_signal, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module
    xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
}

