/* Active click feedback Example */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_spi_flash.h>
#include <driver/adc.h>
#include <driver/mcpwm.h>
#include <math.h>
const char* TAG = "main";

//#define USE_TIMER   //  Whther use the timer or not. Without this definition, the function is called from a normal task.

#ifdef USE_TIMER
# define DT 0.0001  //  In the case of the timer, the minimum period is 50 micro second.
#else
# define DT (1.0/configTICK_RATE_HZ)  
                    //  In the case of the task, the time period is the time slice of the OS specified in menuconfig,
                    //  which is set to 1 ms=1 kHz.  
#endif


struct WaveParam{
    const double damp[3] = {-10, -20, -30};
    const int nDamp = sizeof(damp) / sizeof(damp[0]);
    const double freq[4] = {100, 200, 300, 500};
    const int nFreq = sizeof(freq)/sizeof(freq[0]);
    const double amplitude = 2;
} wave;    //  
int count = 0;
double time = -1;

void hapticFunc(void* arg){
    const char* TAG = "H_FUNC";
    static int i;               //  An integer to select waveform. 
    static double omega = 0;    //  angular frequency
    static double B=0;          //  damping coefficient
    int ad = adc1_get_raw(ADC1_CHANNEL_6);
    if (ad < 2100 && time > 0.3){
        time = -1;
        printf("\r\n");
    }
    if (ad > 2400 && time == -1){   //  When the button is pushed after finishing to output an wave.
        //  set the time to 0 and update the waveform parameters.
        time = 0;
        omega = wave.freq[i % wave.nFreq] * M_PI * 2;
        B = wave.damp[i/wave.nFreq];
        printf("Wave: %3.1fHz, A=%2.2f, B=%3.1f ", omega/(M_PI*2), wave.amplitude, B);
        i++;
        if (i >= wave.nFreq * wave.nDamp) i = 0;
    }
    //  Output the wave
    double pwm = 0;
    if (time >= 0){
        pwm = wave.amplitude * cos (omega * time) * exp(B*time);
        time += DT;
    }else{
        pwm = 0;
    }
    //  Rotating direction
    if (pwm > 0){
        gpio_set_level(GPIO_NUM_5, 0);
        gpio_set_level(GPIO_NUM_17, 1);
#       ifndef USE_TIMER
        if (time >= 0) printf("+");
#       endif
    }else{
        gpio_set_level(GPIO_NUM_5, 1);
        gpio_set_level(GPIO_NUM_17, 0);
        pwm = -pwm;
#       ifndef USE_TIMER
        if (time >= 0) printf("-");
#       endif
    }
    if (pwm > 1) pwm = 1;
    
    //  Set duty rate of pwm
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm* 100);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    count ++;
    if (count >= 1000 ){
        ESP_LOGI(TAG, "ADC:%d", ad);
        count = 0;
    }
}

#ifndef USE_TIMER
void hapticTask(void* arg){
    while(1){
        hapticFunc(arg);
        vTaskDelay(1);
    }
}
#endif

extern "C" void app_main()
{        
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //----------------------------------
    printf("!!! Active Haptic Feedback Start !!!\n");
    
    ESP_LOGI("main", "Initialize ADC");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    ESP_LOGI("main", "Initialize PWM");
    //1. mcpwm gpio initialization  
    const int GPIO_PWM0A_OUT = 16;
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;          //duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A with above settings
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 20000);
    gpio_config_t conf;
    conf.pin_bit_mask = (1 << (17)) | (1 << (5));
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);

#ifdef USE_TIMER
    esp_timer_init();
    esp_timer_create_args_t timerDesc={
        callback: hapticFunc,
        arg: NULL,
        dispatch_method: ESP_TIMER_TASK,        
        name: "haptic"
    };
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&timerDesc, &timerHandle);
    esp_timer_start_periodic(timerHandle, (int)(1000*1000*DT));     // period in micro second (100uS=10kHz)
#else
    TaskHandle_t taskHandle = NULL;
    xTaskCreate(hapticTask, "Haptic", 1024 * 10, NULL, 6, &taskHandle);
#endif

}
