//use of adc in oneshot mode
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include <math.h>

//UMBRALES
#define ROOM_1_TEMPERATURE_UPPER 24
#define ROOM_1_TEMPERATURE_LOWER 20

#define ROOM_2_TEMPERATURE_UPPER 10
#define ROOM_2_TEMPERATURE_LOWER 0

#define ROOM_3_TEMPERATURE_UPPER 70
#define ROOM_3_TEMPERATURE_LOWER -10

#define ROOM_1_ILUMINATION 5
#define ROOM_2_ILUMINATION 400
#define ROOM_3_ILUMINATION 2500

#define CHANNEL_TEMPERATURE ADC_CHANNEL_0
#define CHANNEL_LIGHTNING ADC_CHANNEL_6

#define ROOM_1_SWITCH GPIO_NUM_13
#define ROOM_2_SWITCH GPIO_NUM_12
#define ROOM_3_SWITCH GPIO_NUM_14
#define TEMPERATURE_LED_PIN_1 GPIO_NUM_23
#define TEMPERATURE_LED_PIN_2 GPIO_NUM_22
#define ILUMINATION_LED_PIN_1 GPIO_NUM_19
#define ILUMINATION_LED_PIN_2 GPIO_NUM_18
#define ILUMINATION_LED_PIN_3 GPIO_NUM_2
#define ILUMINATION_LED_PIN_4 GPIO_NUM_15

typedef enum led_temperature {
    blue_led,
    red_led,
    off
}eLedTemperature_t;

typedef enum led_ilumination {
    off_i,    //0%
    level_1,//25%
    level_2,//50%
    level_3,//75%
    level_4//100%
}eLedIlumination_t;

const float BETA = 3950;
const float GAMMA = 0.7;
const float RL10 = 50;

adc_oneshot_unit_handle_t adc1_handle;

void adc_Init(void);
float read_adc_input(adc_channel_t channel);
void init_Gpio(void);
uint8_t read_room_switch(void);
void change_temp_led(float temperature,uint8_t room);
void turn_On_Led_Temperature(eLedTemperature_t led);
void change_ilum_led(float ilumination,uint8_t room);
uint8_t ilumination_level(float ilumination,uint16_t threshold);
void turn_On_Led_Ilumination(eLedIlumination_t led_ilum);

void adc_Init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = 
    {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = 
    {
    .bitwidth = ADC_BITWIDTH_12,
    .atten = ADC_ATTEN_DB_0,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CHANNEL_TEMPERATURE, &config)); //canal 0, pin 14
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CHANNEL_LIGHTNING, &config)); //canal 0, pin 13

}

float read_adc_input(adc_channel_t channel)
{
    int adc_raw;
    float voltage, celsius, result=0;
    //uint16_t adc_raw_uint16;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));

    voltage = (float)adc_raw * 5 / (float)4096;
    
    if(channel == CHANNEL_TEMPERATURE)
    {
        result = 1 / (log(1 / (4095.0 / adc_raw - 1)) / BETA + 1.0 / 298.15) - 273.15;
    }
    else if(channel == CHANNEL_LIGHTNING)
    {   
        float resistance = 2000 * voltage / (1 - voltage / 5);
        float result = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
        return result; //due to weird bug in wokwi
    }
    else
        result=voltage;

    return result;

}

void init_Gpio(void)
{
    //inputs and pull downs
    gpio_set_direction(ROOM_1_SWITCH, GPIO_MODE_INPUT);
    gpio_pulldown_en(ROOM_1_SWITCH);
    gpio_set_direction(ROOM_2_SWITCH, GPIO_MODE_INPUT);
    gpio_pulldown_en(ROOM_2_SWITCH);
    gpio_set_direction(TEMPERATURE_LED_PIN_1, GPIO_MODE_INPUT);
    gpio_pulldown_en(ROOM_3_SWITCH);
    
    //leds
    //gpio_set_direction(TEMPERATURE_LED_PIN_1, GPIO_MODE_OUTPUT);
    
}

uint8_t read_room_switch(void)
{
    uint8_t room;

    //takes pririty to switch room 1
    if(gpio_get_level(ROOM_1_SWITCH))
        room=1;
    else if(gpio_get_level(ROOM_2_SWITCH))
        room=2;
    else if(gpio_get_level(ROOM_3_SWITCH))
        room=3;
    else
        room=0;

    return room;
}

void change_temp_led(float temperature,uint8_t room)
{
    eLedTemperature_t led_tp;

    switch(room)
    {
        case 1:
            if(temperature>=ROOM_1_TEMPERATURE_UPPER)
                led_tp=blue_led;
            else if(temperature<=ROOM_1_TEMPERATURE_LOWER)
                led_tp=red_led;
            else 
                led_tp=off;
            break;
        case 2:
            if(temperature>=ROOM_2_TEMPERATURE_UPPER)
                led_tp=blue_led;
            else if(temperature<=ROOM_2_TEMPERATURE_LOWER)
                led_tp=red_led;
            else 
                led_tp=off;
            break;
        case 3:
            if(temperature>=ROOM_3_TEMPERATURE_UPPER)
                led_tp=blue_led;
            else if(temperature<=ROOM_3_TEMPERATURE_LOWER)
                led_tp=red_led;
            else 
                led_tp=off;
            break;
        default:
            led_tp=off;
    }

    turn_On_Led_Temperature(led_tp);
}

void turn_On_Led_Temperature(eLedTemperature_t led)
{
    switch(led)
    {
        case red_led:
            gpio_set_direction(TEMPERATURE_LED_PIN_1, GPIO_MODE_OUTPUT);
            gpio_set_level(TEMPERATURE_LED_PIN_1,1);
            gpio_set_direction(TEMPERATURE_LED_PIN_2, GPIO_MODE_OUTPUT);
            gpio_set_level(TEMPERATURE_LED_PIN_2,0);
            break;
        case blue_led:
            gpio_set_direction(TEMPERATURE_LED_PIN_1, GPIO_MODE_OUTPUT);
            gpio_set_level(TEMPERATURE_LED_PIN_1,0);
            gpio_set_direction(TEMPERATURE_LED_PIN_2, GPIO_MODE_OUTPUT);
            gpio_set_level(TEMPERATURE_LED_PIN_2,1);
            break;
        default:
            gpio_set_direction(TEMPERATURE_LED_PIN_1, GPIO_MODE_DISABLE);
            gpio_set_direction(TEMPERATURE_LED_PIN_2, GPIO_MODE_DISABLE);
    }
}

void change_ilum_led(float ilumination,uint8_t room)
{
    eLedIlumination_t led_ilum=off_i;

    switch(room)
    {
        case 1:
            led_ilum += ilumination_level(ilumination,ROOM_1_ILUMINATION);
            break;
        case 2:
            led_ilum += ilumination_level(ilumination,ROOM_2_ILUMINATION);
            break;
        case 3:
            led_ilum += ilumination_level(ilumination,ROOM_3_ILUMINATION);
            break;
    }

    turn_On_Led_Ilumination(led_ilum);
}

uint8_t ilumination_level(float ilumination,uint16_t threshold)
{
    float dif, percentage;
    uint8_t level_increm=0;

    dif = (float)threshold - ilumination;

    if(dif>0) //below threshold
    {
        percentage = (ilumination / threshold) * 100;

        if(percentage>0 && percentage<=25)
            level_increm = 4;
        else if(percentage>25 && percentage<=50)
            level_increm = 3;
        else if(percentage>50 && percentage<=75)
            level_increm = 2;
        else if(percentage>75)
            level_increm = 1;
    }
    else //past threshold, aceptable
        level_increm = 0;

    return level_increm;
}

void turn_On_Led_Ilumination(eLedIlumination_t led_ilum)
{
    switch(led_ilum)
    {
        case level_1:
            gpio_set_direction(ILUMINATION_LED_PIN_1, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_1,1);
            gpio_set_direction(ILUMINATION_LED_PIN_2, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_2,0);
            gpio_set_direction(ILUMINATION_LED_PIN_3, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_4, GPIO_MODE_DISABLE);
            break;
        case level_2:
            gpio_set_direction(ILUMINATION_LED_PIN_1, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_1,0);
            gpio_set_direction(ILUMINATION_LED_PIN_2, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_2,1);
            gpio_set_direction(ILUMINATION_LED_PIN_3, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_4, GPIO_MODE_DISABLE);
            break;
        case level_3:
            gpio_set_direction(ILUMINATION_LED_PIN_1, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_2, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_3, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_3,1);
            gpio_set_direction(ILUMINATION_LED_PIN_4, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_4,0);
            break;
        case level_4:
            gpio_set_direction(ILUMINATION_LED_PIN_1, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_2, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_3, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_3,0);
            gpio_set_direction(ILUMINATION_LED_PIN_4, GPIO_MODE_OUTPUT);
            gpio_set_level(ILUMINATION_LED_PIN_4,1);
            break;
        default:
            gpio_set_direction(ILUMINATION_LED_PIN_1, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_2, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_3, GPIO_MODE_DISABLE);
            gpio_set_direction(ILUMINATION_LED_PIN_4, GPIO_MODE_DISABLE);
    }
}

void app_main(void)
{
    float adc_input;
    uint8_t room;

    adc_Init();

    while(1)
    {
        printf("-----------------------\n");

        //read switch for current room
        room=read_room_switch();
        printf("Room: %u\n",room);
        
        //temperature
        adc_input = read_adc_input(CHANNEL_TEMPERATURE);
        printf("Temperature: %.3f C\n",adc_input);
        change_temp_led(adc_input,room);

        //ilumination
        adc_input = read_adc_input(CHANNEL_LIGHTNING);
        printf("Ilumnation: %.3f\n",adc_input);
        change_ilum_led(adc_input,room);

        vTaskDelay(1000 / portTICK_PERIOD_MS); //1 sec
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle)); //recycle adc unit
}