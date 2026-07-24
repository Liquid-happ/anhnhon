#include "led_flash.h"

#if IS_BOOTLOADER == 0

#define MAX_LEDS 9

typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t      GPIO_Pin;   
    FlashMode_t   current_mode;   
    uint32_t      next_toggle_tick;
    uint32_t      on_duration_ms;
    uint32_t      off_duration_ms;    
    uint8_t       is_on;
    uint8_t       active;
} led_control_t;

static led_control_t leds[MAX_LEDS] = {0};

static uint8_t find_led_index(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    for (uint8_t i = 0; i < MAX_LEDS; i++) {
        if (leds[i].active && leds[i].GPIOx == GPIOx && leds[i].GPIO_Pin == GPIO_Pin) {
            return i;
        }
    }
    return 255;
}

static uint8_t get_free_slot(void)
{
    for (uint8_t i = 0; i < MAX_LEDS; i++) {
        if (!leds[i].active) {
            return i;
        }
    }
    return 255;
}

void LED_SetMode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, FlashMode_t mode)
{
    uint8_t index = find_led_index(GPIOx, GPIO_Pin);    
    if (index == 255) {
        index = get_free_slot();
        if (index == 255) return;  
        leds[index] = (led_control_t){0};  
        leds[index].GPIOx    = GPIOx;
        leds[index].GPIO_Pin = GPIO_Pin;
        leds[index].active   = 1;
    }

    if (leds[index].current_mode == mode) {
        return;
    }
		
    leds[index].current_mode = mode;
		
    if (mode == FLASH_MODE_OFF) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);  
        return;
    }
		
		if (mode == FLASH_MODE_ON) {  
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
        return;
    }
		
    switch (mode) {
        case FLASH_MODE_1:
            leds[index].on_duration_ms  = 250;
            leds[index].off_duration_ms = 3750;
            break;
        case FLASH_MODE_2:
            leds[index].on_duration_ms  = 500;
            leds[index].off_duration_ms = 500;
            break;
        case FLASH_MODE_3:
            leds[index].on_duration_ms  = 500;
            leds[index].off_duration_ms = 1500;
            break;
        default:
            return;
    }

    leds[index].is_on = 1;
    leds[index].next_toggle_tick = HAL_GetTick() + leds[index].on_duration_ms;
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); 
}

void LED_On(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void LED_Off(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void LED_Task_1ms(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < MAX_LEDS; i++) {
        if (!leds[i].active || leds[i].current_mode == FLASH_MODE_OFF || leds[i].current_mode == FLASH_MODE_ON) {
            continue;
        }
        if (now >= leds[i].next_toggle_tick) {
            if (leds[i].is_on) {
                HAL_GPIO_WritePin(leds[i].GPIOx, leds[i].GPIO_Pin, GPIO_PIN_SET);   
                leds[i].is_on = 0;
                leds[i].next_toggle_tick = now + leds[i].off_duration_ms;
            } 
						else 
						{
                HAL_GPIO_WritePin(leds[i].GPIOx, leds[i].GPIO_Pin, GPIO_PIN_RESET); 
                leds[i].is_on = 1;
                leds[i].next_toggle_tick = now + leds[i].on_duration_ms;
            }
        }
    }
}

#endif
