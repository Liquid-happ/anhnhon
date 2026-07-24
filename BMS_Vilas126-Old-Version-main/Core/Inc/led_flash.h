#ifndef LED_FLASH_H
#define LED_FLASH_H

#include "main.h"

typedef enum {
    FLASH_MODE_OFF = 0,
    FLASH_MODE_1,     
    FLASH_MODE_2,      
    FLASH_MODE_3,
		FLASH_MODE_ON
} FlashMode_t;

void LED_SetMode	(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, FlashMode_t mode);
void LED_On				(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LED_Off			(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void LED_Task_1ms	(void);   

#endif
