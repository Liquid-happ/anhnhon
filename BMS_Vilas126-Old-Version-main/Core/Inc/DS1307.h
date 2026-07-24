#ifndef __DS1307_H
#define __DS1307_H

#include "stm32f1xx_hal.h"

#define DS1307_ADDR   (0x68 << 1)

typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} DS1307_TIME;

extern I2C_HandleTypeDef hi2c2;
extern void I2C2_Bus_Recovery(void);
extern void Safe_Delay_ms(uint32_t ms);
uint8_t B2D(uint8_t num);
uint8_t D2B(uint8_t num);
void DS1307_SetTime(DS1307_TIME *setTime);
void DS1307_GetTime(DS1307_TIME *getTime);
void DS1307_Enable_SQW_1Hz(void);

#endif
