#include "ds1307.h"

#if IS_BOOTLOADER == 0

uint8_t B2D(uint8_t num)
{
    return ((num >> 4) * 10 + (num & 0x0F));
}

uint8_t D2B(uint8_t num)
{
    return ((num / 10) << 4 | (num % 10));
}

void DS1307_SetTime(DS1307_TIME *setTime)
{
    uint8_t temp[7] = {0};
    temp[0] = D2B(setTime->sec) & 0x7F;
    temp[1] = D2B(setTime->min);
    temp[2] = D2B(setTime->hour);
    temp[3] = D2B(setTime->day);   
    temp[4] = D2B(setTime->date);
    temp[5] = D2B(setTime->month);
    temp[6] = D2B(setTime->year);
    HAL_I2C_Mem_Write(&hi2c2, DS1307_ADDR, 0x00, 1, temp, 7, 100);
}

/* GET TIME */
void DS1307_GetTime(DS1307_TIME *getTime) 
{
    uint8_t temp[7] = {0};
		HAL_StatusTypeDef status;
		for (uint8_t retry = 0; retry < 3; retry++) {
				status = HAL_I2C_Mem_Read(&hi2c2, DS1307_ADDR, 0x00, 1, temp, 7, 100);
        if (status == HAL_OK) {
				getTime->sec   = B2D(temp[0] & 0x7F);  
				getTime->min   = B2D(temp[1]);
				getTime->hour  = B2D(temp[2]);
				getTime->day   = B2D(temp[3]);
				getTime->date  = B2D(temp[4]);
				getTime->month = B2D(temp[5]);
				getTime->year  = B2D(temp[6]);
				return;
        }
				I2C2_Bus_Recovery();
        Safe_Delay_ms(5);
    }
}

void DS1307_Enable_SQW_1Hz(void) {
    uint8_t control_reg = 0x10; 
    HAL_I2C_Mem_Write(&hi2c2, DS1307_ADDR, 0x07, 1, &control_reg, 1, 100);
}

#endif
