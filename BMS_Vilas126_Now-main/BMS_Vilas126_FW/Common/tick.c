#include "tick.h"
#include "stm32f1xx_hal.h"

void Tick_Init(void)
{
    // Uses standard HAL SysTick
}

uint32_t Tick_Get(void)
{
    return HAL_GetTick();
}

void Tick_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}
