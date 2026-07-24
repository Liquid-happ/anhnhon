#ifndef COMMON_TICK_H_
#define COMMON_TICK_H_

#include <stdint.h>

void Tick_Init(void);
uint32_t Tick_Get(void);
void Tick_Delay(uint32_t ms);

#endif /* COMMON_TICK_H_ */
