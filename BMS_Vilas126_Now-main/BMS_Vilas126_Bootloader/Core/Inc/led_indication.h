#ifndef LED_INDICATION_H
#define LED_INDICATION_H

#include "../rd_ota/rd_control.h"

#if IS_BOOTLOADER == 0

extern float SOC;

void Update_LED_Indication(void);   

#endif /* IS_BOOTLOADER == 0 */

#endif
