#ifndef SVC_IND_H_
#define SVC_IND_H_

#include "feat_cfg.h"

#if IS_BOOTLOADER == 0

extern float SOC;

void Update_LED_Indication(void);   

#endif /* IS_BOOTLOADER == 0 */

#endif /* SVC_IND_H_ */
