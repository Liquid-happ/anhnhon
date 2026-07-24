#ifndef COMMON_CTX_H_
#define COMMON_CTX_H_

#include <stdint.h>
#include "types.h"

// System Context structure to hold global states
typedef struct {
    uint32_t uptime_ms;
    uint8_t state;
    uint8_t error_code;
    Bms_Meas_t meas;
    Bms_Fault_t fault;
} Bms_Ctx_t;

extern Bms_Ctx_t g_bms;

void Ctx_Init(void);
void Ctx_Update(void);

#endif /* COMMON_CTX_H_ */
