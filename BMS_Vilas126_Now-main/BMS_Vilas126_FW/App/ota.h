#ifndef APP_OTA_H_
#define APP_OTA_H_

#include <stdint.h>
#include "types.h"

#define TEMPBUFF_LENGTH     (64)
#define MESSAGE_MAXLENGTH   (58)
#define MESSAGE_HEADLENGTH  (6)
#define MESSAGE_OFFSET      (1)

#define OP_START_OTA        (0x5AA5)
#define OP_GET_INFOR        (0x0002)
#define OP_PING_STM32       (0x0003)

typedef struct IncomingData {
    uint8_t   Start[2];
    uint8_t   Length[2];
    uint8_t   Header[2];
    uint8_t   Message[MESSAGE_MAXLENGTH];
} TS_GWIF_IncomingData;

typedef void (*opcode_handler_t)(uint16_t opcode, uint8_t *data, uint16_t len);

void ota_init(void);
void ota_check_data(void);
void ota_process(void);
void ota_send_data_esp(uint8_t *data, uint16_t opcode, uint16_t length);

void handle_start_ota(uint16_t opcode, uint8_t *data, uint16_t len);
void handle_get_infor(uint16_t opcode, uint8_t *data, uint16_t len);
void handle_ping_stm32(uint16_t opcode, uint8_t *data, uint16_t len);

void ota_run_check_uart(void);

#endif /* APP_OTA_H_ */
