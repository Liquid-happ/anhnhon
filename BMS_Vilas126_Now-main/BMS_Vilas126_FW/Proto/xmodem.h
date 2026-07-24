#ifndef PROTO_XMODEM_H_
#define PROTO_XMODEM_H_

#include <stdint.h>
#include <string.h>
#include "types.h"

#define RIIM_type "RC188x"
#define RIIM_code_ok "OKOKRC188x\r\n"

#define BLOCK_SIZE FLASH_PAGE_SIZE // 2KB

typedef enum Xmodem_state_t {
    RD_get_infor,
    RD_get_infor_wait,
    RD_imag,
    RD_imag_wait,
    RD_xmodem_send,
    RD_xmodem_send_wait,
    RD_end,
    RD_end_wait
} Xmodem_state_t;

typedef struct {
    uint32_t received_size;
    uint32_t crc32;
} RD_Xmodem_Result_t;

typedef struct data_flash_in_t {
    uint8_t start;
    uint8_t number_of_block;
    uint8_t check_number_of_block;
    uint8_t data[BLOCK_SIZE];
    uint8_t check_sum_block;
} data_flash_in_t;

extern Xmodem_state_t xmodem_state;

extern uint8_t X_STX;
extern uint8_t X_ACK;
extern uint8_t X_NAK;
extern uint8_t X_EOF[1];

void ota_run_received_ota(void);
void xmodem_send_data(uint8_t *data, uint16_t length);

#endif /* PROTO_XMODEM_H_ */
