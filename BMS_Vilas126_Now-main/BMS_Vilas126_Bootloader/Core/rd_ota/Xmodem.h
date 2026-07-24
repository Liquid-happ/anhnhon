#ifndef _XMODEM_H_
#define _XMODEM_H_

#include "main.h"
#include <string.h>
#include <stdint.h>
#include "rd_control.h"





#define RIIM_type "RC188x"
//#define RIIM_code_ok "OKOKRC188x"
#define RIIM_code_ok "OKOKRC188x\r\n"
typedef enum Xmodem_state_t{
    RD_get_infor,
    RD_get_infor_wait,
    RD_imag,
    RD_imag_wait,
    RD_xmodem_send,
    RD_xmodem_send_wait,
    RD_end,
	RD_end_wait
}Xmodem_state_t;

extern Xmodem_state_t xmodem_state;

extern uint8_t X_STX;
extern uint8_t X_ACK;
extern uint8_t X_NAK;
extern uint8_t X_EOF[1];

typedef struct
{
    uint32_t received_size;
    uint32_t crc32;
} RD_Xmodem_Result_t;

#define BLOCK_SIZE FLASH_PAGE_SIZE //2kb

typedef struct data_flash_in_t{
    uint8_t start;
    uint8_t number_of_block;
    uint8_t check_number_of_block;
    uint8_t data[BLOCK_SIZE];
    uint8_t check_sum_block;
}data_flash_in_t;



void rd_run_recevied_ota(void);


#endif