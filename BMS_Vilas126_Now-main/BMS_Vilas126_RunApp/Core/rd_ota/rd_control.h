#ifndef _RD_CONTROL_H_
#define _RD_CONTROL_H_

#include "main.h"
#include "rd_flash.h"
#include "RingBuffer.h"
#include "Xmodem.h"

#define RD_DEBUG 1
#if RD_DEBUG
    #define uart_debug_id huart4
    extern uint8_t temp_debug[256];
    extern UART_HandleTypeDef uart_debug_id;
#endif

#define IS_BOOTLOADER 0



#define TEMPBUFF_LENGTH		(64)
#define MESSAGE_MAXLENGTH	(58)
#define MESSAGE_HEADLENGTH	(6)
#define MESSAGE_OFFSET      (1)


typedef void (*opcode_handler_t)(uint16_t opcode, uint8_t *data, uint16_t len);

#define OPCODE_LIST \
    X(OP_START_OTA,        0x5AA5, handle_start_ota) \
    X(OP_GET_INFOR,        0x0002, handle_get_infor) \
		X(OP_PING_STM32,	 		 0x0003, handle_ping_stm32)
#define X(name, value, handler) name = value,
typedef enum {
    OPCODE_LIST
} opcode_t;
#undef X

void opcode_dispatch(uint16_t opcode, uint8_t *data, uint16_t len);




#define START_OTA	(0x5AA5)
#define GET_STT     (0x0001)

typedef struct IncomingData{
	uint8_t 	Start[2];
	uint8_t 	Length[2];
	uint8_t		Header[2];
	uint8_t		Message[MESSAGE_MAXLENGTH];
} TS_GWIF_IncomingData;


typedef struct Ota_data_t{
    uint8_t start_ota;
    uint8_t ok_start_ota;
    uint32_t version;
    uint32_t size;
    uint8_t crc[32];  //sha256 crc is 32 bytes
    uint32_t TimeoutCounter;
}Ota_data_t;

extern UART_HandleTypeDef huart1;
extern volatile Ota_data_t Ota_data ;
void rd_init_control(void);

unsigned char Checksum_tts(TS_GWIF_IncomingData *data_1,unsigned int length);
void RD_CheckData(void);
void RD_process(void);

void rd_send_data_esp(uint8_t *data,uint16_t Opcode, uint16_t length);

// opcode handler function prototypes
void handle_start_ota(uint16_t opcode, uint8_t *data, uint16_t len);
void handle_get_infor(uint16_t opcode, uint8_t *data, uint16_t len);
void handle_ping_stm32(uint16_t opcode, uint8_t *data, uint16_t len);





void rd_run_while_check_uart(void);


#endif