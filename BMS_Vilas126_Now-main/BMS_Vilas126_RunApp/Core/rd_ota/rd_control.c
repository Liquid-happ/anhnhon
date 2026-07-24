#include "rd_control.h"

#include <stdio.h>

 TS_GWIF_IncomingData	*vrts_GWIF_IncomeMessage;

 static unsigned char		vrsc_GWIF_TempBuffer[TEMPBUFF_LENGTH] = {0};
 static uint16_t			vrui_GWIF_LengthMeassge;
 static unsigned int		vrui_GWIF_header;


 static _Bool			    vrb_GWIF_RestartMessage = true;
 static _Bool			    vrb_GWIF_UpdateLate = false;
 static _Bool				vrb_GWIF_CheckNow = false;
 static _Bool 				messageUpdate = false;


volatile Ota_data_t Ota_data = {0};
#if RD_DEBUG
uint8_t temp_debug[256];
#endif

typedef struct {
    uint16_t code;
    const char *name;
    opcode_handler_t handler;
} opcode_entry_t;

#define X(name, value, handler) { value, #name, handler },
static const opcode_entry_t opcode_table[] = {
    OPCODE_LIST
};
#undef X
#define OPCODE_COUNT (sizeof(opcode_table)/sizeof(opcode_table[0]))

void opcode_dispatch(uint16_t opcode, uint8_t *data, uint16_t len)
{
    for (int i = 0; i < OPCODE_COUNT; i++)
    {
        if (opcode_table[i].code == opcode)
        {
            opcode_table[i].handler(opcode,data, len);
            return;
        }
    }
}

uint8_t is_valid_opcode(uint16_t opcode)
{
    for (int i = 0; i < OPCODE_COUNT; i++)
    {
        if (opcode_table[i].code == opcode)
            return 1;
    }
    return 0;
}




// uint8_t RD_test[19] ={0x55, 0xaa, 0x00, 12, 0x5a, 0xa5,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x01};
unsigned char Checksum_tts(TS_GWIF_IncomingData *data_1,unsigned int length){
	int i;
	unsigned char result = (data_1->Header[0] + data_1->Header[1 ]+ data_1->Length[0] + data_1->Length[1]);
	for (i = 0; i < length ; i++){
			result = (result + data_1->Message[i]);
	}
	result = result & 0xff;
	return result;
}

unsigned char Checksum_send(uint8_t *data_1,unsigned int length){
	int i;
	unsigned char result = 0;
	for (i = 2; i < length ; i++){
			result = (result + data_1[i]);
	}
	result = result & 0xff;
	return result;
}



void rd_init_control(void){
    ring_init(&vrts_ringbuffer_Data, RINGBUFFER_LEN, sizeof(uint8_t));
    vrts_GWIF_IncomeMessage = (TS_GWIF_IncomingData *)vrsc_GWIF_TempBuffer;
    Ota_data.start_ota = 0;
	rd_send_data_esp((uint8_t *)&ota_info.app_a_version, OP_GET_INFOR, 4); // send app version to ESP at startup
	// RD_test[18] = Checksum_send(RD_test, 18);
		// HAL_UART_Transmit(&huart1, RD_test, 19, 100);
}



void RD_CheckData(void){
	int vrui_Count =0, Count = 0;
	if(vrts_ringbuffer_Data.count >=1){
		// HAL_UART_Transmit(&huart2, (uint8_t *)"Checking data\r\n", 15, 100);
		#if RD_DEBUG
		if(vrts_ringbuffer_Data.count > 1 ){
//			sprintf((char*)temp_debug, "Data in ringbuffer: %d\r\n", vrts_ringbuffer_Data.count);
//			HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
		}
		#endif
		if(vrb_GWIF_UpdateLate == false){
			if(vrb_GWIF_RestartMessage == true){
				if(vrts_ringbuffer_Data.count >= 6){
					ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[0]);
					ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[1]);
					ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[2]);
					ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[3]);
					ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[4]);
					ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[5]);
					vrui_GWIF_LengthMeassge = (vrts_GWIF_IncomeMessage->Length[0]<<8)|(vrts_GWIF_IncomeMessage->Length[1]);
					vrui_GWIF_header = (vrts_GWIF_IncomeMessage->Header[0]<<8)|(vrts_GWIF_IncomeMessage->Header[1]);
				
					vrb_GWIF_RestartMessage = false;
					messageUpdate = true;
				}
			}
			else{
				ring_pop_tail(&vrts_ringbuffer_Data,   (void*)&vrsc_GWIF_TempBuffer[MESSAGE_HEADLENGTH - 1]);
				vrui_GWIF_LengthMeassge = (vrts_GWIF_IncomeMessage->Length[1]) | (vrts_GWIF_IncomeMessage->Length[0]<<8);
				vrui_GWIF_header = (vrts_GWIF_IncomeMessage->Header[0]<<8)|(vrts_GWIF_IncomeMessage->Header[1]);
					
				messageUpdate = true;
			}
			if(messageUpdate == true){
				messageUpdate = false;
				if((vrts_GWIF_IncomeMessage->Start[0] == 0x55) && (vrts_GWIF_IncomeMessage->Start[1] == 0xaa)){
					if( is_valid_opcode(vrui_GWIF_header)){
							if(vrts_ringbuffer_Data.count >= vrui_GWIF_LengthMeassge + 1){
								for(Count = 0; Count < vrui_GWIF_LengthMeassge + 1; Count ++){
									ring_pop_tail(&vrts_ringbuffer_Data,(void*)&vrsc_GWIF_TempBuffer[MESSAGE_HEADLENGTH+Count]);
								}
	    						vrb_GWIF_UpdateLate = false;
	    						vrb_GWIF_CheckNow = true;
	    						vrb_GWIF_RestartMessage = true;
							}
							else{
	    						vrb_GWIF_UpdateLate = true;
	                vrb_GWIF_RestartMessage = false;
	                vrb_GWIF_CheckNow = false;
							}
					}
					else{
    					vrsc_GWIF_TempBuffer[0] = vrsc_GWIF_TempBuffer[1];
    					vrsc_GWIF_TempBuffer[1] = vrsc_GWIF_TempBuffer[2];
    					vrsc_GWIF_TempBuffer[2] = vrsc_GWIF_TempBuffer[3];
    					vrsc_GWIF_TempBuffer[3] = vrsc_GWIF_TempBuffer[4];
    					vrsc_GWIF_TempBuffer[4] = vrsc_GWIF_TempBuffer[5];
    					vrb_GWIF_RestartMessage = false;
              vrb_GWIF_UpdateLate = false;
					}
				}
				else{
					vrsc_GWIF_TempBuffer[0] = vrsc_GWIF_TempBuffer[1];
					vrsc_GWIF_TempBuffer[1] = vrsc_GWIF_TempBuffer[2];
					vrsc_GWIF_TempBuffer[2] = vrsc_GWIF_TempBuffer[3];
					vrsc_GWIF_TempBuffer[3] = vrsc_GWIF_TempBuffer[4];
					vrsc_GWIF_TempBuffer[4] = vrsc_GWIF_TempBuffer[5];
					vrb_GWIF_RestartMessage = false;
					vrb_GWIF_UpdateLate = false;
				}
			}
		}
		else{
			if(vrts_ringbuffer_Data.count >= (vrui_GWIF_LengthMeassge + MESSAGE_OFFSET)){
				for( vrui_Count = 0; vrui_Count < (vrui_GWIF_LengthMeassge + MESSAGE_OFFSET); vrui_Count++){
					ring_pop_tail(&vrts_ringbuffer_Data, (void*)&vrsc_GWIF_TempBuffer[MESSAGE_HEADLENGTH + vrui_Count]);
				}
				vrb_GWIF_UpdateLate = false;
				vrb_GWIF_CheckNow = true;
				vrb_GWIF_RestartMessage = true;
			}
		}
	}
    RD_process();
}

void RD_process(void){
	
	if(vrb_GWIF_CheckNow)
	{
		
		vrb_GWIF_CheckNow = false;
		#if RD_DEBUG
		sprintf((char*)temp_debug, "Header: 0x%04X, Length: %d\r\n", vrui_GWIF_header, vrui_GWIF_LengthMeassge);
		HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
		sprintf((char*)temp_debug, "CRC Received: 0x%02X, CRC Calculated: 0x%02X\r\n", vrts_GWIF_IncomeMessage->Message[vrui_GWIF_LengthMeassge], Checksum_tts(vrts_GWIF_IncomeMessage,vrui_GWIF_LengthMeassge));
		HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
		#endif
        if(vrts_GWIF_IncomeMessage->Message[vrui_GWIF_LengthMeassge] == Checksum_tts(vrts_GWIF_IncomeMessage,vrui_GWIF_LengthMeassge)){
					
					opcode_dispatch(vrui_GWIF_header, vrts_GWIF_IncomeMessage->Message, vrui_GWIF_LengthMeassge);
          
        }
	}
}

// luong logic uart -> start ota -> phan hoi ->ok -> chay luong xmodem de nhan firmware

//void rd_send_data_esp(uint8_t *data,uint16_t Opcode, uint16_t length){
//    uint8_t send_buffer[256] = {0};
//    send_buffer[0] = 0x55; // Header byte 1
//    send_buffer[1] = 0xAA; // Header byte 2
//    send_buffer[2] = (length >> 8) & 0xFF; // Length high byte
//    send_buffer[3] = length & 0xFF; // Length low byte
//    send_buffer[4] = (Opcode >> 8) & 0xFF; // Opcode high byte
//    send_buffer[5] = Opcode & 0xFF; // Opcode low byte
//    memcpy(&send_buffer[6], data, length); // Copy message data
//    uint8_t checksum = Checksum_tts((TS_GWIF_IncomingData *)send_buffer, length + MESSAGE_HEADLENGTH);
//    send_buffer[MESSAGE_HEADLENGTH + length] = checksum; // Append checksum
//		HAL_UART_Transmit(&huart1, send_buffer, (MESSAGE_HEADLENGTH + length + 1), 1000);
//}

// opcode handler function prototypes



void rd_send_data_esp(uint8_t *data, uint16_t Opcode, uint16_t length)
{
    uint8_t send_buffer[256] = {0};

    send_buffer[0] = 0x55;
    send_buffer[1] = 0xAA;
    send_buffer[2] = length >> 8;
    send_buffer[3] = length;
    send_buffer[4] = Opcode >> 8;
    send_buffer[5] = Opcode;

    if ((data != NULL) && (length > 0))
    {
        memcpy(&send_buffer[6], data, length);
    }

    uint8_t checksum = Checksum_tts((TS_GWIF_IncomingData *)send_buffer, length + MESSAGE_HEADLENGTH);
		
    send_buffer[MESSAGE_HEADLENGTH + length] = checksum;
		
    HAL_UART_Transmit(&huart1,send_buffer, MESSAGE_HEADLENGTH + length + 1,1000);
}

// opcode handler function prototypes
void handle_start_ota(uint16_t opcode, uint8_t *data, uint16_t len){
	Ota_data.version = (vrts_GWIF_IncomeMessage->Message[3]<<24) | (vrts_GWIF_IncomeMessage->Message[2]<<16) | (vrts_GWIF_IncomeMessage->Message[1]<<8) | vrts_GWIF_IncomeMessage->Message[0];
	Ota_data.size = (vrts_GWIF_IncomeMessage->Message[7]<<24) | (vrts_GWIF_IncomeMessage->Message[6]<<16) | (vrts_GWIF_IncomeMessage->Message[5]<<8) | vrts_GWIF_IncomeMessage->Message[4];
	//Ota_data.crc = (vrts_GWIF_IncomeMessage->Message[11]<<24) | (vrts_GWIF_IncomeMessage->Message[10]<<16) | (vrts_GWIF_IncomeMessage->Message[9]<<8) | vrts_GWIF_IncomeMessage->Message[8];
	for (uint8_t i = 0; i < 32; i++)
	{
			Ota_data.crc[i] = vrts_GWIF_IncomeMessage->Message[8 + i];
	}
	#if RD_DEBUG
	for(int k = 0; k < 32; k++)
	{
		sprintf((char *)&temp_debug[k*3], "%02X ", Ota_data.crc[k]);
	}
	HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
	sprintf((char*)temp_debug, "START_OTA received: version=%d, size=%d\r\n", Ota_data.version, Ota_data.size);
	HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
	HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"START_OTA received\r\n", 20, 100);
	#endif
	Ota_data.start_ota = 1;
	Ota_data.ok_start_ota = 1;
	// rd_send_data_esp(NULL, OP_START_OTA, 0); // Send acknowledgment with no data ???
	Ota_data.TimeoutCounter = HAL_GetTick();
	
}



void handle_get_infor(uint16_t opcode, uint8_t *data, uint16_t len){
	#if RD_DEBUG
		HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"handle_get_infor\r\n", 18, 100);
	#endif
	uint8_t response_version[4] = {0};
	rd_send_data_esp((uint8_t *)&ota_info.app_a_version, OP_GET_INFOR, 4);
}

void handle_ping_stm32(uint16_t opcode, uint8_t *data, uint16_t len){
	#if RD_DEBUG
		HAL_UART_Transmit(&uart_debug_id, (uint8_t *)"handle_ping_stm32\r\n", 19, 100);
	#endif
	rd_send_data_esp(NULL,OP_PING_STM32,0);
}

void rd_run_while_check_uart(void){
		HAL_IWDG_Refresh(&hiwdg);
		if(Ota_data.start_ota){
			rd_run_recevied_ota();
		}
		else{
			if(Ota_data.ok_start_ota != 0){
				 #if RD_DEBUG
                sprintf((char*)temp_debug, "stop start OTA\r\n");
                HAL_UART_Transmit(&uart_debug_id, temp_debug, strlen((char*)temp_debug), 100);
                #endif
				Ota_data.ok_start_ota = 0;
			}
			RD_CheckData();
		}
}