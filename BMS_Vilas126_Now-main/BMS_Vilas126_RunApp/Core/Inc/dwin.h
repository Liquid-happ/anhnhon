#ifndef __DWIN_H__
#define __DWIN_H__

#include "../rd_ota/rd_control.h"

#if IS_BOOTLOADER == 0

#include "main.h"
#include "bms_state.h"
#include "DS1307.h"
#include "stdint.h"
#include "bq_bms_485.h"

extern UART_HandleTypeDef huart3;
extern BMS_Alarms_t bms_alarms;
extern uint8_t ProtectionsTriggered;
extern uint8_t PFErrorsTriggered;
extern uint8_t UV_Fault;
extern uint8_t OV_Fault;
extern uint8_t OCC_Fault;
extern uint8_t OCD_Fault1;
extern uint8_t OCD_Fault;    
extern uint8_t SCD_Fault; 
extern uint8_t UTC_Fault; 
extern uint8_t UTD_Fault;  
extern uint8_t UTINT_Fault; 
extern uint8_t OTC_Fault;
extern uint8_t OTD_Fault;
extern uint8_t OTINT_Fault;
extern uint8_t OTF_Fault; 
extern uint8_t HWDF_Fault;
extern uint8_t PTO_Fault; 
extern uint8_t COVL_Fault;
extern uint8_t OCDL_Fault; 
extern uint8_t SCDL_Fault; 
extern uint8_t OCD3_Fault; 

void DWIN_WriteWord								(uint16_t VP, uint16_t value); 																						
void DWIN_WriteFloat							(uint16_t VP, float value, uint8_t decimal_places); 											
void DWIN_B_WriteWord							(uint16_t VP, int16_t value); 																							
void DWIN_B_WriteFloat						(uint16_t VP, float value, uint8_t decimal_places); 												
void DWIN_C_WriteByte							(uint16_t VP, uint8_t value);
void DWIN_Write2Word							(uint16_t VP, uint16_t value_h, uint16_t value_l);
void DWIN_TP_Simulate							(uint16_t mode, uint16_t x, uint16_t y);
void DWIN_SendCellVoltages				(float *cell_voltages); 																								
void DWIN_SendPackVoltage					(float pack_voltage); 																										
void DWIN_SendTemperature					(float temperature); 																											
void DWIN_SendTemperature1				(float temperature1); 																											
void DWIN_SendTemperature2				(float temperature2); 																									
void DWIN_SendTemperature3				(float temperature3); 																										
void DWIN_SendTemperature4				(float temperature4); 																										
void DWIN_SendTemperature5				(float temperature5); 																											
void DWIN_SendTemperature6				(float temperature6); 																									
void DWIN_SendTemperatureFET			(float temperatureFET); 																										
void DWIN_SendTemperatureIC				(float temperatureIC); 																								
void DWIN_SendCurrent							(float currentA);  																											
void DWIN_SendSOC									(float soc); 																																
void DWIN_SetSOCIcon							(float soc);																															
void DWIN_BSetSOCIcon							(float soc); 																																
void DWIN_SendReCapa							(float recapa); 																														
void DWIN_SendCycle								(float cycle); 																														
void DWIN_SendTime								(DS1307_TIME *t); 																												
void DWIN_SendSOH									(float soh); 																															
void DWIN_SendRemainEnergy 				(float remainenergy); 																										
void DWIN_UpdateAnyAlarm					(void); 																																	
void DWIN_UpdateAlarms						(void); 																																	
void DWIN_UpdateAnyProtection			(void); 																																	
void DWIN_UpdateProtections				(void); 																																
void DWIN_BacklightOff						(void);
void DWIN_BacklightOn							(uint8_t brightness);
void DWIN_TouchDisable						(void);
void DWIN_TouchEnable							(void);
void DWIN_TP_Click								(uint16_t x, uint16_t y);
void DWIN_ScreenOff								(void);
void DWIN_ScreenOn								(void);

void DWIN_SendSlavePackVoltage		(void);
void DWIN_SendSlaveCurrent				(void);
void DWIN_SendSlaveSOC						(void);              
void DWIN_SendSlaveSOH						(void);             
void DWIN_SendSlaveRemainCapacity	(void);   
void DWIN_SendSlaveCycleCount			(void);
void DWIN_SetSlaveSOCIcon					(void);   
void DWIN_BSetSlaveSOCIcon				(void);
void DWIN_SendSlaveTemperature0		(void);
void DWIN_SendSlaveCellVoltages		(void);
void DWIN_SendSlaveRemainEnergy		(void);
void DWIN_SendSystemTotalVoltage	(void);     
void DWIN_SendSystemAvgSOH				(void);           
void DWIN_SendSystemAvgSOC				(void);           
void DWIN_SendSystemAvgCycles			(void);        
void DWIN_SendSystemTotalCurrent	(void);   
void DWIN_SetSystemAvgSOCIcon			(void);
void DWIN_BSetSystemAvgSOCIcon		(void);
void DWIN_SendSystemTemperature		(void);
void DWIN_SendSystemReCapa				(void);
void DWIN_SendSystemRemainEnergy	(void);
void DWIN_ShowDualPackConnectionStatus(void);
void DWIN_UpdateSpecialProtection	(void);
void DWIN_SendFETStatusDetail			(void);
void DWIN_UpdatePermanentFail			(void); 	
void DWIN_UpdatePF_IndividualIcons(void);			
void DWIN_Control_SlavePageTouch  (void);			

uint32_t Get_Slave_Total_Cap_mAh(uint8_t index);
uint32_t Get_Slave_Remain_Cap_mAh(uint8_t index);

#endif /* IS_BOOTLOADER == 0 */
#endif
