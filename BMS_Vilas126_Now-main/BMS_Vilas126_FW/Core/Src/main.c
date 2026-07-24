/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../rd_ota/rd_control.h"
#include "../rd_ota/Xmodem.h"

#if IS_BOOTLOADER == 0
#include "BQ769x2Header.h"
#include "kalmanfilter.h"
#include "bq_bms_pylon.h"
#include "pylon_can.h"
#include "bq_bms_485.h"
#include "led_flash.h"
#include "bms_state.h"
#include "ntc_table.h"
#include "led_indication.h"
#include "driver_w25qxx.h"
#include "dwin.h"
#include "DS1307.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if (IS_BOOTLOADER == 0)
#define DEV_ADDR        						0x10  						// BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode        						1     						// 0: disabled, 1: enabled (default use CRC)
#define MAX_BUFFER_SIZE 						64
#define R   												0 								// Read; Used in DirectCommands and Subcommands functions
#define W   												1 								// Write; Used in DirectCommands and Subcommands functions
#define W2  												2 								// Write data with two bytes; Used in Subcommands functions
#define NOMINAL_PACK_V      				51.2f   					// 16 × 3.2V
#define FULL_CHARGE_V       				58.4f   					// 16 × 3.65V ----- 58.4V 
#define EMPTY_V             				43.2f   					// 16 × 2.7V  ----- 44.8V
#define NOMINAL_CAPACITY_mAh        314000.0f					// 314000.0f
#define NOMINAL_VOLTAGE_V           3.2f
#define NET_VOLTAGE_V								3.65f
#define GROSS_UNDER_V								2.7f
#define CELL_COUNT                  16
#define TOTAL_NOMINAL_mWh           (NOMINAL_CAPACITY_mAh * NOMINAL_VOLTAGE_V * CELL_COUNT)
#define FULL_PACK_VOLTAGE_mV        56000      				// 56.0V → full
#define REST_CURRENT_THRESHOLD_mA   20						
#define TAPER_CURRENT_mA            200
#define REST_TIME_MS                3600000     			// 60 minutes
#define AVERAGE_SAMPLES 						64
#define AVERAGE_SAMPLES_CURRENT			16
#define AVERAGE_SAMPLES_ADC 				16
#define MAX_SLAVES 									3
#define FLASH_SOC_ADDR           		0x000000    			
#define FLASH_SOH_ADDR           		0x001000    			
#define FLASH_CYCLE_ADDR         		0x002000    			
#define FLASH_VALID_FLAG_ADDR    		0x003000    			
#define FLASH_VALID_FLAG         		0xAA55AA55  			
#define FLASH_SAVE_INTERVAL_MS   		1800000       		
#define FLASH_BACKUP_ADDR        		0x004000    			
#define FLASH_CHANGE_THRESHOLD 			100 							
#define FLASH_OK                 		0
#define FLASH_INIT_ERROR         		1
#define FLASH_READ_ERROR         		2
#define FLASH_WRITE_ERROR        		3
#define FLASH_CRC_ERROR          		4
#define FLASH_NO_DATA            		5
#define FLASH_EMERGENCY_START   		0x020000  				
#define FLASH_EMERGENCY_SIZE   		 	0x010000  				
#define FLASH_EMERGENCY_PAGE    		256       				
#define FLASH_EMERGENCY_ENTRIES 		(FLASH_EMERGENCY_SIZE / FLASH_EMERGENCY_PAGE)  
#define FLASH_SECTOR_SIZE       		4096
#define WEAR_LEVEL_START_ADDR   		0x000000
#define WEAR_LEVEL_SECTORS      		16
#define WEAR_LEVEL_SIZE         		(WEAR_LEVEL_SECTORS * FLASH_SECTOR_SIZE)
#define SPI_TIMEOUT_VAL 						10
#define PRECHARGE_DIFF_THRESHOLD_MV 1500  
#define PRECHARGE_SAFE_DIFF_MV      500
#define PRECHARGE_THRESHOLD_120A 		12000 
#define RECOVERY_THRESHOLD_15A  		1500
#define DIAG_INTERVAL_MS 						86400000
#define MASK_CUV      							(1u << 2)   			// Cell Undervoltage
#define MASK_COV      							(1u << 3)   			// Cell Overvoltage
#define MASK_OCC      							(1u << 4)   			// Overcurrent Charge
#define MASK_OCD     								(1u << 5)   			// Overcurrent Discharge (primary)
#define MASK_OCD1     							(1u << 6)   			// Overcurrent Discharge (secondary)
#define MASK_SCD      							(1u << 7)   			// Short Circuit Discharge
#define MASK_UTC      							(1u << 0)   			// Under Temperature Charge
#define MASK_UTD      							(1u << 1)   			// Under Temperature Discharge
#define MASK_UTINT    							(1u << 2)   			// Under Temperature Internal
#define MASK_OTC    								(1u << 4)   			// Overtemperature Charge
#define MASK_OTD      							(1u << 5)  			 	// Overtemperature Discharge
#define MASK_OTINT    							(1u << 6)  	 			// Over Temperature Internal
#define MASK_OTF    								(1u << 7)  	 			// Over Temperature Fet
#define MASK_HWDF     							(1u << 1)   			// Host Watchdog Fail
#define MASK_PTO      							(1u << 2)  			 	// Pre-discharge Timeout
#define MASK_COVL     							(1u << 4)  	 			// Cell Overvoltage Latch
#define MASK_OCDL     							(1u << 5)  	 			// Overcurrent Discharge Latch
#define MASK_SCDL     							(1u << 6)  				// Short Circuit Discharge Latch
#define READ_UP_IN()        				HAL_GPIO_ReadPin (UP_IN_GPIO_Port, UP_IN_Pin)
#define SET_DN_OP_HIGH()    				HAL_GPIO_WritePin(DN_OP_GPIO_Port, DN_OP_Pin, GPIO_PIN_SET)
#define SET_DN_OP_LOW()     				HAL_GPIO_WritePin(DN_OP_GPIO_Port, DN_OP_Pin, GPIO_PIN_RESET)
HAL_StatusTypeDef res_1;
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t rx_byte 					 = 0;   

#if (IS_BOOTLOADER == 0)
volatile uint8_t rtc_read_flag 		 = 0; 
volatile uint16_t ADCScanVal[4];
volatile uint8_t debug_up_in_logic = 0;
uint8_t debug_precharge_status 	   = 0;
int32_t debug_diff_voltage_mv 		 = 0;
uint8_t flash_status 							 = 0;
uint16_t max_cell_mv 							 = 0;
uint16_t min_cell_mv							 = 0;
uint32_t charge_stop_tick 				 = 0;
uint16_t vref2_check_counts 			 = 0;
uint8_t first_sample_after_reset 	 = 0;
uint8_t avg_index 								 = 0;
uint8_t avg_index_current 				 = 0;
uint8_t adc_local_index 					 = 0;
uint8_t pf_active_latched 				 = 0;
uint8_t pf_ui_already_updated 		 = 0;
uint8_t dwin_need_update 					 = 0;
uint8_t address 									 = 0; 
volatile uint8_t rx_byte_u3 			 = 0;    
volatile uint8_t rx_byte_u4 			 = 0;    
volatile uint8_t rx_byte_u5 			 = 0; 
extern volatile uint16_t rx_idx_uart5;
extern uint32_t last_master_cmd_tick;
extern uint8_t has_received_master_cmd;
uint8_t slave_reset_fails[MAX_SLAVES] = {0}; 
uint8_t slave_isolated[MAX_SLAVES] = {0};    
uint8_t master_auto_healing_state  = 0;       
uint8_t target_healing_slave 			 = 0;            
uint32_t auto_healing_timer	 			 = 0;
uint8_t healing_in_progress 			 = 0; 
uint8_t master_needs_precharge 		 = 0;
uint8_t system_is_precharging 		 = 0;
uint32_t saved_device_hash 				 = 0;
uint8_t hardware_dip_id 					 = 0;      
uint8_t is_auto_coding 						 = 0;        
uint8_t is_master 								 = 0;            
uint8_t pylon_protocol_addr 			 = 0;
uint32_t can_id_offset 						 = 0;
uint8_t m 												 = 0;
uint8_t current_assigning_index 	 = 3;
uint32_t button_press_start_tick 	 = 0;
volatile uint8_t assigned_n 			 = 0;
Auto_Code_State_t auto_code_state  = AUTO_CODE_START;
uint32_t fet_fail_timer 		 			 = 0;
uint32_t auto_code_start_time 		 = 0;
uint8_t debug_precharge_cmd 			 = 0;   
uint8_t debug_precharge_hw_val 		 = 0; 
int16_t debug_current_val 				 = 0;     
uint16_t system_charge_limit_A 		 = 0;
uint16_t system_discharge_limit_A  = 0;
uint16_t system_charge_v_limit_mV 			= 0;
uint16_t system_discharge_v_limit_mV 		= 0;
int32_t system_total_current_mA		 			= 0;
int32_t system_total_current_01A 				= 0;
uint16_t system_total_voltage_01V 			= 0;      
uint16_t system_charge_limit_A_final 		= 0;   
uint16_t system_discharge_limit_A_final = 0; 
uint16_t system_charge_v_limit_final 		= 0;
uint16_t system_discharge_v_limit_final = 0;
uint16_t system_avg_soc						 = 0;          
uint16_t system_avg_soh						 = 0;
uint16_t system_avg_cycles				 = 0;
uint8_t active_packs_count				 = 0;       
int16_t system_max_temp_C 				 = 0;
int16_t system_min_temp_C 				 = 0;
uint8_t sys_system_error 					 = 0;
extern uint8_t bms_sleep_mode;
extern uint8_t reset_step;
uint32_t last_inverter_alive_tick  = 0; 
uint8_t inverter_comm_fault 			 = 0;
uint8_t system_is_shutting_down 	 = 0; 
uint8_t data_saved_by_button 			 = 0;  
uint8_t RX_data [2] 							 = {0x00, 0x00}; 		// used in several functions to store data read from BQ769x2
uint8_t RX_2Byte [2]							 = {0x00, 0x00};
uint8_t RX_16Byte [16] 						 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t RX_32Byte [32] 						 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //used in Subcommands read function
																											// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage [16] 				 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float Temperature [9] 						 = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float CellVoltage_V[16];
uint16_t Stack_Voltage						 = 0.0f;
uint16_t Pack_Voltage 						 = 0.0f;
uint16_t LD_Voltage 							 = 0.0f;
int16_t Pack_Current 		  				 = 0.0f;
int16_t Raw_Current 							 = 0.0f;
float sum_voltage 								 = 0;
int index_check 									 = 0;
uint8_t alarms_enabled 						 = 0;    
uint32_t init_loop_count 					 = 0; 
uint8_t full_charge_request 		   = 0;
uint8_t force_charge 							 = 0;
float SOC								  				 = 100.0f;
float SOH 												 = 100.0f;
float SOC_Capa										 = 0.0f;
float SOH_Capa										 = 0.0f;
float SOC_Energy									 = 0.0f;
float SOH_Energy									 = 0.0f;
float SOC_Fused										 = 0.0f;
float prev_passed_charge_mAh       = 0.0f;
float passed_charge_mAh       		 = 0.0f;
uint32_t cycle_count							 = 0.0f;
float threshold										 = 0.0f;
float wattUsage 									 = 0.0f;      			// Watt-hour usage
float currentUsage		 						 = 0.0f;    				// Amp-hour usage
float delta_Charge_mAh						 = 0.0f;
uint32_t rest_timer_ms 						 = 0;
uint8_t charge_cycle_active				 = 0;
float charge_accum_mAh						 = 0.0f;
float FullChargeCapacity_mAh	 	   = NOMINAL_CAPACITY_mAh;
KalmanFilter kf_soc;  
KalmanFilter kf_soh;  	
float pack_voltage_adc						 = 0.0f;
float ntc_temp 										 = 0.0f;
float ntc_1	 										 	 = 0.0f;
float ntc_2 										 	 = 0.0f;
float debug_voltage 							 = 0.0f;
float debug_r_lower 							 = 0.0f;
float debug_resistance 						 = 0.0f;
float debug_temp 									 = 0.0f;
float remain_capa_Ah							 = 0.0f;
float soc_ocv											 = 0.0f;
uint8_t sys_protect_ov 						 = 0;
uint8_t sys_protect_uv	 				 	 = 0;
uint8_t sys_protect_occ 					 = 0;
uint8_t sys_protect_ocd 					 = 0;
uint8_t sys_protect_ot 						 = 0;
uint8_t sys_protect_ut 						 = 0;
uint8_t sys_alarm_ov 							 = 0;
uint8_t sys_alarm_uv 							 = 0;
uint8_t sys_alarm_occ 						 = 0;
uint8_t sys_alarm_ocd 					 	 = 0;
uint8_t sys_alarm_ot 							 = 0;
uint8_t sys_alarm_ut 							 = 0;
uint8_t uv_recovery_locked				 = 0;
uint8_t ov_recovery_locked				 = 0;
uint8_t occ_streak_count 					 = 0;      
uint8_t occ_software_lock 				 = 0;      
uint8_t prev_occ_bit 							 = 0;          
uint32_t last_occ_timestamp 			 = 0;   
uint32_t sleep_timer_start 				 = 0;
uint32_t mos_ot_lock_tick 				 = 0;
uint8_t cell_failure_count 				 = 0;
uint8_t cell_failure_locked 			 = 0;
uint8_t last_lock_state 					 = 0;  
uint32_t system_total_capacity_mAh = 0;
uint8_t min_v_pack_id 						 = 1;
uint8_t max_v_pack_id 						 = 1;
uint8_t min_t_pack_id 						 = 1, min_t_cell_id = 1;
uint8_t max_t_pack_id 						 = 1, max_t_cell_id = 1;
uint16_t local_max_cell_v 				 = 0;
uint16_t local_min_cell_v 				 = 0xFFFF;
uint8_t local_max_cell_id 				 = 0;
uint8_t local_min_cell_id 				 = 0;
uint16_t sys_max_cell_v 					 = 0;
uint16_t sys_min_cell_v 					 = 0xFFFF;
uint8_t sys_max_v_pack_id 				 = 0;
uint8_t sys_max_v_cell_id 				 = 0;
uint8_t sys_min_v_pack_id 				 = 0;
uint8_t sys_min_v_cell_id	 				 = 0;
uint8_t sys_max_t_pack_id 				 = 0;
uint8_t sys_max_t_cell_id 				 = 0;
uint8_t sys_min_t_pack_id 				 = 0;
uint8_t sys_min_t_cell_id 				 = 0;
uint8_t adv_soc_clamp_active 									= 0;
uint8_t adv_stop_discharge_active 						= 0;
static uint32_t adv_soc_clamp_timer_ms 				= 0;
static uint32_t adv_real_charge_timer_ms 			= 0;
uint16_t AlarmBits 								 = 0x00;
uint8_t value_SafetyAlertA;  													// Safety Status Register A
uint8_t value_SafetyAlertB;  													// Safety Status Register B
uint8_t value_SafetyAlertC;  													// Safety Status Register C
uint8_t value_SafetyStatusA;  		  									// Safety Status Register A
uint8_t value_SafetyStatusB;  		  									// Safety Status Register B
uint8_t value_SafetyStatusC;  		  									// Safety Status Register C
uint8_t value_PFStatusA;   														// Permanent Fail Status Register A
uint8_t value_PFStatusB;   														// Permanent Fail Status Register B
uint8_t value_PFStatusC;   														// Permanent Fail Status Register C
uint8_t value_PFStatusD;   														// Permanent Fail Status Register D
uint8_t FET_Status;  																	// FET Status register contents  - Shows states of FETs
uint8_t value_FETOptions; 														// PreDischarge status
uint16_t CB_ActiveCells;  														// Cell Balancing Active Cells
uint16_t battery_status_sleep 			= 0;
uint16_t control_status     				= 0;  						// 0x00 Control Status
uint16_t battery_status     				= 0;  						// 0x12 Battery Status
uint16_t alarm_status_reg   				= 0;  						// 0x62 Alarm Status (latched)
uint16_t alarm_raw_status   				= 0;  						// 0x64 Alarm Raw Status (unlatched)
uint16_t alarm_enable_mask  				= 0;  				    // 0x66 Alarm Enable mask
uint16_t manufacturing_status 			= 0;
uint64_t OTP;
uint16_t OTP_Status;
uint8_t CFGUPDATE 	  							= 0;
uint8_t PCHG_MODE 	  							= 0;
uint8_t SLEEP_EN 		  							= 0;
uint8_t POR 				  							= 0;
uint8_t WD 				 	  							= 0;
uint8_t COW_CHK 	    							= 0;
uint8_t OTPW 				  							= 0;
uint8_t OTPB 			 	  							= 0;
uint8_t SEC0 				  							= 0;
uint8_t SEC1 			 	  							= 0;
uint8_t FUSE 				  							= 0;
uint8_t SS 					  							= 0;
uint8_t PF 					  							= 0;
uint8_t SDM 				  							= 0;
uint8_t SLEEP 			  							= 0;   						// OTP write pending state
uint8_t	UV_Fault 		  							= 0;   						// under-voltage fault state
uint8_t	OV_Fault 		  							= 0;   						// over-voltage fault state
uint8_t	OCC_Fault 	  							= 0;
uint8_t	OCD_Fault 	  							= 0;  						// over-current fault state
uint8_t	OCD_Fault1 	  							= 0;
uint8_t	SCD_Fault	 	  							= 0;  						// short-circuit fault state
uint8_t	OTF_Fault 	  							= 0;   						// under-voltage fault state
uint8_t	OTINT_Fault   							= 0;   						// over-voltage fault state
uint8_t	OTD_Fault 	  							= 0;  						// short-circuit fault state
uint8_t	OTC_Fault 	  							= 0;  						// over-current fault state
uint8_t	UTINT_Fault   							= 0;
uint8_t	UTD_Fault 	  							= 0;
uint8_t	UTC_Fault 	  							= 0;
uint8_t OCD3_Fault									= 0;
uint8_t SCDL_Fault									= 0;
uint8_t OCDL_Fault									= 0;
uint8_t COVL_Fault									= 0;
uint8_t PTO_Fault									  = 0;
uint8_t HWDF_Fault									= 0;
uint8_t	SUV_PF_Fault 							  = 0; 							// Severe Cell Undervoltage
uint8_t	SOV_PF_Fault 								= 0; 							// Severe Cell Overvoltage
uint8_t	SOCC_PF_Fault 							= 0;					 		// Severe Overcurrent in Charge
uint8_t	SOCD_PF_Fault 							= 0; 							// Severe Overcurrent in Discharge
uint8_t	SOT_PF_Fault 								= 0; 							// Severe Overtemperature
uint8_t	SOTF_PF_Fault 							= 0; 							// Severe Overtemperature FET
uint8_t	CUDEP_PF_Fault 							= 0; 							// Copper Deposition
uint8_t	CFETF_PF_Fault 							= 0; 							// Charge FET Failure
uint8_t	DFETF_PF_Fault 							= 0; 							// Discharge FET Failure
uint8_t	LVL2_PF_Fault 							= 0; 							// Second Level Protector
uint8_t	VIMR_PF_Fault 							= 0; 							// Voltage Imbalance at Rest
uint8_t	VIMA_PF_Fault 							= 0; 							// Voltage Imbalance Active
uint8_t	SCDL_PF_Fault 							= 0; 							// Short Circuit in Discharge Latch
uint8_t	OTPF_PF_Fault 							= 0; 							// OTP Memory Failure
uint8_t	DRMF_PF_Fault 							= 0; 							// Data ROM Failure
uint8_t	IRMF_PF_Fault 							= 0; 							// Instruction ROM Failure
uint8_t	LFOF_PF_Fault 							= 0; 							// Internal LFO Failure
uint8_t	VREF_PF_Fault 							= 0; 							// Internal Voltage Reference Failure
uint8_t	VSSF_PF_Fault 							= 0; 							// Internal VSS Measurement Failure
uint8_t	HWMX_PF_Fault 							= 0; 							// Hardware Mux Failure
uint8_t	CMDF_PF_Fault 							= 0;	 						// Commanded Permanent Fail
uint8_t	TOSF_PF_Fault 							= 0; 							// Top of Stack vs Cell Sum
uint8_t PFErrorsTriggered 					= 0;
uint8_t ProtectionsTriggered 				= 0; 							// Set to 1 if any protection triggers
uint8_t LD_ON 			  							= 0;							// Load Detect status bit
uint8_t DSG 			 	  							= 0;   						// discharge FET state
uint8_t CHG 				  							= 0;   						// charge FET state
uint8_t PCHG 				 								= 0;  						// pre-charge FET state
uint8_t PDSG 				  							= 0;  						// pre-discharge FET state
uint8_t DCHG_pin 		  							= 0;
uint8_t DDSG_pin 		  							= 0;
uint8_t ALRT_pin 		  							= 0;
uint8_t RSVD_pin 		  							= 0;
uint8_t RSVD_00 										= 0;
uint8_t RSVD_01 										= 0;
uint8_t FET_INIT_OFF 								= 0;
uint8_t PDSG_EN 										= 0;
uint8_t FET_CTRL_EN 								= 0;
uint8_t HOST_FET_EN 								= 0;
uint8_t SLEEPCHG 										= 0;
uint8_t SFET 												= 0;
uint32_t CC1; 																				// in BQ769x2_READPASSQ func
uint32_t CC2;																					// in BQ769x2_READPASSQ func
uint32_t CC3;																					// in BQ769x2_READPASSQ func
int32_t  AccumulatedCharge_Int; 											// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac;											// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time;											// in BQ769x2_READPASSQ func
CAN_TxHeaderTypeDef   TxHeader; 											/* Header containing the information of the transmitted frame */
CAN_RxHeaderTypeDef   RxHeader; ; 										/* Header containing the information of the received frame */
uint8_t               TxData[8] = {0};  							/* Buffer of the data to send */
uint8_t               RxData[8];										 	/* Buffer of the received data */
uint32_t              TxMailbox;  										/* The number of the mail box that transmitted the Tx message */
DS1307_TIME time;
uint16_t cell_buffer[16]	[AVERAGE_SAMPLES] 				= {0};
uint16_t pack_buffer			[AVERAGE_SAMPLES] 				= {0};
uint16_t stack_buffer			[AVERAGE_SAMPLES]   			= {0};
uint16_t ld_buffer				[AVERAGE_SAMPLES] 				= {0};
int16_t  current_buffer		[AVERAGE_SAMPLES_CURRENT] = {0};
uint16_t adc_buffer[4] 		[AVERAGE_SAMPLES_ADC] 		= {0};
uint16_t last_valid_stack_voltage 		 = 0;      		
int16_t  last_valid_current 					 = 0;           
float    last_valid_soc 							 = 100.0f;      
uint16_t last_valid_pack_voltage  		 = 0;   
uint16_t last_valid_ld_voltage    		 = 0;  
static uint16_t last_saved_soc_x100 	 = 0;
static uint16_t last_saved_soh_x100 	 = 0;
static uint32_t last_saved_cycle  		 = 0;
static uint32_t flash_save_count 		   = 0;  					 
static float accumulated_discharge_mAh = 0.0f;  
static float mAh_at_last_sync 				 = 0.0f;
static uint32_t system_start_time 		 = 0;           
uint8_t protection_blocked 		 			 = 0;         
uint16_t battery_status_value 	 			 = 0;        
static uint32_t emergency_write_offset = 0;  
static uint32_t current_sequence 			 = 0;
static uint8_t current_wear_slot 			 = 0;       
static uint32_t last_full_charge_day 	 = 0;  
static float soc_at_charge_start       = 100.0f;
static bool  in_significant_charge     = false;
static uint32_t significant_charge_start_tick = 0;      
uint8_t current_pf_status[4]		 			 = {0, 0, 0, 0};
w25qxx_handle_t flash_handle;
uint32_t last_flash_save_time 	       = 0;
uint8_t flash_init_success 		 				 = 0;
uint16_t last_soc_x100 						 		 = 0;
uint16_t last_soh_x100 						 		 = 0;
int16_t last_current 							 		 = 0;
uint16_t last_stack_v 							 	 = 0;
typedef struct __attribute__((packed)) {
    uint16_t soc_x100;         											
    uint16_t soh_x100;         											
    uint32_t cycle_count;
    float    full_capa_mAh;   
    uint32_t last_full_charge_day;
    uint8_t  pf_status[4];
    uint32_t sequence_number;
    float    mAh_at_last_sync;   
    float    accumulated_discharge_mAh;
    uint32_t device_hash;
		uint8_t  pylon_protocol_addr;
    uint8_t  is_master;
    uint32_t crc;
} bms_flash_data_t;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

#if (IS_BOOTLOADER == 0)

void delayUS(uint32_t us) {   												// Sets the delay in microseconds.
		__HAL_TIM_SET_COUNTER(&htim1, 0);  								// set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim1) < us);  			// wait for the counter to reach the us input in the parameter
}

void Safe_Delay_ms(uint32_t ms) {
    HAL_IWDG_Refresh(&hiwdg);
    for (uint32_t i = 0; i < ms; i++) {
//				rd_run_while_check_uart();
        delayUS(1000); 
//				rd_run_while_check_uart();
        HAL_IWDG_Refresh(&hiwdg);
    }
}

void Emergency_Delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 6000; j++) { 
            __NOP(); 
        }
        HAL_IWDG_Refresh(&hiwdg); 
    }
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.	
{
	unsigned char i;
	unsigned char checksum = 0;
	for(i=0; i<len; i++)
	checksum += ptr[i];
	checksum = 0xff & ~checksum;
	return(checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions 
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= 0x107;
			}
			else
				crc *= 2;
			if((*ptr & i)!=0)
				crc ^= 0x107;
		}
		ptr++;
	}
	return(crc);
}

void I2C_Bus_Recovery(void) {
    __HAL_RCC_I2C1_FORCE_RESET();
    delayUS(10);
    __HAL_RCC_I2C1_RELEASE_RESET();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c1);
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        delayUS(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        delayUS(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) break;
    }
    MX_I2C1_Init(); 
}

void I2C2_Bus_Recovery(void) {
    __HAL_RCC_I2C2_FORCE_RESET();
    delayUS(10);
    __HAL_RCC_I2C2_RELEASE_RESET();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c2);
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        delayUS(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        delayUS(10);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) break;
    }
    MX_I2C2_Init(); 
}

HAL_StatusTypeDef I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
	uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#if CRC_Mode
    {
        uint8_t crc_count = count * 2;
        uint8_t crc1stByteBuffer[3] = {0x10, reg_addr, reg_data[0]};
        TX_Buffer[0] = reg_data[0];
        TX_Buffer[1] = CRC8(crc1stByteBuffer, 3);
        unsigned int j = 2;
        for (unsigned int i = 1; i < count; i++) {
            TX_Buffer[j] = reg_data[i];
            j++;
            uint8_t temp_crc_buffer[1] = {reg_data[i]};
            TX_Buffer[j] = CRC8(temp_crc_buffer, 1);
            j++;
        }
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count, 100);
        return status;
    }
#else
    return HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 10);
#endif
}

HAL_StatusTypeDef I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{ 
	HAL_StatusTypeDef status = HAL_OK;
	unsigned int RX_CRC_Fail = 0;  
	uint8_t RX_Buffer [MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#if CRC_Mode
	{
		uint8_t crc_count = count * 2;
		uint8_t ReceiveBuffer [MAX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		unsigned char CRCc = 0;
		uint8_t temp_crc_buffer [3];
		status = HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, ReceiveBuffer, crc_count, 100);
        if (status != HAL_OK) {
            return status;  
        }
		uint8_t crc1stByteBuffer[4] = {0x10, reg_addr, 0x11, ReceiveBuffer[0]};
        CRCc = CRC8(crc1stByteBuffer, 4);
        if (CRCc != ReceiveBuffer[1]) {
            RX_CRC_Fail++;
            return HAL_ERROR;  
        }
		RX_Buffer[0] = ReceiveBuffer[0];
		unsigned int j = 2;
        for (unsigned int i = 1; i < count; i++) {
            RX_Buffer[i] = ReceiveBuffer[j];
            temp_crc_buffer[0] = ReceiveBuffer[j];
            j++;
            CRCc = CRC8(temp_crc_buffer, 1);
            if (CRCc != ReceiveBuffer[j]) {
                RX_CRC_Fail++;
                return HAL_ERROR; 
            }
            j++;
        }
        CopyArray(RX_Buffer, reg_data, count);
        return HAL_OK;
    }
#else
    status = HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 10);
    return status;
#endif
}

HAL_StatusTypeDef I2C_ReadReg_WithRetry(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    HAL_StatusTypeDef status;
    for (uint8_t retry = 0; retry < 3; retry++) {
        status = I2C_ReadReg(reg_addr, reg_data, count);  
        if (status == HAL_OK) {
            if (reg_addr == 0x40 && count >= 3) {
                if (reg_data[0] == 0x00 && reg_data[1] == 0xFF && reg_data[2] == 0xFF) {
                    status = HAL_BUSY; 
                    Safe_Delay_ms(2);
                    continue;
                }
            }
            return HAL_OK;
        }
        I2C_Bus_Recovery();
        Safe_Delay_ms(5);
    }
    return status;
}

HAL_StatusTypeDef I2C_WriteReg_WithRetry(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    HAL_StatusTypeDef status;
    for (uint8_t retry = 0; retry < 3; retry++) {
        status = I2C_WriteReg(reg_addr, reg_data, count);
        if (status == HAL_OK) return HAL_OK;       
        I2C_Bus_Recovery();
        Safe_Delay_ms(5);
    }
    return status;
}

HAL_StatusTypeDef BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2] = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HAL_StatusTypeDef status = HAL_OK;
    // TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff; // 1st byte of data
    switch (datalen)
    {
        case 1: // 1 byte datalength
            status = I2C_WriteReg_WithRetry(0x3E, TX_RegData, 3);
            if (status != HAL_OK) return status;
            Safe_Delay_ms(2);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05; // combined length of register address and data
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
            if (status != HAL_OK) return status;
            Safe_Delay_ms(2);
            break;
        case 2: // 2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            status = I2C_WriteReg_WithRetry(0x3E, TX_RegData, 4);
            if (status != HAL_OK) return status;
            Safe_Delay_ms(2);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06; // combined length of register address and data
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
            if (status != HAL_OK) return status;
            Safe_Delay_ms(2);
            break;
        case 4: // 4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            status = I2C_WriteReg_WithRetry(0x3E, TX_RegData, 6);
            if (status != HAL_OK) return status;
            Safe_Delay_ms(2);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08; // combined length of register address and data
            status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
            if (status != HAL_OK) return status;
            Safe_Delay_ms(2);
            break;
        default:
            return HAL_ERROR; // Invalid datalen
    }
    return HAL_OK;
}

HAL_StatusTypeDef DM_Read8(uint16_t addr, uint8_t *p_value)
{
    uint8_t tx[2] = { addr & 0xFF, (uint8_t)(addr >> 8) };
    HAL_StatusTypeDef status;
    status = I2C_WriteReg_WithRetry(0x3E, tx, 2);
    if (status != HAL_OK) {
        return status;
    }
    Safe_Delay_ms(2);
    uint8_t temp_data = 0;
    status = I2C_ReadReg_WithRetry(0x40, &temp_data, 1);
    if (status == HAL_OK) {
        *p_value = temp_data;
        return HAL_OK;
    }
    return status;
}

HAL_StatusTypeDef CommandSubcommands(uint16_t command)
{
    uint8_t TX_Reg[2] = {0};
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;
    HAL_StatusTypeDef status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 2);
    Safe_Delay_ms(2);
    return status;
}

HAL_StatusTypeDef Subcommands(uint16_t command, uint16_t data, uint8_t type)
{
    uint8_t TX_Reg[4] = {0};
    uint8_t TX_Buffer[2] = {0};
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;
		if (type == R) {
				HAL_StatusTypeDef status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 2);
				if (status != HAL_OK) return status;
				uint32_t timeout_start = HAL_GetTick();
				uint8_t cmd_check[2];
				do {
						status = I2C_ReadReg_WithRetry(0x3E, cmd_check, 2);
						if (status != HAL_OK) return status;
						if (cmd_check[0] == TX_Reg[0] && cmd_check[1] == TX_Reg[1]) {
								break;
						}
						delayUS(100);  
						if (HAL_GetTick() - timeout_start > 100) {  
								return HAL_TIMEOUT;
						}
				} while (1);
				return I2C_ReadReg_WithRetry(0x40, RX_32Byte, 32);
		}
    else if (type == W) {
        TX_Reg[2] = data & 0xff;
        HAL_StatusTypeDef status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 3);
        if (status != HAL_OK) return status;
        Safe_Delay_ms(1);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;
        status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        if (status != HAL_OK) return status;
        Safe_Delay_ms(1);
        return HAL_OK;
    }
    else if (type == W2) {
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        HAL_StatusTypeDef status = I2C_WriteReg_WithRetry(0x3E, TX_Reg, 4);
        if (status != HAL_OK) return status;
        Safe_Delay_ms(1);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;
        status = I2C_WriteReg_WithRetry(0x60, TX_Buffer, 2);
        if (status != HAL_OK) return status;
        Safe_Delay_ms(1);
        return HAL_OK;
    }
    return HAL_OK;
}

HAL_StatusTypeDef DirectCommands(uint8_t command, uint16_t data, uint8_t type, HAL_StatusTypeDef *ptr_status)
{
    uint8_t TX_data[2] = {data & 0xff, (data >> 8) & 0xff};
    if (type == R) {
        HAL_StatusTypeDef status = I2C_ReadReg_WithRetry(command, RX_data, 2);
        if (ptr_status) *ptr_status = status;
        return status;
    }
    if (type == W) {
        HAL_StatusTypeDef status = I2C_WriteReg_WithRetry(command, TX_data, 2);
        Safe_Delay_ms(2);
        if (ptr_status) *ptr_status = status;
        return status;
    }
    if (ptr_status) *ptr_status = HAL_OK;
    return HAL_OK;
}

void Unlock_BQ76952(void) {
    CommandSubcommands(0x1011); Safe_Delay_ms(2);
    CommandSubcommands(0x2001); Safe_Delay_ms(2);
    CommandSubcommands(0x4C4F); Safe_Delay_ms(2);
    CommandSubcommands(0x4E47); Safe_Delay_ms(2);

    CommandSubcommands(0x0414); Safe_Delay_ms(2);
    CommandSubcommands(0x3672); Safe_Delay_ms(2);
    CommandSubcommands(0xFFFF); Safe_Delay_ms(2);
    CommandSubcommands(0xFFFF); Safe_Delay_ms(2);
}

void BQ769x2_Init() {
	Unlock_BQ76952();
	HAL_StatusTypeDef status = CommandSubcommands(SET_CFGUPDATE);
  if (status != HAL_OK) {
    return; 
  }
	// Calibration
	BQ769x2_SetRegister(Cell1Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell2Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell3Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell4Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell5Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell6Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell7Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell8Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell9Gain, 0, 2);											// Done
	BQ769x2_SetRegister(Cell10Gain, 0, 2);										// Done
	BQ769x2_SetRegister(Cell11Gain, 0, 2);										// Done
	BQ769x2_SetRegister(Cell12Gain, 0, 2);										// Done
	BQ769x2_SetRegister(Cell13Gain, 0, 2);										// Done
	BQ769x2_SetRegister(Cell14Gain, 0, 2);										// Done
	BQ769x2_SetRegister(Cell15Gain, 0, 2);										// Done
	BQ769x2_SetRegister(Cell16Gain, 0, 2);										// Done
	BQ769x2_SetRegister(PackGain, 0, 2);											// Done
	BQ769x2_SetRegister(TOSGain, 0, 2);												// Done
	BQ769x2_SetRegister(LDGain, 0, 2);												// Done
	BQ769x2_SetRegister(ADCGain, 0, 2);												// Done
	BQ769x2_SetRegister(CCGain, 0x42958937, 4);      					// Done	// Rsense 7.4768 / 0.1 = 74.768*
	BQ769x2_SetRegister(CapacityGain, 0x4BAA2384, 4);					// Done // 298261.6178 * CCGain
	BQ769x2_SetRegister(VcellOffset, 0, 2);        						// Done
	BQ769x2_SetRegister(VdivOffset, 0, 2);     								// Done
	BQ769x2_SetRegister(CoulombCounterOffsetSamples, 64, 2);	// Done
	BQ769x2_SetRegister(BoardOffset, 18, 2);									// Done // Initial 0
	BQ769x2_SetRegister(InternalTempOffset, 0, 1);
	BQ769x2_SetRegister(CFETOFFTempOffset, 0, 1);
	BQ769x2_SetRegister(DFETOFFTempOffset, 0, 1);
	BQ769x2_SetRegister(ALERTTempOffset, 0, 1);
	BQ769x2_SetRegister(TS1TempOffset, 0, 1);
	BQ769x2_SetRegister(TS2TempOffset, 0, 1);
	BQ769x2_SetRegister(TS3TempOffset, 0, 1);
	BQ769x2_SetRegister(HDQTempOffset, 0, 1);
	BQ769x2_SetRegister(DCHGTempOffset, 0, 1);
	BQ769x2_SetRegister(DDSGTempOffset, 0, 1);
	BQ769x2_SetRegister(IntGain, 25390, 2);
	BQ769x2_SetRegister(Intbaseoffset, 3032, 2);
	BQ769x2_SetRegister(IntMaximumAD, 16383, 2);
	BQ769x2_SetRegister(IntMaximumTemp, 6379, 2);
	BQ769x2_SetRegister(T18kCoeffa1, (uint16_t)-15524, 2);
	BQ769x2_SetRegister(T18kCoeffa2, 26423, 2);
	BQ769x2_SetRegister(T18kCoeffa3, (uint16_t)-22664, 2);
	BQ769x2_SetRegister(T18kCoeffa4, 28834, 2);
	BQ769x2_SetRegister(T18kCoeffa5, 672, 2);
	BQ769x2_SetRegister(T18kCoeffb1, (uint16_t)-371, 2);
	BQ769x2_SetRegister(T18kCoeffb2, 708, 2);
	BQ769x2_SetRegister(T18kCoeffb3, (uint16_t)-3498, 2);
	BQ769x2_SetRegister(T18kCoeffb4, 5051, 2);
	BQ769x2_SetRegister(T18kAdc0, 11703, 2);
	BQ769x2_SetRegister(T180kCoeffa1, (uint16_t)-17513, 2);
	BQ769x2_SetRegister(T180kCoeffa2, 25759, 2);
	BQ769x2_SetRegister(T180kCoeffa3, (uint16_t)-23593, 2);
	BQ769x2_SetRegister(T180kCoeffa4, 32175, 2);
	BQ769x2_SetRegister(T180kCoeffa5, 2090, 2);
	BQ769x2_SetRegister(T180kCoeffb1, (uint16_t)-2055, 2);
	BQ769x2_SetRegister(T180kCoeffb2, 2955, 2);
	BQ769x2_SetRegister(T180kCoeffb3, (uint16_t)-3427, 2);
	BQ769x2_SetRegister(T180kCoeffb4, 4385, 2);
	BQ769x2_SetRegister(T180kAdc0, 17246, 2);
	BQ769x2_SetRegister(CustomCoeffa1, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa2, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa3, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa4, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffa5, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb1, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb2, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb3, 0x00, 2);
	BQ769x2_SetRegister(CustomCoeffb4, 0x00, 2);
	BQ769x2_SetRegister(CustomRc0, 0x00, 2);
	BQ769x2_SetRegister(CustomAdc0, 0x00, 2);
	BQ769x2_SetRegister(CoulombCounterDeadband, 9, 1);				// Done
	BQ769x2_SetRegister(CUVThresholdOverride, 0xFFFF, 2);			// Done
	BQ769x2_SetRegister(COVThresholdOverride, 0xFFFF, 2);			// Done
//	BQ769x2_SetRegister(MinBlowFuseVoltage, 500, 2);
//	BQ769x2_SetRegister(FuseBlowTimeout, 30, 1);
	BQ769x2_SetRegister(PowerConfig, 0x2CB0, 2);							// 0x2C80	
	BQ769x2_SetRegister(REG12Config, 0x00, 1);								// OFF LDO (0x0D)
	BQ769x2_SetRegister(REG0Config, 0x00, 1);									// OFF REG0 (0x01)
	BQ769x2_SetRegister(HWDRegulatorOptions, 0x00, 1);
	BQ769x2_SetRegister(CommType, 0x12, 1);
	BQ769x2_SetRegister(I2CAddress, 0, 1);
	BQ769x2_SetRegister(SPIConfiguration, 0x20, 1);
	BQ769x2_SetRegister(CommIdleTime, 0, 1);
	BQ769x2_SetRegister(DFETOFFPinConfig, 0x46, 1);						// 0x00
	BQ769x2_SetRegister(CFETOFFPinConfig, 0x02, 1);						// 0x07	
	BQ769x2_SetRegister(ALERTPinConfig, 0x82, 1);							// 0x00
	BQ769x2_SetRegister(TS1Config, 0x07, 1);					
	BQ769x2_SetRegister(TS2Config, 0x07, 1);
	BQ769x2_SetRegister(TS3Config, 0x07, 1);	
	BQ769x2_SetRegister(HDQPinConfig, 0x07, 1);  							// 0x0B
	BQ769x2_SetRegister(DCHGPinConfig, 0x01, 1); 							// 0x02
	BQ769x2_SetRegister(DDSGPinConfig, 0x01, 1); 							// 0x02
	BQ769x2_SetRegister(DAConfiguration, 0x06, 1); 						// Done 
	BQ769x2_SetRegister(VCellMode, 0xFFFF, 2);								// Done	
	BQ769x2_SetRegister(CC3Samples, 80, 1);
	BQ769x2_SetRegister(ProtectionConfiguration, 0x0602, 2); 	// 0x0002
	BQ769x2_SetRegister(EnabledProtectionsA, 0xFC, 1);			 	// 0xFC
	BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);
	BQ769x2_SetRegister(EnabledProtectionsC, 0x76, 1);				// 0x56
	BQ769x2_SetRegister(CHGFETProtectionsA, 0x98, 1);
	BQ769x2_SetRegister(CHGFETProtectionsB, 0xD5, 1);
	BQ769x2_SetRegister(CHGFETProtectionsC, 0x56, 1);
	BQ769x2_SetRegister(DSGFETProtectionsA, 0xE4, 1);	
	BQ769x2_SetRegister(DSGFETProtectionsB, 0xE6, 1);
	BQ769x2_SetRegister(DSGFETProtectionsC, 0x62, 1);
	BQ769x2_SetRegister(BodyDiodeThreshold, 2000, 2);
	BQ769x2_SetRegister(DefaultAlarmMask, 0xF800, 2);
	BQ769x2_SetRegister(SFAlertMaskA, 0xFC, 1);
	BQ769x2_SetRegister(SFAlertMaskB, 0xF7, 1);
	BQ769x2_SetRegister(SFAlertMaskC, 0x74, 1);
	BQ769x2_SetRegister(PFAlertMaskA, 0x5F, 1);
	BQ769x2_SetRegister(PFAlertMaskB, 0x9F, 1);
	BQ769x2_SetRegister(PFAlertMaskC, 0x00, 1);
	BQ769x2_SetRegister(PFAlertMaskD, 0x00, 1);
	BQ769x2_SetRegister(EnabledPFA, 0x5F, 1);
	BQ769x2_SetRegister(EnabledPFB, 0x90, 1);
	BQ769x2_SetRegister(EnabledPFC, 0x07, 1);
	BQ769x2_SetRegister(EnabledPFD, 0x01, 1);									// Done
	BQ769x2_SetRegister(FETOptions, 0x1D, 1);									// Done or 0x1D
	BQ769x2_SetRegister(ChgPumpControl, 0x05, 1);							// Done
	BQ769x2_SetRegister(PrechargeStartVoltage, 2750, 2);		  // Done //2800
	BQ769x2_SetRegister(PrechargeStopVoltage, 2900, 2);				// Done	//2950
	BQ769x2_SetRegister(PredischargeTimeout, 250, 1);	
	BQ769x2_SetRegister(PredischargeStopDelta, 20, 1);
	BQ769x2_SetRegister(DsgCurrentThreshold, 10, 2);				  // Discharge State 100mA
	BQ769x2_SetRegister(ChgCurrentThreshold, 5, 2);				 	  // Charge State 50mA
	BQ769x2_SetRegister(CheckTime, 5, 1);
	BQ769x2_SetRegister(Cell1Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell2Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell3Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell4Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell5Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell6Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell7Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell8Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell9Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell10Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell11Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell12Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell13Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell14Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell15Interconnect, 0, 2);
	BQ769x2_SetRegister(Cell16Interconnect, 0, 2);						
	BQ769x2_SetRegister(MfgStatusInit, 0x00D0, 2);						// Done
	BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);			// Done
	BQ769x2_SetRegister(MinCellTemp, (uint8_t)-20, 1);   			// Done -20C
	BQ769x2_SetRegister(MaxCellTemp, 60, 1);   								// Done 60C
	BQ769x2_SetRegister(MaxInternalTemp, 70, 1); 							// Done 70C
	BQ769x2_SetRegister(CellBalanceInterval, 3, 1);					  // Done
	BQ769x2_SetRegister(CellBalanceMaxCells, 12, 1);					// Done
	BQ769x2_SetRegister(CellBalanceMinCellVCharge, 3400, 2);	// Equalizing Opening Voltage 3400mV
	BQ769x2_SetRegister(CellBalanceMinDeltaCharge, 30, 1);		// Opening Pressure Difference 30mV
	BQ769x2_SetRegister(CellBalanceStopDeltaCharge, 20, 1);		// Done
	BQ769x2_SetRegister(CellBalanceMinCellVRelax, 3400, 2);		// Equalizing Opening Voltage 3400mV
	BQ769x2_SetRegister(CellBalanceMinDeltaRelax, 30, 1);			// Opening Pressure Difference 30mV
	BQ769x2_SetRegister(CellBalanceStopDeltaRelax, 20, 1);		// Done
	BQ769x2_SetRegister(ShutdownCellVoltage, 0, 2);			      // Done
	BQ769x2_SetRegister(ShutdownStackVoltage, 4320, 2);				// Shutdown Stack Voltage 43200mV
	BQ769x2_SetRegister(LowVShutdownDelay, 5, 1);  						// Delay 2s
	BQ769x2_SetRegister(ShutdownTemperature, 80, 1);					// Done
	BQ769x2_SetRegister(ShutdownTemperatureDelay, 5, 1);			// Done
	BQ769x2_SetRegister(FETOffDelay, 0, 1);
	BQ769x2_SetRegister(ShutdownCommandDelay, 0, 1);
	BQ769x2_SetRegister(AutoShutdownTime, 0, 1);
	BQ769x2_SetRegister(RAMFailShutdownTime, 5, 1);
	BQ769x2_SetRegister(SleepCurrent, 10, 2);									// Sleep Current is 10mA
	BQ769x2_SetRegister(VoltageTime, 5, 1);										// Sleep Voltage time 5s
	BQ769x2_SetRegister(WakeComparatorCurrent, 200, 2);				// Icc > 200mA in Sleep ->>> Normal Mode
	BQ769x2_SetRegister(SleepHysteresisTime, 10, 1);					// Sleep Hysteresis Time 10s
	BQ769x2_SetRegister(SleepChargerVoltageThreshold, 6000,2);// Done
	BQ769x2_SetRegister(SleepChargerPACKTOSDelta, 200, 2);		// Done
	BQ769x2_SetRegister(ConfigRAMSignature, 0, 2);
	BQ769x2_SetRegister(CUVThreshold, 54, 1);									// Individual Over Discharge Protection: Protection Voltage 2700mV = 54 * 50.6 = 2732.4mV (-32.4mV) 			
 	BQ769x2_SetRegister(CUVDelay, 301, 2);										// Individual Over Discharge Protection: Protection Delay 1.0s / 3.3ms - 2 = 301
 	BQ769x2_SetRegister(CUVRecoveryHysteresis, 4, 1);					// Individual Over Discharge Protection: Voltage of Protection Released 2950mV = 58 * 50.6 = 2934.8mV 
	BQ769x2_SetRegister(COVThreshold, 72, 1);									// Single Over Charge Protection: Protection Voltage 3650mV = 72*50.6 = 3643.2mV (-6.8mV)
	BQ769x2_SetRegister(COVDelay, 301, 2);										// Single Over Charge Protection: Protection Delay 1.0s / 3.3ms - 2 = 301
	BQ769x2_SetRegister(COVRecoveryHysteresis, 5, 1);					// Single Over Charge Protection: Voltage of Protection Released 3380mV = 67 * 50.6 = 3390.2 mV
	BQ769x2_SetRegister(COVLLatchLimit, 3, 1);								// Done
	BQ769x2_SetRegister(COVLCounterDecDelay, 10, 1);					// Done
	BQ769x2_SetRegister(COVLRecoveryTime, 15, 1);							// Done 
	BQ769x2_SetRegister(OCCThreshold, 7, 1);									// Rsense 140A
	BQ769x2_SetRegister(OCCDelay, 127, 1);										// OCC Delay 127 + 2 * 3.3 = 425ms
	BQ769x2_SetRegister(OCCRecoveryThreshold, (uint16_t)-1000, 2);	// Discharge current > 1 A for Protection Recovery Time 60s 
	BQ769x2_SetRegister(OCCPACKTOSDelta, 200, 2);	
	BQ769x2_SetRegister(OCD1Threshold, 7, 1);									// Discharge Over Current I Protection: Rsense 140A
	BQ769x2_SetRegister(OCD1Delay, 127, 1);										// OCD1 Delay 127 + 2 * 3.3 = 425ms
	BQ769x2_SetRegister(OCD2Threshold, 8, 1);								  // Protection OCD2 160A
	BQ769x2_SetRegister(OCD2Delay, 127, 1);										// OCD2 Delay 127 + 2 * 3.3 = 425ms
	BQ769x2_SetRegister(SCDThreshold, 6, 1);  								// SCD 1250A
	BQ769x2_SetRegister(SCDDelay, 31, 1);											// SCD 450us
	BQ769x2_SetRegister(SCDRecoveryTime, 5, 1);								// SCD Recovery 5s
	BQ769x2_SetRegister(OCD3Threshold, (uint16_t)-2000, 2);	  // Off OCD3
	BQ769x2_SetRegister(OCD3Delay, 2, 1);											// Off OCD3
	BQ769x2_SetRegister(OCDRecoveryThreshold, 1000, 2);				// Icc > 1A recovery OCD
	BQ769x2_SetRegister(OCDLLatchLimit, 10, 1);								// OCD 10 times latch 
	BQ769x2_SetRegister(OCDLCounterDecDelay, 10, 1);	
	BQ769x2_SetRegister(OCDLRecoveryTime, 60, 1);							// Auto realese after 1 min
	BQ769x2_SetRegister(OCDLRecoveryThreshold, 1000, 2);			// Icc > 1A recovery OCDL
	BQ769x2_SetRegister(SCDLLatchLimit, 5, 1);
	BQ769x2_SetRegister(SCDLCounterDecDelay, 10, 1);	
	BQ769x2_SetRegister(SCDLRecoveryTime, 15, 1);	
	BQ769x2_SetRegister(SCDLRecoveryThreshold, 1000, 2);	
	BQ769x2_SetRegister(OTCThreshold, 65, 1);									// Over Temperature in Charge 65C
	BQ769x2_SetRegister(OTCDelay, 2, 1);											// Delay 2s
	BQ769x2_SetRegister(OTCRecovery, 55, 1);									// Over Temperature in Charge Release 55C
	BQ769x2_SetRegister(OTDThreshold, 70, 1);									// Over Temperature in Discharge 70C
	BQ769x2_SetRegister(OTDDelay, 2, 1);											// Delay 2s
	BQ769x2_SetRegister(OTDRecovery, 60, 1);									// Over Temperature in Discharge Release 60C
	BQ769x2_SetRegister(OTFThreshold, 115, 1);								// Over Temperature Protection 115C
	BQ769x2_SetRegister(OTFDelay, 2, 1);											// 2s Delay
	BQ769x2_SetRegister(OTFRecovery, 85, 1);									// Over Temperature Protection Release 85C
	BQ769x2_SetRegister(OTINTThreshold, 75, 1);								// Over Temperature Int 75C
	BQ769x2_SetRegister(OTINTDelay, 2, 1);										// Delay 2s
	BQ769x2_SetRegister(OTINTRecovery, 65, 1);								// Over Temperature Int Release 65C
	BQ769x2_SetRegister(UTCThreshold, 0, 1);									// Under Temperature in Charge 0C
	BQ769x2_SetRegister(UTCDelay, 2, 1);											// Delay 2s		
	BQ769x2_SetRegister(UTCRecovery, 5, 1);										// Under Temperature in Charge Release 5C
	BQ769x2_SetRegister(UTDThreshold, (uint8_t)-20, 1);				// Under Temperature in Discharge -20C
	BQ769x2_SetRegister(UTDDelay, 2, 1);											// Delay 2s
	BQ769x2_SetRegister(UTDRecovery, (uint8_t)-15, 1);			  // Under Temperature in Discharge Release -15C
	BQ769x2_SetRegister(UTINTThreshold, (uint8_t)-20, 1);			// Under Temperature Int -20C
	BQ769x2_SetRegister(UTINTDelay, 2, 1);										// Delay 2s
	BQ769x2_SetRegister(UTINTRecovery, (uint8_t)-15, 1);			// Under Temperature Int -15C
	BQ769x2_SetRegister(ProtectionsRecoveryTime, 5, 1);	
	BQ769x2_SetRegister(HWDDelay, 60, 2);
	BQ769x2_SetRegister(LoadDetectActiveTime, 5, 1);
	BQ769x2_SetRegister(LoadDetectRetryDelay, 50, 1);	
	BQ769x2_SetRegister(LoadDetectTimeout, 1, 2);
	BQ769x2_SetRegister(PTOChargeThreshold, 250, 2);
	BQ769x2_SetRegister(PTODelay, 0, 2);	
	BQ769x2_SetRegister(PTOReset, 2, 2);	
	BQ769x2_SetRegister(CUDEPThreshold, 2250, 2);							// Done
	BQ769x2_SetRegister(CUDEPDelay, 2, 1);										// Done
	BQ769x2_SetRegister(SUVThreshold, 2550, 2);								// Done
	BQ769x2_SetRegister(SUVDelay, 5, 1);											// Done
	BQ769x2_SetRegister(SOVThreshold, 3800, 2);								// Done
	BQ769x2_SetRegister(SOVDelay, 5, 1);											// Done
	BQ769x2_SetRegister(TOSSThreshold, 500, 2);								// Done
	BQ769x2_SetRegister(TOSSDelay, 5, 1);											// Done
	BQ769x2_SetRegister(SOCCThreshold, 18000, 2);	
	BQ769x2_SetRegister(SOCCDelay, 5, 1);
	BQ769x2_SetRegister(SOCDThreshold, (uint16_t)-20000, 2);	
	BQ769x2_SetRegister(SOCDDelay, 5, 1);	
	BQ769x2_SetRegister(SOTThreshold, 80, 1);
	BQ769x2_SetRegister(SOTDelay, 5, 1);	
	BQ769x2_SetRegister(SOTFThreshold, 125, 1);	
	BQ769x2_SetRegister(SOTFDelay, 5, 1);		
	BQ769x2_SetRegister(VIMRCheckVoltage, 3500, 2);
	BQ769x2_SetRegister(VIMRMaxRelaxCurrent, 10, 2);	
	BQ769x2_SetRegister(VIMRThreshold, 500, 2);								// Done
	BQ769x2_SetRegister(VIMRDelay, 5, 1);											// Done
	BQ769x2_SetRegister(VIMRRelaxMinDuration, 100, 2);	
	BQ769x2_SetRegister(VIMACheckVoltage, 3700, 2);	
	BQ769x2_SetRegister(VIMAMinActiveCurrent, 50, 2);
	BQ769x2_SetRegister(VIMAThreshold, 200, 2);								// Done
	BQ769x2_SetRegister(VIMADelay, 5, 1);											// Done
	BQ769x2_SetRegister(CFETFOFFThreshold, 20, 2);						// 20
	BQ769x2_SetRegister(CFETFOFFDelay, 5, 1);			
	BQ769x2_SetRegister(DFETFOFFThreshold, (uint16_t)-20, 2);	
	BQ769x2_SetRegister(DFETFOFFDelay, 5, 1);
	BQ769x2_SetRegister(VSSFFailThreshold, 100, 2);	
	BQ769x2_SetRegister(VSSFDelay, 5, 1);	
	BQ769x2_SetRegister(PF2LVLDelay, 5, 1);
	BQ769x2_SetRegister(LFOFDelay, 5, 1);	
	BQ769x2_SetRegister(HWMXDelay, 5, 1);
	BQ769x2_SetRegister(SecuritySettings, 0x01, 1);
	BQ769x2_SetRegister(UnsealKeyStep1, 0x1011, 2);
  BQ769x2_SetRegister(UnsealKeyStep2, 0x2001, 2);
	BQ769x2_SetRegister(FullAccessKeyStep1, 0x4C4F, 2);
  BQ769x2_SetRegister(FullAccessKeyStep2, 0x4E47, 2);
//	BQ769x2_SetRegister(SecuritySettings, 0x00, 1);
//	BQ769x2_SetRegister(UnsealKeyStep1, 0x0414, 2);
//	BQ769x2_SetRegister(UnsealKeyStep2, 0x3672, 2);
//	BQ769x2_SetRegister(FullAccessKeyStep1, 0xFFFF, 2);
//	BQ769x2_SetRegister(FullAccessKeyStep2, 0xFFFF, 2);
	status = CommandSubcommands(EXIT_CFGUPDATE);
  if (status != HAL_OK) {
  }
}

//  ********************************* FET Control Commands  ***************************************
void BQ769x2_BOTHOFF () {
	// Disables all FETs using the DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);    // DFETOFF pin (BOTHOFF) set high
}

void BQ769x2_RESET_BOTHOFF () {
	// Resets DFETOFF (BOTHOFF) pin
	// The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
	HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_ReadFETStatus() {
	HAL_StatusTypeDef status;
  DirectCommands(FETStatus, 0x00, R, &status);
	if (status == HAL_OK) {
  FET_Status = (RX_data[1]*256 + RX_data[0]);
  CHG   		 = (RX_data[0] & 0x01) >> 0;
  PCHG  		 = (RX_data[0] & 0x02) >> 1;
  DSG   		 = (RX_data[0] & 0x04) >> 2;
  PDSG  		 = (RX_data[0] & 0x08) >> 3;
  DCHG_pin 	 = (RX_data[0] & 0x10) >> 4; 
  DDSG_pin 	 = (RX_data[0] & 0x20) >> 5; 
  ALRT_pin 	 = (RX_data[0] & 0x40) >> 6; 
  RSVD_pin 	 = (RX_data[0] & 0x80) >> 7; 
	}
}
// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************
void BQ769x2_ShutdownPin() {
	// Puts the device into SHUTDOWN mode using the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function	
	HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_SET);    // Sets RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin() {
	// Releases the RST_SHUT pin
	// The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function	
	HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_RESET);  // Resets RST_SHUT pin
}
// ********************************* End of BQ769x2 Power Commands   *****************************************

// ********************************* BQ769x2 Status and Fault Commands   *****************************************
void BQ769x2_ReadSafetyStatus() { //good example functions
	// Read Safety Status A/B/C and find which bits are set
	// This shows which primary protections have been triggered
	HAL_StatusTypeDef status;
	DirectCommands(SafetyAlertA, 0x00, R, &status);
	if (status == HAL_OK) value_SafetyAlertA = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(SafetyAlertB, 0x00, R, &status);
	if (status == HAL_OK) value_SafetyAlertB = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(SafetyAlertC, 0x00, R, &status);
	if (status == HAL_OK) value_SafetyAlertC = (RX_data[1]*256 + RX_data[0]);
	DirectCommands(SafetyStatusA, 0x00, R, &status);
	if (status == HAL_OK) value_SafetyStatusA = (RX_data[1]*256 + RX_data[0]);
	UV_Fault 		= ((0x04 & RX_data[0])>>2);
	OV_Fault 		= ((0x08 & RX_data[0])>>3);
	OCC_Fault 	= ((0x10 & RX_data[0])>>4);
	OCD_Fault1 	= ((0x20 & RX_data[0])>>5);
	OCD_Fault 	= ((0x40 & RX_data[0])>>6);
	SCD_Fault 	= ((0x80 & RX_data[0])>>7);
	DirectCommands(SafetyStatusB, 0x00, R, &status);
	if (status == HAL_OK) value_SafetyStatusB = (RX_data[1]*256 + RX_data[0]);
	UTC_Fault 	= ((0x01 & RX_data[0])>>0);
	UTD_Fault 	= ((0x02 & RX_data[0])>>1);
	UTINT_Fault = ((0x04 & RX_data[0])>>2);
	OTC_Fault 	= ((0x10 & RX_data[0])>>4);
	OTD_Fault 	= ((0x20 & RX_data[0])>>5);
	OTINT_Fault = ((0x40 & RX_data[0])>>6);
	OTF_Fault 	= ((0x80 & RX_data[0])>>7);
	DirectCommands(SafetyStatusC, 0x00, R, &status);
	if (status == HAL_OK) value_SafetyStatusC = (RX_data[1]*256 + RX_data[0]);
	HWDF_Fault 	= ((0x02 & RX_data[0])>>1);
	PTO_Fault 	= ((0x04 & RX_data[0])>>2);
	COVL_Fault 	= ((0x10 & RX_data[0])>>4);
	OCDL_Fault 	= ((0x20 & RX_data[0])>>5);
	SCDL_Fault 	= ((0x40 & RX_data[0])>>6);
	OCD3_Fault 	= ((0x80 & RX_data[0])>>7);
	if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 0) {
		ProtectionsTriggered = 1; }
	else {
		ProtectionsTriggered = 0; }
}

void BQ769x2_ReadPFStatus() {
	HAL_StatusTypeDef status;

	DirectCommands(PFStatusA, 0x00, R, &status);
	if (status == HAL_OK) value_PFStatusA = (RX_data[1]*256 + RX_data[0]);
	SUV_PF_Fault 	  = ((0x01 & RX_data[0])>>0);
	SOV_PF_Fault 		= ((0x02 & RX_data[0])>>1);
	SOCC_PF_Fault 	= ((0x04 & RX_data[0])>>2);
	SOCD_PF_Fault 	= ((0x08 & RX_data[0])>>3);
	SOT_PF_Fault 		= ((0x10 & RX_data[0])>>4);
	SOTF_PF_Fault 	= ((0x40 & RX_data[0])>>6);
	CUDEP_PF_Fault 	= ((0x80 & RX_data[0])>>7);

	DirectCommands(PFStatusB, 0x00, R, &status);
	if (status == HAL_OK) value_PFStatusB = (RX_data[1]*256 + RX_data[0]);
	CFETF_PF_Fault 	= ((0x01 & RX_data[0])>>0);
	DFETF_PF_Fault 	= ((0x02 & RX_data[0])>>1);
	LVL2_PF_Fault 	= ((0x04 & RX_data[0])>>2);
	VIMR_PF_Fault 	= ((0x08 & RX_data[0])>>3);
	VIMA_PF_Fault 	= ((0x10 & RX_data[0])>>4);
	SCDL_PF_Fault 	= ((0x80 & RX_data[0])>>7);

	DirectCommands(PFStatusC, 0x00, R, &status);
	if (status == HAL_OK) value_PFStatusC = (RX_data[1]*256 + RX_data[0]);
	OTPF_PF_Fault 	= ((0x01 & RX_data[0])>>0);
	DRMF_PF_Fault 	= ((0x02 & RX_data[0])>>1);
	IRMF_PF_Fault 	= ((0x04 & RX_data[0])>>2);
	LFOF_PF_Fault 	= ((0x08 & RX_data[0])>>3);
	VREF_PF_Fault 	= ((0x10 & RX_data[0])>>4);
	VSSF_PF_Fault 	= ((0x20 & RX_data[0])>>5);
	HWMX_PF_Fault 	= ((0x40 & RX_data[0])>>6);
	CMDF_PF_Fault 	= ((0x80 & RX_data[0])>>7);

	DirectCommands(PFStatusD, 0x00, R, &status);
	if (status == HAL_OK) value_PFStatusD = (RX_data[1]*256 + RX_data[0]);
	TOSF_PF_Fault 	= ((0x01 & RX_data[0])>>0);

	if ((value_PFStatusA + value_PFStatusB + value_PFStatusC + value_PFStatusD) > 0) {
		PFErrorsTriggered = 1; 
	} else {
		PFErrorsTriggered = 0; 
	}
}

void statusread() {
    uint8_t value;
    HAL_StatusTypeDef status = DM_Read8(FETOptions, &value);
    if (status == HAL_OK) {
        value_FETOptions = value;
        SFET        = (value & 0x01) >> 0;
        SLEEPCHG    = (value & 0x02) >> 1;
        HOST_FET_EN = (value & 0x04) >> 2;
        FET_CTRL_EN = (value & 0x08) >> 3;
        PDSG_EN     = (value & 0x10) >> 4;  
        FET_INIT_OFF= (value & 0x20) >> 5;  
        RSVD_01     = (value & 0x40) >> 6;  
        RSVD_00     = (value & 0x80) >> 7;  
    }
}

uint16_t BQ769x2_ReadControlStatus(void)
{
		HAL_StatusTypeDef status;
		DirectCommands(ControlStatus, 0x00, R, &status);
		if (status == HAL_OK) return (RX_data[1]*256 + RX_data[0]);
		return 0;
}

uint16_t BQ769x2_ReadBatteryStatus(void)
{
		HAL_StatusTypeDef status;
		DirectCommands(BatteryStatus, 0x00, R, &status);
		if (status == HAL_OK) {
				battery_status_value = ((uint16_t)RX_data[1] << 8) | (uint16_t)RX_data[0];
				battery_status_sleep = battery_status_value;
				SLEEP = (battery_status_value >> 15) & 0x01;
				return battery_status_value;
		}
		return 0; 
}

uint16_t BQ769x2_ReadAlarmStatusReg(void)
{
		HAL_StatusTypeDef status;
		DirectCommands(AlarmStatus, 0x00, R, &status);
		if (status == HAL_OK) return (RX_data[1]*256 + RX_data[0]);
		return 0;
}

void BQ769x2_ClearLatchedAlerts(void)
{
    uint16_t alarm_status = BQ769x2_ReadAlarmStatusReg();
    if (alarm_status != 0) {
        uint8_t tx[2] = { (uint8_t)alarm_status, (uint8_t)(alarm_status >> 8) };
        HAL_StatusTypeDef status;
        DirectCommands(AlarmStatus, *((uint16_t*)tx), W, &status);
        if (status != HAL_OK) 
					{
        }
    }
}
 
void BQ769x2_PrepareFetOn(void)
{
    HAL_StatusTypeDef status = CommandSubcommands(ALL_FETS_ON);
    if (status == HAL_OK) {
        system_start_time = HAL_GetTick();
        protection_blocked = 0;
    } else 
			{
    }
}

static void BQ769x2_HandleProtection(void)
{
    if (HAL_GetTick() - system_start_time < 3000) return;
    BQ769x2_ReadSafetyStatus();
    if (!occ_software_lock) 
    {
        uint8_t current_occ_bit = (value_SafetyStatusA & MASK_OCC) ? 1 : 0;
        if (current_occ_bit == 1 && prev_occ_bit == 0)  
        {
            occ_streak_count++;                   
            last_occ_timestamp = HAL_GetTick(); 
            if (occ_streak_count >= 10) 
            {
                occ_software_lock = 1; 
            }
        }
        if (occ_streak_count > 0 && current_occ_bit == 0)
        {
            if (HAL_GetTick() - last_occ_timestamp > 60000) 
            {
                occ_streak_count = 0; 
            }
        }
        prev_occ_bit = current_occ_bit; 
    }
		if (occ_software_lock && Pack_Current < -100) 
    {
        occ_software_lock = 0;
        occ_streak_count = 0;
        BQ769x2_ClearLatchedAlerts();
    }
    if (ntc_temp >= 115.0f && !protection_blocked) 
    {
        protection_blocked = 1; 
        mos_ot_lock_tick = HAL_GetTick();
    }
    else if (protection_blocked && ntc_temp <= 85.0f && (HAL_GetTick() - mos_ot_lock_tick >= 30000))
    {
        protection_blocked = 0;
    }	
    if (bms_alarms.Cell_Failure_Alarm) 
    {
        cell_failure_count++;
        if (cell_failure_count >= 5) 
        {
            cell_failure_locked = 1; 
        }
    } 
    else 
    {
        cell_failure_count = 0;
    }
		
		static uint32_t uv_current_stable_tick = 0;
    static uint32_t ov_current_stable_tick = 0;    
    if (Stack_Voltage >= 58400) 
    {
        ov_recovery_locked = 1;
        ov_current_stable_tick = 0;
    }
    else {
    if (Stack_Voltage <= 54000 || SOC < 96.0f) 
        {
            ov_recovery_locked = 0;
        }
		else if (Pack_Current < -200)
        {
            if (ov_current_stable_tick == 0) ov_current_stable_tick = HAL_GetTick();
            if (HAL_GetTick() - ov_current_stable_tick >= 500) {
                ov_recovery_locked = 0;
            }
        }
        else {
            ov_current_stable_tick = 0;
        }
    }
    if (Stack_Voltage <= 43200) 
    {
        uv_recovery_locked = 1;
        uv_current_stable_tick = 0;
    }
    else {
		if (Stack_Voltage >= 47200) 
        {
            uv_recovery_locked = 0;
        }
		else if (Pack_Current > 100)
        {
            if (uv_current_stable_tick == 0) uv_current_stable_tick = HAL_GetTick();
            if (HAL_GetTick() - uv_current_stable_tick >= 500) {
                uv_recovery_locked = 0;
            }
        }
        else {
            uv_current_stable_tick = 0;
        }
    }
    if (occ_software_lock || cell_failure_locked || protection_blocked)
    {
        static uint32_t critical_cmd_tick = 0;
        if (HAL_GetTick() - critical_cmd_tick > 1000) 
        { 
            CommandSubcommands(ALL_FETS_OFF); 
            critical_cmd_tick = HAL_GetTick();
        }
        HAL_GPIO_WritePin(DFETOFF_GPIO_Port, DFETOFF_Pin, GPIO_PIN_SET); 
        last_lock_state = 1;      
        return;
    }
    if (!occ_software_lock && !cell_failure_locked && !protection_blocked) {	
        if (last_lock_state == 1) 
        {
            HAL_GPIO_WritePin(DFETOFF_GPIO_Port, DFETOFF_Pin, GPIO_PIN_RESET);
            last_lock_state = 0;
        }
    }
		if (OCC_Fault || OCD_Fault1 || OCD_Fault || OCDL_Fault || SCD_Fault || SCDL_Fault) 
    {
        HAL_GPIO_WritePin(GPIOA, APTOMAT_SWITCH_Pin, GPIO_PIN_SET); 
    } 
    else 
    {
        HAL_GPIO_WritePin(GPIOA, APTOMAT_SWITCH_Pin, GPIO_PIN_RESET);
    }
}

uint16_t BQ769x2_ReadAlarmRawStatus(void)
{
		HAL_StatusTypeDef status;
		DirectCommands(AlarmRawStatus, 0x00, R, &status);
		if (status == HAL_OK) return (RX_data[1]*256 + RX_data[0]);
		return 0;
}

uint16_t BQ769x2_ReadAlarmEnable(void)
{
		HAL_StatusTypeDef status;
		DirectCommands(AlarmEnable, 0x00, R, &status);
		if (status == HAL_OK) return (RX_data[1]*256 + RX_data[0]);
		return 0;
}

uint16_t BQ769x2_ReadManufacturingStatus(void)
{
    HAL_StatusTypeDef status = Subcommands(MANUFACTURINGSTATUS, 0x00, R);
    if (status == HAL_OK) {
        return (uint16_t)((RX_32Byte[1] << 8) | RX_32Byte[0]);
    }
    return 0;
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************

// ********************************* BQ769x2 Measurement Commands   *****************************************
uint16_t BQ769x2_ReadVoltage(uint8_t command, HAL_StatusTypeDef *ptr_status)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	HAL_StatusTypeDef status = I2C_ReadReg_WithRetry(command, RX_data, 2);
  if (ptr_status) *ptr_status = status;
	if (status == HAL_OK) {
        if (command >= Cell1Voltage && command <= Cell16Voltage) {
            return (RX_data[1]*256 + RX_data[0]);  // mV
        } else {
            return 10 * (RX_data[1]*256 + RX_data[0]);  // 0.01V → mV
        }
    }
    return 0;  
}

static uint32_t Average_U16(const uint16_t buf[AVERAGE_SAMPLES])
{
    uint32_t sum = 0;
    for (int i = 0; i < AVERAGE_SAMPLES; i++) sum += buf[i];
    return sum / AVERAGE_SAMPLES;
}

static int32_t Average_I16(const int16_t buf[AVERAGE_SAMPLES_CURRENT])
{
    int32_t sum = 0;
    for (int i = 0; i < AVERAGE_SAMPLES_CURRENT; i++) sum += buf[i];
    return sum / AVERAGE_SAMPLES_CURRENT;
}

void BQ769x2_ReadAllVoltages(void)
{
    HAL_StatusTypeDef status;
    int cellvoltageholder = Cell1Voltage;
    for (int i = 0; i < 16; i++) {
        uint16_t raw = BQ769x2_ReadVoltage(cellvoltageholder, &status);
        if (status == HAL_OK) {
						if (raw > 6500) { 
								raw = 0; 
								for (int j = 0; j < AVERAGE_SAMPLES; j++) {
										cell_buffer[i][j] = 0;
								}
						}
            if (first_sample_after_reset) {
                for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                    cell_buffer[i][j] = raw;
                }
            } else {
                cell_buffer[i][avg_index] = raw;
            }
            CellVoltage[i] = (uint16_t)Average_U16(cell_buffer[i]);
        }
        cellvoltageholder += 2;
    }
    uint16_t raw_stack = BQ769x2_ReadVoltage(StackVoltage, &status);
    if (status == HAL_OK) {
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                stack_buffer[j] = raw_stack;
            }
        } else {
            stack_buffer[avg_index] = raw_stack;
        }
        Stack_Voltage = (uint16_t)Average_U16(stack_buffer);
    }
    uint16_t raw_pack = BQ769x2_ReadVoltage(PACKPinVoltage, &status);
    if (status == HAL_OK) {
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                pack_buffer[j] = raw_pack;
            }
        } else {
            pack_buffer[avg_index] = raw_pack;
        }
        Pack_Voltage = (uint16_t)Average_U16(pack_buffer);
    }
    uint16_t raw_ld = BQ769x2_ReadVoltage(LDPinVoltage, &status);
    if (status == HAL_OK) {
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES; j++) {
                ld_buffer[j] = raw_ld;
            }
        } else {
            ld_buffer[avg_index] = raw_ld;
        }
        LD_Voltage = (uint16_t)Average_U16(ld_buffer);
    }
    avg_index = (avg_index + 1) % AVERAGE_SAMPLES;
    sum_voltage = 0;
    for (int i = 0; i < 16; i++) {
        sum_voltage += CellVoltage[i];
    }
}

int16_t BQ769x2_ReadCurrent(void)
{
    HAL_StatusTypeDef status;
    DirectCommands(CC2Current, 0x00, R, &status);  
    if (status == HAL_OK) {
        int16_t raw = (int16_t)((RX_data[1] << 8) | RX_data[0]);
        if (first_sample_after_reset) {
            for (int j = 0; j < AVERAGE_SAMPLES_CURRENT; j++) {
                current_buffer[j] = raw;
            }
        } else {
            current_buffer[avg_index_current] = raw;
        }
        avg_index_current = (avg_index_current + 1) % AVERAGE_SAMPLES_CURRENT;
        return (int16_t)Average_I16(current_buffer);
    }
    return 0;
}

uint16_t AFE_ReadCurrent(void)
{
    HAL_StatusTypeDef status = I2C_ReadReg_WithRetry(0x3A, RX_2Byte, 2);
    if (status == HAL_OK) {
        return (RX_2Byte[1]*256 + RX_2Byte[0]); 
    }
    return 0;
}

float BQ769x2_ReadTemperature(uint8_t command)
{
    HAL_StatusTypeDef status;
    DirectCommands(command, 0x00, R, &status);
    if (status == HAL_OK) {
        return (0.1f * (float)(RX_data[1]*256 + RX_data[0])) - 273.15f; 
    }
    return 25.0f;
}

float Get_PackVoltage_V(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < AVERAGE_SAMPLES_ADC; i++) {
        sum += adc_buffer[0][i];
    }
    uint16_t adc_raw = (uint16_t)(sum / AVERAGE_SAMPLES_ADC);
    float voltage_divided = (adc_raw * 3.3f) / 4095.0f;
    pack_voltage_adc = voltage_divided * 20.0f;
    return pack_voltage_adc;
}

float Get_ExternalNTCTemp_C(uint8_t sensor_idx)
{
    if (sensor_idx < 1 || sensor_idx > 3) return 25.0f;
    uint32_t sum = 0;
    for (int i = 0; i < AVERAGE_SAMPLES_ADC; i++) {
        sum += adc_buffer[sensor_idx][i];
    }
    uint16_t adc_raw = (uint16_t)(sum / AVERAGE_SAMPLES_ADC);
    if (adc_raw < 100) return 25.0f;
    return NTC_AdcToTemp(adc_raw);
}

void BQ769x2_ReadPassQ(void) 
{
		memset(RX_32Byte, 0, 32); 
    HAL_StatusTypeDef status = Subcommands(DASTATUS6, 0x00, R);
    if (status == HAL_OK) {
		AccumulatedCharge_Int  = ((int32_t)RX_32Byte[3]<<24) | ((int32_t)RX_32Byte[2]<<16) | ((int32_t)RX_32Byte[1]<<8) | (int32_t)RX_32Byte[0]; 	 		 //Bytes 0-3
	  if (AccumulatedCharge_Int == -32767 || (uint32_t)AccumulatedCharge_Int == 0xFFFFFFFF) return;
		AccumulatedCharge_Frac = ((uint32_t)RX_32Byte[7]<<24) | ((uint32_t)RX_32Byte[6]<<16) | ((uint32_t)RX_32Byte[5]<<8) | (uint32_t)RX_32Byte[4]; 	 //Bytes 4-7
		AccumulatedCharge_Time = ((uint32_t)RX_32Byte[11]<<24) | ((uint32_t)RX_32Byte[10]<<16) | ((uint32_t)RX_32Byte[9]<<8) | (uint32_t)RX_32Byte[8]; //Bytes 8-11
    double total_userAh = (double)AccumulatedCharge_Int + (double)AccumulatedCharge_Frac / 4294967296.0f;
    passed_charge_mAh = (float)(total_userAh * 9.95f);
		}
}

uint32_t Calculate_Device_Hash(void) {
    uint32_t uid[3];
    uid[0] = *(uint32_t*)0x1FFFF7E8;
    uid[1] = *(uint32_t*)0x1FFFF7EC;
    uid[2] = *(uint32_t*)0x1FFFF7F0;

		uint32_t secret_dob = 10112001;
		uint32_t secret_name = 0x4C4F4E47;
	
    uint32_t hash = uid[0] ^ uid[1] ^ uid[2] ^ secret_dob ^ secret_name;
    hash = (hash << 5) | (hash >> 27);     
    return hash;
}

uint32_t calculate_crc32(uint8_t *data, uint32_t length);
uint8_t w25qxx_spi_init(void);
uint8_t w25qxx_spi_deinit(void);
uint8_t w25qxx_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                   uint32_t address, uint8_t address_line, uint8_t address_len,
                                   uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                   uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                   uint8_t *out_buf, uint32_t out_len, uint8_t data_line);
void flash_debug_print(const char *const fmt, ...);
uint8_t BMS_Flash_Init(void);
uint8_t BMS_Save_Data_To_Flash(void);
uint8_t BMS_Load_Data_From_Flash(void);
void Flash_Error_Indication(uint8_t error_code);

void Update_Cycle_Count(void)
{
    static float last_passed_charge = 0.0f; 
    BQ769x2_ReadPassQ();  
    if (last_passed_charge == 0.0f) {
        last_passed_charge = passed_charge_mAh;
        return;
    }
    delta_Charge_mAh = passed_charge_mAh - last_passed_charge;
    last_passed_charge = passed_charge_mAh;
    if (delta_Charge_mAh < -0.01f) { 
        accumulated_discharge_mAh += fabs(delta_Charge_mAh); 
    }
    float threshold = 0.8f * NOMINAL_CAPACITY_mAh; 
    if (accumulated_discharge_mAh >= threshold) {
        uint32_t new_cycles = (uint32_t)(accumulated_discharge_mAh / threshold);
        cycle_count += new_cycles;
        accumulated_discharge_mAh -= (float)new_cycles * threshold;
        if (flash_init_success) {
            BMS_Save_Data_To_Flash();
        }
        DWIN_SendCycle((float)cycle_count);
    }
}

float get_soc_from_ocv(float v_cell)
{
		const float ocv_soc_table_101[101] = {
		2.8500f, 2.8800f, 2.9100f, 2.9400f, 2.9700f, 3.0000f, 3.0200f, 3.0400f, 3.0600f, 3.0800f, // 0% - 9%
    3.1000f, 3.1200f, 3.1400f, 3.1600f, 3.1800f, 3.2000f, 3.2080f, 3.2160f, 3.2240f, 3.2320f, // 10% - 19%
    3.2400f, 3.2420f, 3.2440f, 3.2460f, 3.2480f, 3.2500f, 3.2520f, 3.2540f, 3.2560f, 3.2580f, // 20% - 29%
    3.2600f, 3.2620f, 3.2640f, 3.2660f, 3.2680f, 3.2700f, 3.2720f, 3.2740f, 3.2760f, 3.2780f, // 30% - 39%
    3.2800f, 3.2810f, 3.2820f, 3.2830f, 3.2840f, 3.2850f, 3.2860f, 3.2870f, 3.2880f, 3.2890f, // 40% - 49%
    3.2900f, 3.2910f, 3.2920f, 3.2930f, 3.2940f, 3.2950f, 3.2960f, 3.2970f, 3.2980f, 3.2990f, // 50% - 59%
    3.3000f, 3.3010f, 3.3020f, 3.3030f, 3.3040f, 3.3050f, 3.3060f, 3.3070f, 3.3080f, 3.3090f, // 60% - 69%
    3.3100f, 3.3110f, 3.3120f, 3.3130f, 3.3140f, 3.3150f, 3.3160f, 3.3170f, 3.3180f, 3.3190f, // 70% - 79%
    3.3200f, 3.3210f, 3.3220f, 3.3230f, 3.3240f, 3.3250f, 3.3260f, 3.3270f, 3.3280f, 3.3290f, // 80% - 89%
    3.3300f, 3.3340f, 3.3380f, 3.3420f, 3.3460f, 3.3500f, 3.3600f, 3.3700f, 3.3800f, 3.3900f, // 90% - 99%
    3.4000f                                                                                   // 100%																																																																		// 100%
		};
    if (v_cell <= ocv_soc_table_101[0])  return 0.0f;
    if (v_cell >= ocv_soc_table_101[100]) return 100.0f;
    for (int i = 0; i < 100; i++) {
        if (v_cell >= ocv_soc_table_101[i] && v_cell <= ocv_soc_table_101[i+1]) {
            float v1 = ocv_soc_table_101[i];
            float v2 = ocv_soc_table_101[i+1];
            float soc1 = (float)i;
            float soc2 = (float)(i + 1);
            return soc1 + (v_cell - v1) * (soc2 - soc1) / (v2 - v1);
        }
    }
    return 50.0f;
}

void Update_SOC_SOH_FromBQ(void)
{
    BQ769x2_ReadPassQ();
    BQ769x2_ReadSafetyStatus();
    bool is_bad_cell_condition = bms_alarms.Cell_Failure_Alarm || cell_failure_locked;
	
    if (Pack_Current > 500) {
        if (!in_significant_charge) {
            soc_at_charge_start = SOC;
            significant_charge_start_tick = HAL_GetTick();
            in_significant_charge = true;
        }
    } 
    else if (Pack_Current < -200) {
        in_significant_charge = false;
    }
    else if (abs(Pack_Current) < 10) {
        if (charge_stop_tick == 0) charge_stop_tick = HAL_GetTick();
        if (HAL_GetTick() - charge_stop_tick > 600000) {
            in_significant_charge = false;
        }
    } else {
        charge_stop_tick = 0;
    }
		
		static uint8_t last_uv = 0;
    if ((UV_Fault || uv_recovery_locked) && !last_uv)
    {
        if (!is_bad_cell_condition) 
        {
            SOC = 0.0f;
            SOC_Capa = 0.0f;
            CommandSubcommands(RESET_PASSQ);					
            passed_charge_mAh = 0.0f;
            mAh_at_last_sync = FullChargeCapacity_mAh;
            currentUsage = FullChargeCapacity_mAh;
						remain_capa_Ah = (SOC / 100.0f) * (FullChargeCapacity_mAh / 1000.0f);
            DWIN_SendReCapa(remain_capa_Ah);					
            if (flash_init_success) BMS_Save_Data_To_Flash();
        }
        last_uv = 1;
    }
    else if (!(UV_Fault || uv_recovery_locked) && last_uv)
    {
        last_uv = 0;
    }
		
		static uint8_t last_ov = 0;
    if ((OV_Fault || ov_recovery_locked) && !last_ov)
    {
        if (!is_bad_cell_condition)
        {
            SOC = 100.0f;
            SOC_Capa = 100.0f;
            CommandSubcommands(RESET_PASSQ);					
            passed_charge_mAh = 0.0f;
            mAh_at_last_sync = 0.0f;
            currentUsage = 0.0f;
						remain_capa_Ah = (SOC / 100.0f) * (FullChargeCapacity_mAh / 1000.0f);
            DWIN_SendReCapa(remain_capa_Ah);
            if (flash_init_success) BMS_Save_Data_To_Flash();						
        }
        last_ov = 1;
    }
    else if (!(OV_Fault || ov_recovery_locked) && last_ov)
    {
        last_ov = 0;
    }
		
    if (!UV_Fault && !OV_Fault && !uv_recovery_locked && !ov_recovery_locked) {
        float delta_mAh = mAh_at_last_sync - passed_charge_mAh;
        currentUsage = fmaxf(0.0f, fminf(FullChargeCapacity_mAh * 1.13f, delta_mAh));
        float remaining_mAh = FullChargeCapacity_mAh - currentUsage;
        SOC_Capa = (remaining_mAh / FullChargeCapacity_mAh) * 100.0f;
        if (fabs(SOC - SOC_Capa) > 0.01f) {
            SOC += (SOC_Capa - SOC) * 0.05f;
        }
        SOC = fmaxf(0.0f, fminf(100.0f, SOC));
    }
		
    static uint32_t taper_timer = 0;
    bool is_taper_condition = (Stack_Voltage >= 56000) && (Pack_Current > 0) && (Pack_Current <= TAPER_CURRENT_mA);
    if (is_taper_condition) {
        taper_timer++;
        if (taper_timer >= 120) {
            bool cycle_valid = (soc_at_charge_start <= 30.0f) || (HAL_GetTick() - significant_charge_start_tick >= 1800000UL);
            if (cycle_valid && !is_bad_cell_condition) {
                float measured_capa = FullChargeCapacity_mAh - currentUsage;
                float capacity_error = measured_capa - FullChargeCapacity_mAh;
                float learning_rate = (flash_status != FLASH_OK) ? 0.05f : 0.01f;
                FullChargeCapacity_mAh += capacity_error * learning_rate;
                FullChargeCapacity_mAh = fmaxf(NOMINAL_CAPACITY_mAh * 0.60f, fminf(NOMINAL_CAPACITY_mAh * 1.15f, FullChargeCapacity_mAh));
            }
            float soh_from_capa = (FullChargeCapacity_mAh / NOMINAL_CAPACITY_mAh) * 100.0f;
            uint16_t min_cell_at_full = CellVoltage[0];
            for (int i = 1; i < CELL_COUNT; i++) {
                if (CellVoltage[i] < min_cell_at_full) min_cell_at_full = CellVoltage[i];
            }
            float min_v = min_cell_at_full / 1000.0f;
            float soh_from_ocv = (min_v / 3.4f) * 100.0f;
            soh_from_ocv = fmaxf(80.0f, fminf(100.0f - (cycle_count / 8000.0f * 20.0f), soh_from_ocv));
            float ocv_weight = (cycle_count < 500) ? 0.10f : 0.20f;
            float combined_soh_input = ((1.0f - ocv_weight) * soh_from_capa) + (ocv_weight * soh_from_ocv);
            SOH = kalman_filter_update(&kf_soh, combined_soh_input, 2.0f);
            SOH = fmaxf(60.0f, fminf(100.0f, SOH));
            float aging_capacity = (SOH / 100.0f) * NOMINAL_CAPACITY_mAh;
            if (SOH < 98.0f) {
                FullChargeCapacity_mAh = fminf(FullChargeCapacity_mAh, aging_capacity);
            }
            SOC = 100.0f;
            SOC_Capa = 100.0f;
            CommandSubcommands(RESET_PASSQ);
            passed_charge_mAh = 0.0f;
            mAh_at_last_sync = 0.0f;
            currentUsage = 0.0f;
            remain_capa_Ah = (SOC / 100.0f) * (FullChargeCapacity_mAh / 1000.0f);
            DWIN_SendReCapa(remain_capa_Ah);
            DWIN_SendSOH(SOH);
            if (flash_init_success) {
                BMS_Save_Data_To_Flash();
                flash_status = FLASH_OK; 
            }
            soc_at_charge_start = 100.0f;
            in_significant_charge = false;
            taper_timer = 0;
        }
    } else {
        taper_timer = 0;
    }
    if (adv_soc_clamp_active && SOC > 20.0f) 
    {
        SOC -= 0.05f; 
        if (SOC <= 20.0f) SOC = 20.0f;
        SOC_Capa = SOC; 
        currentUsage = FullChargeCapacity_mAh * (1.0f - (SOC / 100.0f));
    }

    if (adv_stop_discharge_active && SOC > 20.0f)
    {
        SOC = 20.0f;
        SOC_Capa = SOC;
        currentUsage = FullChargeCapacity_mAh * (1.0f - (SOC / 100.0f));
    }
    wattUsage = currentUsage * (Stack_Voltage / 1000.0f);
    float max_energy_mWh = FullChargeCapacity_mAh * (Stack_Voltage / 1000.0f);
    wattUsage = fmaxf(0.0f, fminf(max_energy_mWh / 1000.0f, wattUsage));
}

void BMS_Refresh_Current_Buffer(void) {
    int16_t raw_current = BQ769x2_ReadCurrent(); 
    for (int j = 0; j < AVERAGE_SAMPLES_CURRENT; j++) {
        current_buffer[j] = raw_current;
    }
    Pack_Current = raw_current;
}

void BQ769x2_OTP_STATUS(void) {
    HAL_StatusTypeDef status = Subcommands(OTP_WR_CHECK, 0x00, R);
    if (status == HAL_OK) {
        OTP = ((uint64_t)RX_32Byte[7] << 56) | ((uint64_t)RX_32Byte[6] << 48) |
              ((uint64_t)RX_32Byte[5] << 40) | ((uint64_t)RX_32Byte[4] << 32) |
              ((uint64_t)RX_32Byte[3] << 24) | ((uint64_t)RX_32Byte[2] << 16) |
              ((uint64_t)RX_32Byte[1] << 8) | RX_32Byte[0];
    }
}

void BQ769x2_OTP_SCAN(){
		HAL_StatusTypeDef status;
		DirectCommands(BatteryStatus, 0x00, R, &status);
		OTP_Status = (RX_data[1]*256 + RX_data[0]);
}

float Calculate_RemainEnergy(float SOC, float SOH)
{
    float FSOC = SOC / 100.0f;
    float FSOH = SOH / 100.0f;
    const float NomCapa = NOMINAL_CAPACITY_mAh * 0.001f;
    remain_capa_Ah = FSOC * FSOH * NomCapa;
    const float NomVol = 51.2f;
    return (remain_capa_Ah * NomVol) / 1000.0f;
}

uint8_t BMS_Get_Address_ID(void) {
    uint8_t group_m = 0;
    if (HAL_GPIO_ReadPin(GPIOE, K2_Pin) == GPIO_PIN_RESET) group_m |= (1 << 0); 
    if (HAL_GPIO_ReadPin(GPIOE, K3_Pin) == GPIO_PIN_RESET) group_m |= (1 << 1); 
    if (HAL_GPIO_ReadPin(GPIOE, K4_Pin) == GPIO_PIN_RESET) group_m |= (1 << 2); 
    return group_m;
}

void BMS_Init_Address(void) {
    uint8_t dip_k1 = (HAL_GPIO_ReadPin(GPIOE, K1_Pin) == GPIO_PIN_RESET);
    m = BMS_Get_Address_ID();
    can_id_offset = 0x1000 * m;
    uint32_t target_baud = (dip_k1) ? 9600 : 115200;
    
    if (huart5.Init.BaudRate != target_baud) {
        HAL_UART_DeInit(&huart5);
        huart5.Init.BaudRate = target_baud;
        if (HAL_UART_Init(&huart5) != HAL_OK) { Error_Handler(); }
        HAL_UART_Receive_IT(&huart5, (uint8_t *)&rx_byte_u5, 1);
    }
		if (m == 0) {
        if (pylon_protocol_addr >= 0x02 && pylon_protocol_addr <= 0x12) {
            is_auto_coding = 0;
            auto_code_state = AUTO_CODE_DONE;
        } else {
            is_auto_coding = 1;
            auto_code_state = AUTO_CODE_START;
            is_master = 0;
            pylon_protocol_addr = 0xFF;
        }
    } else {
        is_auto_coding = 0;
        auto_code_state = AUTO_CODE_DONE;      
        if (m == 1) {
            is_master = 1;
            pylon_protocol_addr = 0x02;
        } else { 
            is_master = 0;
            pylon_protocol_addr = 0x02 + (m - 1);
        }
    }
    SET_DN_OP_LOW(); 
    Safe_Delay_ms(100);
}

void BMS_Auto_Coding_Task(void) { 
    if (!is_auto_coding || auto_code_state == AUTO_CODE_DONE) return;

    static uint32_t state_timeout_tick = 0;
    static uint32_t last_send_tick = 0;
    static uint32_t up_in_stable_tick = 0;
    HAL_IWDG_Refresh(&hiwdg);
    uint8_t up_in_active = (READ_UP_IN() == GPIO_PIN_SET);

    switch (auto_code_state) {
        case AUTO_CODE_START: 
            is_master = 0;
            assigned_n = 0;
            pylon_protocol_addr = 0xFF; 
            SET_DN_OP_LOW();            
            state_timeout_tick = HAL_GetTick();
            up_in_stable_tick = HAL_GetTick();
            auto_code_state = AUTO_CODE_WAIT_UP_IN;
            break;

        case AUTO_CODE_WAIT_UP_IN:
            if (up_in_active) {
                if (HAL_GetTick() - up_in_stable_tick > 500) { 
                    auto_code_state = AUTO_CODE_WAIT_ADDRESS_CMD;
                    state_timeout_tick = HAL_GetTick(); 
                }
            } else {
                if (HAL_GetTick() - state_timeout_tick > 3000) {
                    auto_code_state = AUTO_CODE_IS_MASTER;
                }
                up_in_stable_tick = HAL_GetTick();
            }
            break;

        case AUTO_CODE_IS_MASTER:
            is_master = 1;
            pylon_protocol_addr = 0x02;
            SET_DN_OP_HIGH(); 
            memset(slave_online_status, 0, sizeof(slave_online_status));
            active_packs_count = 1; 
            current_assigning_index = 3; 
            state_timeout_tick = HAL_GetTick();
            auto_code_state = AUTO_CODE_MASTER_ASSIGNING;
            break;

        case AUTO_CODE_MASTER_ASSIGNING:
            if (HAL_GetTick() - last_send_tick >= 500) {
                Pylon_Send_Assign_ID(&huart5, current_assigning_index);
                last_send_tick = HAL_GetTick();
            }

            if (slave_online_status[current_assigning_index - 3] > 0) {
                current_assigning_index++;
                active_packs_count++;
                state_timeout_tick = HAL_GetTick();               
                if (current_assigning_index > (0x02 + MAX_SLAVES)) auto_code_state = AUTO_CODE_DONE;
            }
            if (HAL_GetTick() - state_timeout_tick > 5000) {
                auto_code_state = AUTO_CODE_DONE; 
            }
            break;

        case AUTO_CODE_WAIT_ADDRESS_CMD:
            if (assigned_n >= 3) {
                is_master = 0;
                pylon_protocol_addr = assigned_n;
                SET_DN_OP_HIGH();               
                auto_code_state = AUTO_CODE_DONE;
            }
            if (HAL_GetTick() - state_timeout_tick > 15000) {
                auto_code_state = AUTO_CODE_DONE; 
            }
            break;

        case AUTO_CODE_DONE:
            is_auto_coding = 0; 
            break;

        default:
            auto_code_state = AUTO_CODE_DONE;
            break;
    }
}					
// ********************************* End of BQ769x2 Measurement Commands   *****************************************
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if (IS_BOOTLOADER == 0)
static uint32_t Get_Day_Stamp(void)
{
    DS1307_TIME t;
    DS1307_GetTime(&t);
    uint32_t days = (t.year - 0) * 365U;     
    days += (t.month - 1) * 30U;             
    days += t.date - 1;                      
    return days;
}

static uint8_t wait_flash_ready(uint32_t timeout_loops) {
    uint32_t loop_count = 0;
    uint8_t status;
    do {
        if (w25qxx_get_status1(&flash_handle, &status) != 0) {
            return 1; 
        }
        if ((status & 0x01) == 0) {
            return 0; 
        }
        Safe_Delay_ms(1); 
        loop_count++;
    } while (loop_count < timeout_loops); 
    return 1; 
}

uint32_t calculate_crc32(uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

void flash_debug_print(const char *const fmt, ...) {
}

void Flash_Error_Indication(uint8_t error_code) {
    switch (error_code) {
        case FLASH_OK:
            LED_SetMode(TEST_LED_GPIO_Port, TEST_LED_Pin, FLASH_MODE_ON);
            break;
        case FLASH_INIT_ERROR:
						LED_SetMode(TEST_LED_GPIO_Port, TEST_LED_Pin, FLASH_MODE_1);
            break;
        case FLASH_READ_ERROR:
						LED_SetMode(TEST_LED_GPIO_Port, TEST_LED_Pin, FLASH_MODE_2);
            break;
        case FLASH_WRITE_ERROR:
						LED_SetMode(TEST_LED_GPIO_Port, TEST_LED_Pin, FLASH_MODE_3);
            break;
        case FLASH_CRC_ERROR:
						HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
						Safe_Delay_ms(50);
            break;
        case FLASH_NO_DATA:
            LED_SetMode(TEST_LED_GPIO_Port, TEST_LED_Pin, FLASH_MODE_OFF);
            break;
        default:
						LED_SetMode(TEST_LED_GPIO_Port, TEST_LED_Pin, FLASH_MODE_OFF);
            break;
    }
}

uint8_t w25qxx_spi_init(void) {
    return 0;
}

uint8_t w25qxx_spi_deinit(void) {
    return 0;
}

uint8_t w25qxx_spi_qspi_write_read(uint8_t  instruction,
                                   uint8_t  instruction_line,
                                   uint32_t address,
                                   uint8_t  address_line,
                                   uint8_t  address_len,
                                   uint32_t alternate,
                                   uint8_t  alternate_line,
                                   uint8_t  alternate_len,
                                   uint8_t  dummy,
                                   uint8_t *in_buf,
                                   uint32_t in_len,
                                   uint8_t *out_buf,
                                   uint32_t out_len,
                                   uint8_t  data_line) {
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    delayUS(10);
    if (instruction != 0x00) {
        status = HAL_SPI_Transmit(&hspi2, &instruction, 1, 1000);
        if (status != HAL_OK) goto error;
    }
    if (address_len > 0) {
        uint8_t addr_buf[4];
        for (int i = address_len - 1; i >= 0; i--) {
            addr_buf[i] = (address >> (8 * (address_len - 1 - i))) & 0xFF;
        }
        status = HAL_SPI_Transmit(&hspi2, addr_buf, address_len, 1000);
        if (status != HAL_OK) goto error;
    }
    if (alternate_len > 0) {
        uint8_t alt_buf[4];
        for (int i = alternate_len - 1; i >= 0; i--) {
            alt_buf[i] = (alternate >> (8 * (alternate_len - 1 - i))) & 0xFF;
        }
        status = HAL_SPI_Transmit(&hspi2, alt_buf, alternate_len, 1000);
        if (status != HAL_OK) goto error;
    }
    if (dummy > 0) {
        uint8_t dummy_byte  = 0xFF;
        uint8_t dummy_count = (dummy + 7) / 8;
        for (uint8_t i = 0; i < dummy_count; i++) {
            status = HAL_SPI_Transmit(&hspi2, &dummy_byte, 1, 1000);
            if (status != HAL_OK) goto error;
        }
    }
    if (in_len > 0 && in_buf != NULL) {
        status = HAL_SPI_Transmit(&hspi2, in_buf, in_len, 1000);
        if (status != HAL_OK) goto error;
    }
    if (out_len > 0 && out_buf != NULL) {
        status = HAL_SPI_Receive(&hspi2, out_buf, out_len, 1000);
        if (status != HAL_OK) goto error;
    }
    delayUS(10);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return 0;
error:
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return 1;
}
																	 
uint8_t BMS_Flash_Init(void) 
{
    uint8_t res;
    uint8_t manufacturer = 0;
    uint8_t device_id[2] = {0};
    uint8_t status1 = 0;
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_RESET);
		
    DRIVER_W25QXX_LINK_INIT(&flash_handle, w25qxx_handle_t);
    DRIVER_W25QXX_LINK_SPI_QSPI_INIT(&flash_handle, w25qxx_spi_init);
    DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(&flash_handle, w25qxx_spi_deinit);
    DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(&flash_handle, w25qxx_spi_qspi_write_read);
    DRIVER_W25QXX_LINK_DELAY_MS(&flash_handle, Safe_Delay_ms);
    DRIVER_W25QXX_LINK_DELAY_US(&flash_handle, delayUS);
    DRIVER_W25QXX_LINK_DEBUG_PRINT(&flash_handle, flash_debug_print);

    res = w25qxx_set_type(&flash_handle, W25Q64);
    if (res != 0) {
        Flash_Error_Indication(FLASH_INIT_ERROR);
        return FLASH_INIT_ERROR;
    }
    res = w25qxx_set_interface(&flash_handle, W25QXX_INTERFACE_SPI);
    if (res != 0) {
        Flash_Error_Indication(FLASH_INIT_ERROR);
        return FLASH_INIT_ERROR;
    }
    res = w25qxx_set_dual_quad_spi(&flash_handle, W25QXX_BOOL_FALSE);
    if (res != 0) {
        Flash_Error_Indication(FLASH_INIT_ERROR);
        return FLASH_INIT_ERROR;
    }
    res = w25qxx_init(&flash_handle);
    if (res != 0) {
        Flash_Error_Indication(FLASH_INIT_ERROR);
        return FLASH_INIT_ERROR;
    }
    w25qxx_enable_reset(&flash_handle);
    w25qxx_reset_device(&flash_handle);
    Safe_Delay_ms(100);
    res = w25qxx_get_jedec_id(&flash_handle, &manufacturer, device_id);
    if (res != 0 || manufacturer != 0xEF || device_id[0] != 0x40 || device_id[1] != 0x17) {
        Flash_Error_Indication(FLASH_INIT_ERROR);
        return FLASH_INIT_ERROR;
    }
		
    w25qxx_enable_write(&flash_handle); 
    w25qxx_set_status1(&flash_handle, 0x00);
    w25qxx_set_status2(&flash_handle, 0x00);
    Safe_Delay_ms(20);

    w25qxx_get_status1(&flash_handle, &status1);
    if ((status1 & 0x7C) != 0) {  
        Flash_Error_Indication(FLASH_INIT_ERROR);
        return FLASH_INIT_ERROR;
    }
    flash_init_success = 1;
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_SET);
    return FLASH_OK;
}

static uint8_t restore_data_from_flash(const bms_flash_data_t *data) {
    uint32_t calc_crc = calculate_crc32((uint8_t*)data, sizeof(bms_flash_data_t) - sizeof(uint32_t));
    if (calc_crc != data->crc ||
        data->soc_x100 > 10500 ||
        data->soh_x100 > 10500) {
        return FLASH_CRC_ERROR;
    }
    SOC = data->soc_x100 / 100.0f;
    SOH = data->soh_x100 / 100.0f;
    cycle_count = data->cycle_count;
		if (data->full_capa_mAh < 300000.0f) { 
        FullChargeCapacity_mAh = NOMINAL_CAPACITY_mAh;
    } else {
        FullChargeCapacity_mAh = data->full_capa_mAh;
    }
		current_sequence = data->sequence_number;
		last_full_charge_day = data->last_full_charge_day;
		mAh_at_last_sync = data->mAh_at_last_sync;
		accumulated_discharge_mAh = data->accumulated_discharge_mAh;
		memcpy(current_pf_status, data->pf_status, 4);
		saved_device_hash   = data->device_hash;
		pylon_protocol_addr = data->pylon_protocol_addr;
    is_master = data->is_master;
    last_saved_soc_x100 = data->soc_x100;
    last_saved_soh_x100 = data->soh_x100;
    last_saved_cycle = data->cycle_count;
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_SET);
    Safe_Delay_ms(300);
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_RESET);
    return FLASH_OK;
}

uint8_t BMS_Save_Data_To_Flash(void)
{
    if (!flash_init_success) {
        return FLASH_INIT_ERROR;
    }
    current_sequence++; 
    bms_flash_data_t flash_data = {
        .soc_x100 = (uint16_t)roundf(SOC * 100.0f),
        .soh_x100 = (uint16_t)roundf(SOH * 100.0f),
        .cycle_count = cycle_count,
        .full_capa_mAh = FullChargeCapacity_mAh,
				.last_full_charge_day = last_full_charge_day,
				.pf_status = {current_pf_status[0], current_pf_status[1], current_pf_status[2], current_pf_status[3]},
        .sequence_number = current_sequence,
				.mAh_at_last_sync = mAh_at_last_sync,
				.accumulated_discharge_mAh = accumulated_discharge_mAh,
				.device_hash = Calculate_Device_Hash(),
				.pylon_protocol_addr = pylon_protocol_addr,
        .is_master = is_master
    };
    flash_data.crc = calculate_crc32((uint8_t*)&flash_data, sizeof(bms_flash_data_t) - sizeof(uint32_t));
    uint32_t sector_index = current_wear_slot;
    uint32_t write_addr = WEAR_LEVEL_START_ADDR + (sector_index * FLASH_SECTOR_SIZE);
		if (wait_flash_ready(1000) != 0) {
        Flash_Error_Indication(FLASH_WRITE_ERROR);
        return FLASH_WRITE_ERROR;
    }
    if (w25qxx_sector_erase_4k(&flash_handle, write_addr) != 0) {
        Flash_Error_Indication(FLASH_WRITE_ERROR);
        return FLASH_WRITE_ERROR;
    }
    if (wait_flash_ready(1000) != 0) {
        Flash_Error_Indication(FLASH_WRITE_ERROR);
        return FLASH_WRITE_ERROR;
    }
    if (w25qxx_write(&flash_handle, write_addr, (uint8_t*)&flash_data, sizeof(bms_flash_data_t)) != 0) {
        Flash_Error_Indication(FLASH_WRITE_ERROR);
        return FLASH_WRITE_ERROR;
    }
    if (wait_flash_ready(100) != 0) {
        Flash_Error_Indication(FLASH_WRITE_ERROR);
        return FLASH_WRITE_ERROR;
    }
    current_wear_slot = (current_wear_slot + 1) % WEAR_LEVEL_SECTORS;
    last_saved_soc_x100 = flash_data.soc_x100;
    last_saved_soh_x100 = flash_data.soh_x100;
    last_saved_cycle = cycle_count;
    flash_save_count++;
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_SET);
    Safe_Delay_ms(100);
    HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_RESET);
    return FLASH_OK;
}

uint8_t BMS_Load_Data_From_Flash(void)
{
    if (!flash_init_success) {
        return FLASH_INIT_ERROR;
    }
    bms_flash_data_t best_data;
    uint32_t max_seq = 0;
    uint8_t found_valid = 0;
    uint8_t best_slot_index = 0;
    uint32_t best_emergency_offset = 0;
    uint8_t loaded_from_emergency = 0;
    for (uint8_t i = 0; i < WEAR_LEVEL_SECTORS; i++) {
        uint32_t addr = WEAR_LEVEL_START_ADDR + (i * FLASH_SECTOR_SIZE);
        bms_flash_data_t temp_data;
        if (w25qxx_read(&flash_handle, addr, (uint8_t*)&temp_data, sizeof(bms_flash_data_t)) != 0) continue;
        uint32_t calc_crc = calculate_crc32((uint8_t*)&temp_data, sizeof(bms_flash_data_t) - sizeof(uint32_t));
        if (calc_crc == temp_data.crc && temp_data.sequence_number > max_seq) {
            max_seq = temp_data.sequence_number;
            best_data = temp_data;
            found_valid = 1;
            best_slot_index = i;
            loaded_from_emergency = 0;
        }
    }
    for (uint16_t i = 0; i < FLASH_EMERGENCY_ENTRIES; i++) {
        uint32_t addr = FLASH_EMERGENCY_START + (i * FLASH_EMERGENCY_PAGE);
        bms_flash_data_t temp_data;
        if (w25qxx_read(&flash_handle, addr, (uint8_t*)&temp_data, sizeof(bms_flash_data_t)) != 0) continue;
        uint32_t calc_crc = calculate_crc32((uint8_t*)&temp_data, sizeof(bms_flash_data_t) - sizeof(uint32_t));
        if (calc_crc == temp_data.crc && temp_data.sequence_number > max_seq) {
            max_seq = temp_data.sequence_number;
            best_data = temp_data;
            found_valid = 1;
            best_emergency_offset = addr - FLASH_EMERGENCY_START;
            loaded_from_emergency = 1;
        }
    }
		if (found_valid) {
        current_sequence = max_seq;
        current_wear_slot = (best_slot_index + 1) % WEAR_LEVEL_SECTORS;
        emergency_write_offset = (best_emergency_offset + FLASH_EMERGENCY_PAGE) % FLASH_EMERGENCY_SIZE;      
        uint8_t ret = restore_data_from_flash(&best_data);
				if (ret == FLASH_OK && loaded_from_emergency) {
            return 0x99;
        }
        return ret;
				}
				current_sequence = 0;
				current_wear_slot = 0;
				last_full_charge_day = Get_Day_Stamp();
				Flash_Error_Indication(FLASH_NO_DATA);
				return FLASH_NO_DATA;
}

void BMS_Sync_Flash_To_State(void) {
    currentUsage = FullChargeCapacity_mAh * (1.0f - SOC / 100.0f);
    wattUsage = currentUsage * NOMINAL_PACK_V;  
    CommandSubcommands(RESET_PASSQ);
    Safe_Delay_ms(100); 
    BQ769x2_ReadPassQ();
    mAh_at_last_sync = passed_charge_mAh + currentUsage;
    kalman_filter_init(&kf_soc, SOC, 1.0f, 2.0f);
    kalman_filter_init(&kf_soh, SOH, 1.0f, 2.0f);
}

static void Check_Full_Charge_Request(void)
{
    static uint8_t prev_request = 0; 
    uint32_t current_day = Get_Day_Stamp();
    if (SOC >= 97.0f) {
        if (last_full_charge_day != current_day) {
            last_full_charge_day = current_day;
            BMS_Save_Data_To_Flash();  
        }
        full_charge_request = 0;
    }
    else if ((current_day - last_full_charge_day) >= 30) {
        full_charge_request = 1;
    }
    if (full_charge_request && !prev_request) {
        BMS_Save_Data_To_Flash();
    }
    prev_request = full_charge_request;
}

void Check_And_Log_Permanent_Fail(void)
{
    static uint8_t pf_debounce_count = 0;
    static uint8_t prev_pf = 0;
    uint16_t battery_status = BQ769x2_ReadBatteryStatus();
    uint8_t current_pf = (battery_status & (1 << 12)) != 0;
    if (current_pf) {
        if (pf_debounce_count < 5) pf_debounce_count++;
        if (pf_debounce_count >= 5 && !prev_pf) {
            uint8_t new_pf[4] = {0};
            HAL_StatusTypeDef status;
            DirectCommands(PFStatusA, 0x00, R, &status);
            if (status == HAL_OK) new_pf[0] = RX_data[0];
            DirectCommands(PFStatusB, 0x00, R, &status);
            if (status == HAL_OK) new_pf[1] = RX_data[0];
            DirectCommands(PFStatusC, 0x00, R, &status);
            if (status == HAL_OK) new_pf[2] = RX_data[0];
            DirectCommands(PFStatusD, 0x00, R, &status);
            if (status == HAL_OK) new_pf[3] = RX_data[0];
            if (memcmp(new_pf, current_pf_status, 4) != 0) {
                memcpy(current_pf_status, new_pf, 4);
                BMS_Save_Data_To_Flash();
            }
        }
    } else {
        pf_debounce_count = 0;
    }
    prev_pf = current_pf;
}

void BMS_Clear_All_Buffers(void)
{		
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < AVERAGE_SAMPLES; j++) {
            cell_buffer[i][j] = 0;
        }
        CellVoltage[i] = 0;
    }
    for (int j = 0; j < AVERAGE_SAMPLES; j++) {
        stack_buffer[j] = 0;
        pack_buffer[j] = 0;
        ld_buffer[j] = 0;
        adc_buffer[0][j] = 0;
        adc_buffer[1][j] = 0;
        adc_buffer[2][j] = 0; 
        adc_buffer[3][j] = 0; 
    }
    for (int j = 0; j < AVERAGE_SAMPLES_CURRENT; j++) {
        current_buffer[j] = 0;
    }
		for (int i = 0; i < 9; i++) {
        Temperature[i] = 0.0f;
    }
    ntc_temp = 0.0f;
    ntc_1 = 0.0f;
    ntc_2 = 0.0f;
    avg_index = 0;
    avg_index_current = 0;
    adc_local_index = 0;
    last_soc_x100 = 0xFFFF;
    last_soh_x100 = 0xFFFF;
    last_current = 0x7FFF;
    last_stack_v = 0xFFFF;
		Stack_Voltage = 0;
    Pack_Voltage = 0;
    Pack_Current = 0;
}

void BMS_Smart_Parallel_Control(void) {
    if (is_master != 1) return;

    uint32_t max_pack_v = Stack_Voltage;
    for (uint8_t i = 0; i < MAX_SLAVES; i++) {
        if (slave_online_status[i] > 0 && slave_isolated[i] == 0) {
            if (slave_analog_data[i].total_voltage > max_pack_v) {
                max_pack_v = slave_analog_data[i].total_voltage;
            }
        }
    }
        
    const int32_t THRESH_HIGH = 1500;
    const int32_t THRESH_RELEASE = 1000;

    uint8_t cmd_chg[MAX_SLAVES + 1] = {1, 1, 1, 1}; 
    uint8_t cmd_dsg[MAX_SLAVES + 1] = {1, 1, 1, 1};
    uint8_t cmd_pchg[MAX_SLAVES + 1] = {0, 0, 0, 0};
    static uint32_t pchg_start_tick[MAX_SLAVES + 1] = {0};
    static uint8_t pchg_active[MAX_SLAVES + 1] = {0};
    uint8_t any_precharge_active = 0;
        
    for (uint8_t i = 0; i <= MAX_SLAVES; i++) {
        if (i > 0 && (slave_online_status[i-1] == 0 || slave_isolated[i-1] == 1)) {
            pchg_active[i] = 0;
            continue;      
        }   
        int32_t pack_v = (i == 0) ? Stack_Voltage : slave_analog_data[i-1].total_voltage;
        int32_t diff_from_max = max_pack_v - pack_v;
        
        uint8_t current_pchg = (i == 0) ? (HAL_GPIO_ReadPin(GPIOB, EN_PRECHARGE_Pin) == GPIO_PIN_SET) : slave_mgmt_data[i-1].stt.precharge_en;
        uint8_t current_chg  = (i == 0) ? CHG : slave_mgmt_data[i-1].stt.charge_en;

        if (!pchg_active[i]) {
            if (diff_from_max > THRESH_HIGH) {
                pchg_active[i] = 1;
                pchg_start_tick[i] = HAL_GetTick();
                cmd_chg[i] = 0;          
                cmd_pchg[i] = 1;
                any_precharge_active = 1;
            } else {
                if (i == 0) {
                    cmd_chg[0] = CHG; 
                    cmd_dsg[0] = DSG; 
                    cmd_pchg[0] = current_pchg;
                } else {
                    cmd_chg[i] = current_chg;
                    cmd_dsg[i] = slave_mgmt_data[i-1].stt.discharge_en;
                    cmd_pchg[i] = current_pchg;
                }
            }
        } 
        else {
            uint32_t elapsed = HAL_GetTick() - pchg_start_tick[i];
            if (diff_from_max <= THRESH_RELEASE || elapsed > 18000000UL) {
                pchg_active[i] = 0;
                cmd_chg[i] = 1;
                cmd_dsg[i] = 1; 
                cmd_pchg[i] = 0;
            } else {
                cmd_chg[i] = 0;          
                cmd_pchg[i] = 1;
                any_precharge_active = 1;
            }
        }
    }

    system_is_precharging = any_precharge_active;
    if (OV_Fault || UV_Fault || OCC_Fault || OCD_Fault || OCD_Fault1 || SCD_Fault ||
        OTC_Fault || OTD_Fault || UTC_Fault || UTD_Fault || OTF_Fault || PFErrorsTriggered ||
        occ_software_lock || cell_failure_locked || protection_blocked || system_is_shutting_down || healing_in_progress) {
        if (OV_Fault) {
            for (uint8_t i = 1; i <= MAX_SLAVES; i++) cmd_chg[i] = 0;
        }
        if (UV_Fault || OCD_Fault || OCD_Fault1 || SCD_Fault) {
            for (uint8_t i = 1; i <= MAX_SLAVES; i++) cmd_dsg[i] = 0;
        }
				} else {
        if (ov_recovery_locked)
				{ 
            cmd_chg[0] = 0;
            for (uint8_t i = 1; i <= MAX_SLAVES; i++) {
                cmd_chg[i] = 0;
            }
        }          
        if (uv_recovery_locked)
        { 
            cmd_dsg[0] = 0;
            for (uint8_t i = 1; i <= MAX_SLAVES; i++) {
                cmd_dsg[i] = 0;
            }
        }        
       static uint8_t last_master_pchg = 0;
        uint8_t curr_master_pchg = (cmd_pchg[0] && cmd_dsg[0]);
        static uint32_t last_master_fet_tick = 0;
            
        if (curr_master_pchg != last_master_pchg) {
            if (curr_master_pchg == 1) { 
                CommandSubcommands(ALL_FETS_ON);
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
                Safe_Delay_ms(20);
                HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_SET);
                Safe_Delay_ms(10);
                CommandSubcommands(CHG_PCHG_OFF);
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET);
            } else {
                CommandSubcommands(ALL_FETS_ON);
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
                Safe_Delay_ms(50); 
                HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
            }
            last_master_pchg = curr_master_pchg;
            last_master_fet_tick = HAL_GetTick(); 
        }
                
        uint8_t desired_master_fet = (cmd_chg[0] << 1) | cmd_dsg[0];
        uint8_t actual_master_fet = (CHG << 1) | DSG;

        if (!curr_master_pchg && (desired_master_fet != actual_master_fet) && (HAL_GetTick() - last_master_fet_tick > 2000)) {
            if (cmd_chg[0] == 0 && cmd_dsg[0] == 0) CommandSubcommands(ALL_FETS_OFF);
            else if (cmd_chg[0] == 0) CommandSubcommands(CHG_PCHG_OFF);
            else if (cmd_dsg[0] == 0) CommandSubcommands(DSG_PDSG_OFF);
            else CommandSubcommands(ALL_FETS_ON);
            last_master_fet_tick = HAL_GetTick();
        }
    }
        
    for (uint8_t i = 0; i < MAX_SLAVES; i++) {
        if (slave_online_status[i] > 0) {
            uint8_t target_adr = 0x03 + i;
            pylon_rs485_set_mgmt_t cmd = {0};
            cmd.command_value = target_adr;
            cmd.set_charge_v_limit    = 57600; 
            cmd.set_discharge_v_limit = 44800;
            cmd.stt.charge_en    = cmd_chg[i+1];
            cmd.stt.discharge_en = cmd_dsg[i+1];
            cmd.stt.precharge_en = cmd_pchg[i+1];
            cmd.set_charge_i_limit    = (cmd.stt.charge_en) ? (system_charge_limit_A) : 0;
            cmd.set_discharge_i_limit = (cmd.stt.discharge_en) ? (system_discharge_limit_A) : 0;
            Pylon_Request_Set_Mgmt(&huart5, target_adr, &cmd);
        }
    }
}

void BMS_Auto_Healing_Task(void) {
    if (is_master != 1) return;
    uint32_t now = HAL_GetTick();
    int16_t sys_curr = system_total_current_01A;

    switch (master_auto_healing_state) {
        case 0:
            for (uint8_t i = 0; i < MAX_SLAVES; i++) {
                if (slave_online_status[i] > 0 && slave_isolated[i] == 0) {
                    int16_t s_curr = slave_analog_data[i].current;
                    if (abs(sys_curr) > 200 && abs(s_curr) <= 20 && slave_alarm_data[i].s2.charge_mos == 0 && slave_alarm_data[i].s2.discharge_mos == 0 && slave_alarm_data[i].s3.system_error == 0) {                       
                        target_healing_slave = i;
                        master_auto_healing_state = 1;
                        auto_healing_timer = now;
                        healing_in_progress = 1;
                        master_needs_precharge = 0;
                        break;
                    }
                }
            }
            break;

        case 1:
            if (now - auto_healing_timer > 3000) {
                uint8_t target_adr = 0x03 + target_healing_slave;
                uint8_t reset_payload = 0x03;
                int32_t delta_v = (int32_t)Stack_Voltage - (int32_t)slave_analog_data[target_healing_slave].total_voltage;

                if (delta_v > 1500) {
                    reset_payload = 0x02;
                } else if (delta_v < -1500) {
                    reset_payload = 0x01;
                    master_needs_precharge = 1;                  
                    CommandSubcommands(ALL_FETS_ON);                     
                    HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET); 
                    HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET); 
                    Safe_Delay_ms(20);                                     
                    HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_SET); 
                    Safe_Delay_ms(10);
                    CommandSubcommands(CHG_PCHG_OFF);                     
                    HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET);  
                }
                Pylon_Request_Soft_Reset(&huart5, target_adr, reset_payload);
                slave_reset_fails[target_healing_slave]++;
                auto_healing_timer = now;
                master_auto_healing_state = 2;
            }
            break;

        case 2:
            {
                if (master_needs_precharge) {
                    int32_t cur_delta = (int32_t)slave_analog_data[target_healing_slave].total_voltage - (int32_t)Stack_Voltage;                    
                    if (cur_delta < 1000) {
                        CommandSubcommands(ALL_FETS_ON);                      
                        HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET); 
                        HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET); 
                        Safe_Delay_ms(50);                                   
                        HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
                        master_needs_precharge = 0;
                    }
                }
                int16_t s_curr = slave_analog_data[target_healing_slave].current;
                if (slave_online_status[target_healing_slave] > 0 && 
                   (abs(s_curr) > 10 || (slave_alarm_data[target_healing_slave].s2.charge_mos == 1 && slave_alarm_data[target_healing_slave].s2.discharge_mos == 1))) {
                    slave_reset_fails[target_healing_slave] = 0;
                    master_auto_healing_state = 3;
                }
                if (now - auto_healing_timer > 15000) {
                    master_auto_healing_state = 3;
                }
            }
            break;

        case 3:
            if (master_needs_precharge) {
                CommandSubcommands(ALL_FETS_ON);
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
                Safe_Delay_ms(50);
                HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
                master_needs_precharge = 0;
            }

            if (slave_reset_fails[target_healing_slave] >= 5) {
                slave_isolated[target_healing_slave] = 1;
            }          
            healing_in_progress = 0;
            master_auto_healing_state = 0;
            break;
    }
}

void BMS_Slave_FET_Control(void) {
    if (is_master == 1) return;
    if (occ_software_lock || cell_failure_locked || protection_blocked || system_is_shutting_down ||
        OV_Fault || UV_Fault || OCC_Fault || OCD_Fault || OCD_Fault1 || SCD_Fault ||
        OTC_Fault || OTD_Fault || UTC_Fault || UTD_Fault || OTF_Fault || PFErrorsTriggered) {
				if (HAL_GPIO_ReadPin(GPIOB, EN_PRECHARGE_Pin) == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
        }
        return; 
    }   
    uint8_t local_chg = !ov_recovery_locked;
    uint8_t local_dsg = !uv_recovery_locked;
    uint8_t net_chg = 1, net_dsg = 1, net_pchg = 0;
        
    if (has_received_master_cmd && (HAL_GetTick() - last_master_cmd_tick < 10000)) {
        net_chg = last_master_cmd.stt.charge_en;
        net_dsg = last_master_cmd.stt.discharge_en;
        net_pchg = last_master_cmd.stt.precharge_en;
    } else {
        net_chg = 1; net_dsg = 1; net_pchg = 0;
    }   
    
    uint8_t final_chg = local_chg && net_chg;
    uint8_t final_dsg = local_dsg && net_dsg;
    uint8_t desired_state = (final_chg << 1) | final_dsg;
    uint8_t actual_state = (CHG << 1) | DSG;
    uint8_t current_pchg_pin = HAL_GPIO_ReadPin(GPIOB, EN_PRECHARGE_Pin);

    if (desired_state != actual_state) {
        if (net_pchg == 0 && current_pchg_pin == GPIO_PIN_SET) {
            CommandSubcommands(ALL_FETS_ON); 
            Safe_Delay_ms(50);    
            HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
        } 
        else {
            if (final_chg == 0 && final_dsg == 0) CommandSubcommands(ALL_FETS_OFF);
            else if (final_chg == 0) CommandSubcommands(CHG_PCHG_OFF);
            else if (final_dsg == 0) CommandSubcommands(DSG_PDSG_OFF);
            else CommandSubcommands(ALL_FETS_ON);
        }
    }

    if (net_pchg && final_dsg) {
        HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_SET);
    } else if (current_pchg_pin == GPIO_PIN_SET && desired_state == actual_state) {
        HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET); // Rào chắn an toàn vét đáy
    }
}
   
//void BMS_Check_FET_Stuck_Failure(void) {
//    BQ769x2_ReadFETStatus();
//    uint8_t is_stuck = 0;
//    const int16_t STUCK_CURRENT_THRESHOLD = 500; 
//    if (CHG == 0 && Pack_Current > STUCK_CURRENT_THRESHOLD) {
//        is_stuck = 1;
//    }
//    else if (DSG == 0 && Pack_Current < -STUCK_CURRENT_THRESHOLD) {
//        is_stuck = 1;
//    }
//    if (is_stuck) {
//    if (fet_fail_timer == 0) {
//        fet_fail_timer = HAL_GetTick();
//        } 
//        else if (HAL_GetTick() - fet_fail_timer >= 8000) {
//            bms_state = BMS_STATE_FAULT;
//        }
//    } 
//    else {
//        fet_fail_timer = 0;
//    }
//}

void BMS_Integrated_Power_Management(void) {
    static uint32_t button_press_start_tick = 0;
    static uint32_t button_release_tick = 0;
    static uint8_t prev_button_state = 0;
	
    uint8_t current_button_pressed = (HAL_GPIO_ReadPin(GPIOC, BUTTON_STATE_Pin) == GPIO_PIN_SET);
	
    if (current_button_pressed) {
        HAL_GPIO_WritePin(GPIOC, POWER_HOLD_Pin, GPIO_PIN_SET);
        button_release_tick = 0;
        if (!prev_button_state) {
            if (bms_sleep_mode == 1) {
                BMS_Init_Address();  
                DWIN_ScreenOn();    
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);
                CommandSubcommands(ALL_FETS_ON);             
                bms_sleep_mode = 0;
                bms_state = BMS_STATE_STANDBY;
                BMS_Clear_All_Buffers();
                first_sample_after_reset = 1;
                Safe_Delay_ms(250);            
                BQ769x2_ReadAllVoltages();    
                Pack_Current = BQ769x2_ReadCurrent();
                Update_SOC_SOH_FromBQ();      
                first_sample_after_reset = 0;
                dwin_need_update = 1;
            }
            button_press_start_tick = HAL_GetTick();
        }
        if (bms_sleep_mode == 0 && is_auto_coding == 0) {
            if (HAL_GetTick() - button_press_start_tick > 5000) {
                is_master = 0;
                assigned_n = 0;
                pylon_protocol_addr = 0x00; 
                SET_DN_OP_LOW();                 
                is_auto_coding = 1; 
                auto_code_state = AUTO_CODE_START;
                button_press_start_tick = HAL_GetTick() + 10000;
            }
        }
        data_saved_by_button = 0;
        system_is_shutting_down = 0;
    } 
    else {
        if (prev_button_state) {
            button_release_tick = HAL_GetTick();
        }
				if (button_release_tick != 0 && (HAL_GetTick() - button_release_tick > 3000)) { 
            if (bms_sleep_mode == 0) {
                system_is_shutting_down = 1;
                DWIN_ScreenOff();
                HAL_GPIO_WritePin(DN_OP_GPIO_Port, DN_OP_Pin, GPIO_PIN_RESET);
								CommandSubcommands(ALL_FETS_OFF);
                HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);
								if (flash_init_success && !data_saved_by_button) {
                    BMS_Save_Data_To_Flash();
                    data_saved_by_button = 1;
                }
								LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
								LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_OFF);
                current_buzzer_mode = BUZZER_MODE_OFF;
                HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET);
                bms_sleep_mode = 1;
                bms_state = BMS_STATE_SLEEP;
                Safe_Delay_ms(200);
                HAL_IWDG_Refresh(&hiwdg);
                HAL_GPIO_WritePin(GPIOC, POWER_HOLD_Pin, GPIO_PIN_RESET);              
                button_release_tick = 0;
            }
        }
    }
    prev_button_state = current_button_pressed;
}

void BMS_Parallel_Timeout_Check(void) {
    static uint32_t last_scan_tick = 0;
    if (is_master != 1) return;
    if (HAL_GetTick() - last_scan_tick >= 1000) {
        last_scan_tick = HAL_GetTick();
        for (int i = 0; i < MAX_SLAVES; i++) {
            if (slave_online_status[i] > 0) {
                slave_online_status[i]--;
                if (slave_online_status[i] == 0) {
                    memset(&slave_analog_data[i], 0, sizeof(pylon_rs485_analog_t));
                    memset(&slave_alarm_data[i], 0, sizeof(pylon_rs485_alarm_t));
                    memset(&slave_mgmt_data[i], 0, sizeof(pylon_rs485_chg_dis_mgmt_t));
                }
            }
            if (slave_isolated[i] == 1) {
                extern HAL_StatusTypeDef Pylon_Request_Data(UART_HandleTypeDef *huart, uint8_t target_adr, uint8_t cid2);
                Pylon_Request_Data(&huart5, 0x03 + i, PYLON_CID2_TURN_OFF);
            }
        }
    }
}

void BMS_Aggregate_System_Data(void) {
    if (is_master != 1) return;
    uint8_t count = 1;
    uint8_t current_sys_err = 0;
    if (PFErrorsTriggered || cell_failure_locked) {
        current_sys_err = 1;
    }
    int32_t total_curr_01A = (int16_t)Pack_Current;
    uint32_t total_voltage_mv = (uint32_t)Stack_Voltage;
    float total_remain_mah = (float)FullChargeCapacity_mAh * (SOC / 100.0f);
    float total_full_mah   = (float)FullChargeCapacity_mAh;
    uint32_t total_cycles = cycle_count;
    float total_soh_cap_mah = (float)FullChargeCapacity_mAh;

    uint32_t total_chg_curr_limit = system_charge_limit_A; 
    uint32_t total_dis_curr_limit = system_discharge_limit_A;
    
    uint16_t g_max_cell_v = 0;
    uint16_t g_min_cell_v = 0xFFFF;
    uint8_t  g_max_v_pack_id = 1, g_max_v_cell_id = 1;
    uint8_t  g_min_v_pack_id = 1, g_min_v_cell_id = 1;

    int16_t g_max_temp = -273; 
    int16_t g_min_temp = 1000;
    uint8_t g_max_t_pack_id = 1, g_max_t_cell_id = 1;
    uint8_t g_min_t_pack_id = 1, g_min_t_cell_id = 1;
		
		int16_t master_t_arr[8] = {
        (int16_t)Temperature[0], (int16_t)Temperature[1], 
        (int16_t)Temperature[2], (int16_t)Temperature[3],
        (int16_t)ntc_1,          (int16_t)ntc_2, 
        (int16_t)Temperature[6], (int16_t)ntc_temp
    };

    for (int t = 0; t < 8; t++) {
        int16_t m_temp = master_t_arr[t];
        if (m_temp > g_max_temp) { 
            g_max_temp = m_temp; 
            g_max_t_pack_id = 1; 
            g_max_t_cell_id = t + 1; 
        }
        if (m_temp < g_min_temp) { 
            g_min_temp = m_temp; 
            g_min_t_pack_id = 1; 
            g_min_t_cell_id = t + 1; 
        }
    }
		
    local_max_cell_v = 0;
    local_min_cell_v = 0xFFFF;
    
    for(int i = 0; i < 16; i++) {
        if (CellVoltage[i] > local_max_cell_v) { 
            local_max_cell_v = CellVoltage[i]; 
            local_max_cell_id = i + 1; 
        }
        if (CellVoltage[i] < local_min_cell_v && CellVoltage[i] > 500) { 
            local_min_cell_v = CellVoltage[i]; 
            local_min_cell_id = i + 1; 
        }
    }
		
		g_max_cell_v = local_max_cell_v;
    g_max_v_cell_id = local_max_cell_id;
    g_min_cell_v = local_min_cell_v;
    g_min_v_cell_id = local_min_cell_id;
		
		sys_protect_ov  = OV_Fault;
    sys_protect_uv  = UV_Fault;
    sys_protect_occ = OCC_Fault;
    sys_protect_ocd = (OCD_Fault || OCD_Fault1);
    sys_protect_ot  = (OTC_Fault || OTD_Fault || OTF_Fault);
    sys_protect_ut  = (UTC_Fault || UTD_Fault);

    sys_alarm_ov  = bms_alarms.OV_Alarm || bms_alarms.Stack_OV_Alarm;
    sys_alarm_uv  = bms_alarms.UV_Alarm || bms_alarms.Stack_UV_Alarm;
    sys_alarm_occ = bms_alarms.OCC_Alarm;
    sys_alarm_ocd = bms_alarms.OCD1_Alarm;
    sys_alarm_ot  = bms_alarms.OTC_Alarm || bms_alarms.OTD_Alarm;
    sys_alarm_ut  = bms_alarms.UTC_Alarm || bms_alarms.UTD_Alarm;
		
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (slave_online_status[i] > 0 && slave_isolated[i] == 0) {
            count++;
            total_curr_01A += (int32_t)slave_analog_data[i].current;
            total_voltage_mv += (uint32_t)slave_analog_data[i].total_voltage;
            
            uint32_t slave_rem = (slave_analog_data[i].remain_cap_2[0] << 16) | (slave_analog_data[i].remain_cap_2[1] << 8) | slave_analog_data[i].remain_cap_2[2];
            uint32_t slave_tot = (slave_analog_data[i].total_cap_2[0] << 16) | (slave_analog_data[i].total_cap_2[1] << 8) | slave_analog_data[i].total_cap_2[2];
            
            if (slave_analog_data[i].user_def_count < 4) {
                slave_rem = slave_analog_data[i].remain_cap_1 * 100;
                slave_tot = slave_analog_data[i].total_cap_1 * 100;
            }
                    
            total_remain_mah += (float)slave_rem;
            total_full_mah   += (float)slave_tot;
            total_soh_cap_mah += (float)slave_tot;
                    
            total_chg_curr_limit += slave_mgmt_data[i].charge_current_limit;
            total_dis_curr_limit += slave_mgmt_data[i].discharge_current_limit;
                        
            for (int c = 0; c < slave_analog_data[i].cell_count; c++) {
                uint16_t cv = slave_analog_data[i].cell_voltages[c];
                if (cv > g_max_cell_v) { 
                    g_max_cell_v = cv; 
                    g_max_v_pack_id = i + 2;
                    g_max_v_cell_id = c + 1; 
                }
                if (cv < g_min_cell_v && cv > 500) { 
                    g_min_cell_v = cv; 
                    g_min_v_pack_id = i + 2; 
                    g_min_v_cell_id = c + 1; 
                }
            }
            for (int t = 0; t < slave_analog_data[i].temp_count; t++) {
                int16_t s_temp = (int16_t)((slave_analog_data[i].temperatures[t] - 2731) / 10);
                if (s_temp > g_max_temp) { 
                    g_max_temp = s_temp; 
                    g_max_t_pack_id = i + 2; 
                    g_max_t_cell_id = t + 1; 
                }
                if (s_temp < g_min_temp) { 
                    g_min_temp = s_temp; 
                    g_min_t_pack_id = i + 2; 
                    g_min_t_cell_id = t + 1; 
                }
            }

            total_cycles += slave_analog_data[i].cycle_count;
                        
            if (slave_alarm_data[i].s1.module_ov) sys_protect_ov = 1;
            if (slave_alarm_data[i].s1.module_uv || slave_alarm_data[i].s1.cell_uv) sys_protect_uv = 1;
            if (slave_alarm_data[i].s1.charge_oc) sys_protect_occ = 1;
            if (slave_alarm_data[i].s1.discharge_oc) sys_protect_ocd = 1;
            if (slave_alarm_data[i].s1.charge_ot || slave_alarm_data[i].s1.discharge_ot) sys_protect_ot = 1;
            if (slave_alarm_data[i].module_volt_status == 0x02) sys_alarm_ov = 1;
            if (slave_alarm_data[i].module_volt_status == 0x01) sys_alarm_uv = 1;
            if (slave_alarm_data[i].charge_curr_status == 0x02) sys_alarm_occ = 1;
            if (slave_alarm_data[i].discharge_curr_status == 0x02) sys_alarm_ocd = 1;
                        
            for (int c = 0; c < slave_alarm_data[i].cell_count; c++) {
                if (slave_alarm_data[i].cell_status[c] == 0x02) { sys_protect_ov = 1; sys_alarm_ov = 1; }
                if (slave_alarm_data[i].cell_status[c] == 0x01) { sys_protect_uv = 1; sys_alarm_uv = 1; }
            }
            for (int t = 0; t < slave_alarm_data[i].temp_count; t++) {
                if (slave_alarm_data[i].temp_status[t] == 0x02) { sys_protect_ot = 1; sys_alarm_ot = 1; }
                if (slave_alarm_data[i].temp_status[t] == 0x01) { sys_protect_ut = 1; sys_alarm_ut = 1; }
            }
            if (slave_alarm_data[i].s3.system_error) {
                current_sys_err = 1; 
            }
        }
    }
		
		active_packs_count = count;
    system_total_current_01A = total_curr_01A;
    system_total_capacity_mAh = (uint32_t)total_full_mah;    
    if (count > 0) {
        system_total_voltage_01V = (uint16_t)(total_voltage_mv / count);
        system_avg_cycles = (uint16_t)roundf((float)total_cycles / count);
        if (total_full_mah > 0) {
            system_avg_soc = (uint16_t)roundf(total_remain_mah * 100.0f / total_full_mah);
        }
        if (system_avg_soc >= 99 && !sys_protect_ov && !ov_recovery_locked) {
            system_avg_soc = 99;
        } else if (sys_protect_ov || ov_recovery_locked) {
            system_avg_soc = 100;
        }
        float nominal_sys_cap = (float)count * 314000.0f; 
        system_avg_soh = (uint16_t)roundf(total_soh_cap_mah * 100.0f / nominal_sys_cap);
    }
    
    system_charge_v_limit_final    = system_charge_v_limit_mV;
    system_discharge_v_limit_final = system_discharge_v_limit_mV;
    system_charge_limit_A_final    = (uint16_t)total_chg_curr_limit;
//    system_discharge_limit_A_final = (uint16_t)total_dis_curr_limit;
    if (adv_stop_discharge_active) 
    {
        system_discharge_limit_A_final = 0;
    }
    else if (adv_soc_clamp_active) 
    {
        system_discharge_limit_A_final = (uint16_t)(total_dis_curr_limit * 0.1f); 
        if (system_discharge_limit_A_final < 10) system_discharge_limit_A_final = 10; 
    }
    else 
    {
        system_discharge_limit_A_final = (uint16_t)total_dis_curr_limit;
    }	
    if (sys_protect_ov || ov_recovery_locked || occ_software_lock || cell_failure_locked || protection_blocked || current_sys_err || healing_in_progress || system_is_precharging) {
        system_charge_limit_A_final = 0;
    }

    if (sys_protect_uv || uv_recovery_locked || cell_failure_locked || protection_blocked || current_sys_err || healing_in_progress || system_is_precharging) {
        system_discharge_limit_A_final = 0;
    }
    
    sys_system_error = current_sys_err;
    system_max_temp_C = g_max_temp;
    system_min_temp_C = g_min_temp;
    
    sys_max_cell_v = g_max_cell_v;
    sys_min_cell_v = g_min_cell_v;
    sys_max_v_pack_id = g_max_v_pack_id;
    sys_max_v_cell_id = g_max_v_cell_id;
    sys_min_v_pack_id = g_min_v_pack_id;
    sys_min_v_cell_id = g_min_v_cell_id;
    
    sys_max_t_pack_id = g_max_t_pack_id;
    sys_max_t_cell_id = g_max_t_cell_id;
    sys_min_t_pack_id = g_min_t_pack_id;
    sys_min_t_cell_id = g_min_t_cell_id;
}

void BMS_Advanced_Discharge_Logic(void)
{
    if (is_master != 1) return; 
    uint32_t now = HAL_GetTick();
    static uint32_t last_exec_tick = 0;
    uint32_t delta_t = now - last_exec_tick;
    if (delta_t < 250) return;
    last_exec_tick = now;  
    uint16_t delta_cell = sys_max_cell_v - sys_min_cell_v;
    uint8_t is_discharging = (system_total_current_01A <= -50);
    static uint32_t last_discharge_tick = 0;
    if (is_discharging) last_discharge_tick = now;
    uint8_t is_or_was_discharging = (now - last_discharge_tick < 60000);
    if (SOC <= 20.0f || sys_min_cell_v <= 2950) 
    {
        adv_stop_discharge_active = 1;
        adv_soc_clamp_active = 0;
    }
    else if (sys_min_cell_v <= 3200 && Stack_Voltage <= 52000 && delta_cell <= 100 && 
             is_or_was_discharging && SOC > 20.0f && !adv_stop_discharge_active) 
    {
        if (adv_soc_clamp_timer_ms == 0) adv_soc_clamp_timer_ms = now;
        if (now - adv_soc_clamp_timer_ms >= 60000)
        {
            adv_soc_clamp_active = 1;
        }
    } 
    else 
    {
        adv_soc_clamp_timer_ms = 0;
    }
    uint8_t is_real_charging = (system_total_current_01A >= 200);
    if (is_real_charging) 
    {
        if (adv_real_charge_timer_ms == 0) adv_real_charge_timer_ms = now;
    } 
    else 
    {
        adv_real_charge_timer_ms = 0;
    }
    if (SOC >= 25.0f && sys_min_cell_v >= 3150 && 
        (adv_real_charge_timer_ms != 0 && (now - adv_real_charge_timer_ms >= 300000)) && !is_discharging) 
    {
        adv_soc_clamp_active = 0;
        adv_stop_discharge_active = 0;
    }
}

#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  volatile int i = 0;
	uint8_t reset_by_iwdg = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
	{
			reset_by_iwdg = 1;
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (reset_by_iwdg)
	{
			HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_SET);
			for (int k = 0; k < 50; k++)
			{
					HAL_IWDG_Refresh(&hiwdg);
					for (volatile int j = 0; j < 60000; j++);
			}
			HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_RESET);
	}
//	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	rd_flash_init();
	rd_init_control();

  #if (IS_BOOTLOADER == 0)
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
	// Start timer
	I2C_Bus_Recovery();
	DS1307_Enable_SQW_1Hz();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_byte_u4, 1);
  HAL_UART_Receive_IT(&huart5, (uint8_t *)&rx_byte_u5, 1);
	
  HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, RS485_DE_RE1_Pin, GPIO_PIN_RESET);    
  HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin, GPIO_PIN_RESET);  // RST_SHUT pin set low
  HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_RESET);   // DFETOFF pin (BOTHOFF) set low
  HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, POWER_HOLD_Pin, GPIO_PIN_SET);

  Safe_Delay_ms(10);
	CommandSubcommands(BQ769x2_RESET);  											// Resets the BQ769x2 registers
	Safe_Delay_ms(60);
	BQ769x2_Init();  																					// Configure all of the BQ769x2 register settings
	Safe_Delay_ms(60);
//	CommandSubcommands(CHGTEST); 														// Enable the CHG and DSG FETs
//	delayUS(10000);
//	CommandSubcommands(DSGTEST); 	 													// Enable the CHG and DSG FETs
//	delayUS(10000);
//	CommandSubcommands(PCHGTEST); 													// Enable the CHG and DSG FETs
//	delayUS(10000);
//	CommandSubcommands(PDSGTEST); 	 												// Enable the CHG and DSG FETs
//	delayUS(10000);
//	CommandSubcommands(ALL_FETS_ON); 	 											// Enable the CHG and DSG FETs
//	delayUS(10000);
//	CommandSubcommands(FET_ENABLE); 	 											// Enable the CHG and DSG FETs
//	delayUS(10000);
//	Subcommands(FET_CONTROL, 0x00, W);
//	delayUS(10000);
	CommandSubcommands(SLEEP_DISABLE); 												// Sleep mode is enabled by default. For this example, Sleep is disabled to 
																														// demonstrate full-speed measurements in Normal mode.
	Safe_Delay_ms(60);
	Safe_Delay_ms(60); 
	Safe_Delay_ms(60); 
	Safe_Delay_ms(60);  																			//wait to start measurements after FETs close	
	system_charge_v_limit_mV    = 57600; 
	system_discharge_v_limit_mV = 44800; 
	system_charge_limit_A       = 120;   
	system_discharge_limit_A    = 120;
	hardware_dip_id = BMS_Get_Address_ID(); 
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCScanVal, 4);
  Safe_Delay_ms(10);
  for (int j = 0; j < AVERAGE_SAMPLES_ADC; j++) {
    adc_buffer[0][j] = ADCScanVal[0];   
    adc_buffer[1][j] = ADCScanVal[1];
    adc_buffer[2][j] = ADCScanVal[2];   
    adc_buffer[3][j] = ADCScanVal[3];       
  }
	
	LED_SetMode(LED9_GPIO_Port, LED9_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED8_GPIO_Port, LED8_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED7_GPIO_Port, LED7_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED6_GPIO_Port, LED6_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED5_GPIO_Port, LED5_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED4_GPIO_Port, LED4_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED3_GPIO_Port, LED3_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED2_GPIO_Port, LED2_Pin, FLASH_MODE_OFF);
  LED_SetMode(LED1_GPIO_Port, LED1_Pin, FLASH_MODE_OFF);
  DS1307_GetTime(&time);
  if (time.year < 26) { 
      time.sec   = 30;
      time.min   = 22;
      time.hour  = 9;
      time.day   = 04;       
      time.date  = 03;       
      time.month = 06;       
      time.year  = 26;      
      DS1307_SetTime(&time);
  }
	flash_status = BMS_Flash_Init();
    if (flash_status == FLASH_OK) {
        emergency_write_offset = 0;
        uint8_t load_res = BMS_Load_Data_From_Flash();
        if (load_res == FLASH_OK || load_res == 0x99) {
            uint32_t current_chip_hash = Calculate_Device_Hash();
            if (saved_device_hash != current_chip_hash) {
                bms_state = BMS_STATE_FAULT;
                CommandSubcommands(ALL_FETS_OFF);
                while(1) {
                    HAL_GPIO_TogglePin(GPIOD, TEST_LED_Pin);
                    HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);
                    Safe_Delay_ms(200);
                    rd_run_while_check_uart();
                }
            }
            BMS_Sync_Flash_To_State();
            flash_status = FLASH_OK;
            if (load_res == 0x99) {
                BMS_Save_Data_To_Flash();
                for (uint32_t offset = 0; offset < FLASH_EMERGENCY_SIZE; offset += FLASH_SECTOR_SIZE) {
                    if (Ota_data.start_ota == 1) break;
                    w25qxx_sector_erase_4k(&flash_handle, FLASH_EMERGENCY_START + offset);
                    wait_flash_ready(1000);
                }
                emergency_write_offset = 0; 
            }
        } else {
            SOC = 100.0f;
            SOH = 100.0f;
            cycle_count = 0;
            FullChargeCapacity_mAh = NOMINAL_CAPACITY_mAh;
            last_full_charge_day = Get_Day_Stamp();
            mAh_at_last_sync = 0.0f;
            accumulated_discharge_mAh = 0.0f;
            for (uint32_t offset = 0; offset < FLASH_EMERGENCY_SIZE; offset += FLASH_SECTOR_SIZE) {
                if (Ota_data.start_ota == 1) break;
                w25qxx_sector_erase_4k(&flash_handle, FLASH_EMERGENCY_START + offset);
                wait_flash_ready(1000);
            }
            emergency_write_offset = 0;
            BMS_Sync_Flash_To_State();
            BMS_Save_Data_To_Flash();
            flash_status = FLASH_NO_DATA;
        }
    } else {
        SOC = 100.0f;
        SOH = 100.0f;
        cycle_count = 0;
        FullChargeCapacity_mAh = NOMINAL_CAPACITY_mAh;
        BMS_Sync_Flash_To_State();
    }

//	if (flash_status == FLASH_OK) {
//        for (int i = 0; i < WEAR_LEVEL_SECTORS; i++) {
//						if (Ota_data.start_ota == 1) break;
//            w25qxx_sector_erase_4k(&flash_handle, WEAR_LEVEL_START_ADDR + (i * FLASH_SECTOR_SIZE));
//            wait_flash_ready(100);
//        }
//        for (uint32_t offset = 0; offset < FLASH_EMERGENCY_SIZE; offset += FLASH_SECTOR_SIZE) {
//          if (Ota_data.start_ota == 1) break;  
//					w25qxx_sector_erase_4k(&flash_handle, FLASH_EMERGENCY_START + offset);
//          wait_flash_ready(100);
//        }
//				if (Ota_data.start_ota == 0) {
//        SOC = 100.0f;
//        SOH = 100.0f;
//        cycle_count = 0;
//        FullChargeCapacity_mAh = NOMINAL_CAPACITY_mAh;
//        last_full_charge_day = Get_Day_Stamp();
//        mAh_at_last_sync = 0.0f; 
//        accumulated_discharge_mAh = 0.0f;
//        memset(current_pf_status, 0, 4);
//        BMS_Sync_Flash_To_State();
//        BMS_Save_Data_To_Flash();
//				}
//        while(1) {
//        HAL_GPIO_TogglePin(GPIOD, TEST_LED_Pin);
//        rd_run_while_check_uart();
//        Safe_Delay_ms(100);
//        if(Ota_data.start_ota == 1) {
//            Safe_Delay_ms(1);
//        }
//    }
//}

//	if (flash_status == FLASH_OK) {
//			emergency_write_offset = 0;		
//			uint8_t load_res = BMS_Load_Data_From_Flash();		
//			if (load_res == FLASH_OK) {
//					cycle_count = 0;
//					accumulated_discharge_mAh = 0.0f; 					
//					BMS_Save_Data_To_Flash(); 				
//					while(1) {
//							HAL_GPIO_TogglePin(GPIOD, TEST_LED_Pin);
//							Safe_Delay_ms(100);
//   						rd_run_while_check_uart();
//					}
//			} else {
//					while(1) {
//							HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_SET);
//                            Safe_Delay_ms(100);
//                            rd_run_while_check_uart();
//					}
//			}
//	}
	BMS_Init_Address();
	first_sample_after_reset = 1;
  BQ769x2_ReadAllVoltages();  
  Pack_Current = BQ769x2_ReadCurrent();  
  Update_SOC_SOH_FromBQ();
  first_sample_after_reset = 0;   
    
  BQ769x2_ReadPassQ(); 
  BQ769x2_ClearLatchedAlerts();
  BQ769x2_PrepareFetOn();
  Safe_Delay_ms(10);
    
  __HAL_RCC_PWR_CLK_ENABLE();
  PWR_PVDTypeDef sPVDConfig = {0};
  sPVDConfig.PVDLevel = PWR_PVDLEVEL_7;
  sPVDConfig.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
  HAL_PWR_ConfigPVD(&sPVDConfig);
  HAL_PWR_EnablePVD();
	
	#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
  {       
      HAL_IWDG_Refresh(&hiwdg);
      #if IS_BOOTLOADER
					rd_run_while_check_uart();
					rd_run_whiletrue_boot();
					HAL_Delay(1);
      #else
					rd_run_while_check_uart();
//		 			if (Ota_data.start_ota == 1) 
//		 			{
//		 				continue; 
//          }
      uint32_t current_tick_2 = HAL_GetTick();
			static uint32_t last_key_tick = 0;
      if (current_tick_2 - last_key_tick >= 10) 
      {
					last_key_tick = current_tick_2;
          BMS_Integrated_Power_Management();
          Check_Reset_Button();
					Process_Remote_Reset_Task();
          BMS_Auto_Coding_Task();
          adc_buffer[0][adc_local_index] = ADCScanVal[0];  
          adc_buffer[1][adc_local_index] = ADCScanVal[1];
          adc_buffer[2][adc_local_index] = ADCScanVal[2];  
          adc_buffer[3][adc_local_index] = ADCScanVal[3];               
          adc_local_index = (adc_local_index + 1) % AVERAGE_SAMPLES_ADC;
			}
			if (!BMS_IsResetting() && !system_is_shutting_down)
      {
			static uint32_t last_rs485_tick = 0;
          extern volatile uint8_t rs485_rx_success_flag;
          if (current_tick_2 - last_rs485_tick >= 20) {
              last_rs485_tick = current_tick_2;
              if (is_master == 1) {
                  static uint8_t slave_target_id = 3; 
                  static uint8_t cmd_state = 0;
                  static uint32_t wait_timeout_tick = 0;
                  if (slave_target_id == pylon_protocol_addr) {
                      slave_target_id++;
                  }
									if (slave_target_id <= (0x02 + MAX_SLAVES)) {
                      switch (cmd_state) {
                          case 0:
                              rs485_rx_success_flag = 0;
                              Pylon_Request_Data(&huart5, slave_target_id, PYLON_CID2_ANALOG_DATA);
                              wait_timeout_tick = current_tick_2;
                              cmd_state = 1;
                              break;
                          case 1:
                              if (rs485_rx_success_flag == 1 || (current_tick_2 - wait_timeout_tick >= 60)) cmd_state = 2;
                              break;
                          case 2:
                              rs485_rx_success_flag = 0;
                              Pylon_Request_Data(&huart5, slave_target_id, PYLON_CID2_ALARM_INFO);
                              wait_timeout_tick = current_tick_2;
                              cmd_state = 3;
                              break;
                          case 3:
                              if (rs485_rx_success_flag == 1 || (current_tick_2 - wait_timeout_tick >= 40)) cmd_state = 4; 
                              break;
                          case 4:
                              rs485_rx_success_flag = 0;
                              Pylon_Request_Data(&huart5, slave_target_id, PYLON_CID2_CHG_DIS_MGMT);
                              wait_timeout_tick = current_tick_2;
                              cmd_state = 5;
                              break;
                          case 5:
                              if (rs485_rx_success_flag == 1 || (current_tick_2 - wait_timeout_tick >= 30)) {
                                  cmd_state = 0;  
                                  slave_target_id++; 
                              }
                              break;
                      }
                  }
							
							if (slave_target_id > (0x02 + MAX_SLAVES)) {
                      if (cmd_state == 0) {
                          BMS_Aggregate_System_Data();                      
                          extern void Pylon_Broadcast_Master_Analog(UART_HandleTypeDef *huart);
                          Pylon_Broadcast_Master_Analog(&huart5);                    
                          wait_timeout_tick = current_tick_2;
                          cmd_state = 100; 
                      } 
                      else if (cmd_state == 100) {
                          if (current_tick_2 - wait_timeout_tick >= 50) {
                              slave_target_id = 3; 
                              cmd_state = 0;       
                          }
                      }
                  }
              }
          }
			static uint32_t last_fast_tick = 0; 
          if (current_tick_2 - last_fast_tick >= 250) {
              last_fast_tick = current_tick_2;
              battery_status_value = BQ769x2_ReadBatteryStatus();
              BQ769x2_ReadAllVoltages();  
              Pack_Current = BQ769x2_ReadCurrent();
							BMS_Advanced_Discharge_Logic();							
              Update_SOC_SOH_FromBQ();
							BMS_Smart_Parallel_Control();
							BMS_Auto_Healing_Task();
              BMS_Slave_FET_Control();
              BQ769x2_HandleProtection();
							uint8_t is_pf_active = (battery_status_value & (1u << 12)) != 0;
              if (is_pf_active) 
              {
                if (!pf_active_latched) 
                  {
                      pf_active_latched = 1;
                      bms_state = BMS_STATE_FAULT;
                      dwin_need_update = 1;
                  }
              } 
              else 
              {
                  pf_active_latched = 0;
              }
          }
					static uint32_t last_1sec_tick = 0;
          if (current_tick_2 - last_1sec_tick >= 1000) {
              last_1sec_tick = current_tick_2;
              BMS_Parallel_Timeout_Check();
          }
					if (rtc_read_flag) 
          {
              rtc_read_flag = 0;
              DS1307_GetTime(&time); 
              DWIN_SendTime(&time);
              
              if (init_loop_count < 20) 
              {
                  init_loop_count++;
                  if (init_loop_count == 20) 
                  {
                      alarms_enabled = 1;
                  }
              }
							Update_Cycle_Count();
              pack_voltage_adc = Get_PackVoltage_V();
              ntc_temp = Get_ExternalNTCTemp_C(1);
              ntc_1    = Get_ExternalNTCTemp_C(2);
              ntc_2    = Get_ExternalNTCTemp_C(3);              
              Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
              Temperature[1] = BQ769x2_ReadTemperature(TS2Temperature);
              Temperature[2] = BQ769x2_ReadTemperature(TS3Temperature);
              Temperature[3] = BQ769x2_ReadTemperature(HDQTemperature);
              Temperature[5] = BQ769x2_ReadTemperature(HDQTemperature);
              Temperature[6] = BQ769x2_ReadTemperature(IntTemperature);
							BQ769x2_ReadFETStatus();
              statusread();
              Check_And_Log_Permanent_Fail();
              Check_Full_Charge_Request();
              Update_LED_Indication();
              Update_Buzzer_Logic();
              Update_DryContacts();
							if (is_master == 1) 
              {
								Tx_BQ_BMS_Status_via_CAN(&hcan);
              }
							if (HAL_GetTick() - last_inverter_alive_tick > 5000) {
                  inverter_comm_fault = 1;
              }
							static uint8_t ten_sec_counter = 0;
              if (++ten_sec_counter >= 10) {
                  if (Subcommands(DASTATUS5, 0x00, R) == HAL_OK) {
                      vref2_check_counts = ((RX_32Byte[1]<<8) + RX_32Byte[0]);
                  }
                  BQ769x2_OTP_STATUS();
                  BQ769x2_OTP_SCAN();
                  control_status = BQ769x2_ReadControlStatus();
                  battery_status = BQ769x2_ReadBatteryStatus();
                  alarm_status_reg = BQ769x2_ReadAlarmStatusReg();
                  alarm_raw_status = BQ769x2_ReadAlarmRawStatus();
                  alarm_enable_mask = BQ769x2_ReadAlarmEnable();
                  manufacturing_status = BQ769x2_ReadManufacturingStatus();
                  if (ALRT_pin == 0x01 && !protection_blocked)
                  {
                      BQ769x2_ClearLatchedAlerts();
                  }
									ten_sec_counter = 0;
              }
          }
					static uint16_t last_soc_x100                = 0xFFFF;
          static uint16_t last_soh_x100                = 0xFFFF;
          static int16_t last_current                  = 0x7FFF;
          static uint16_t last_stack_v                 = 0xFFFF;
          static uint32_t last_dwin_force_update       = 0;
					uint16_t curr_soc_x100                       = (uint16_t)roundf(SOC * 100.0f);
          uint16_t curr_soh_x100                       = (uint16_t)roundf(SOH * 100.0f);
          int16_t curr_current                         = Pack_Current;
          uint16_t curr_stack_v                        = Stack_Voltage;
					if (pf_active_latched) 
          {
              if (!pf_ui_already_updated) 
              {
                  dwin_need_update = 1;
                  pf_ui_already_updated = 1;
              }
          }
					else 
          {
              pf_ui_already_updated = 0;
              if (last_soc_x100 == 0xFFFF ||
                  abs((int16_t)(curr_soc_x100 - last_soc_x100)) >= 10 ||
                  abs((int16_t)(curr_soh_x100 - last_soh_x100)) >= 10 ||
                  abs(curr_current - last_current) >= 20 ||
                  abs((int16_t)(curr_stack_v - last_stack_v)) >= 20)            
              {
                  dwin_need_update = 1;
              }
              if (current_tick_2 - last_dwin_force_update >= 5000) {
                  dwin_need_update = 1;
                  last_dwin_force_update = current_tick_2;
              }
          }
			if (dwin_need_update) 
			{
					static uint8_t dwin_step = 0;
					switch (dwin_step)
					{
						case 0:
							{
								for (int i = 0; i < 16; i++) CellVoltage_V[i] = CellVoltage[i] / 1000.0f;
								dwin_step++;
								break;
							}
						case 1:		
							{
								DWIN_SendCellVoltages(CellVoltage_V);
								DWIN_SendSlaveCellVoltages();
								dwin_step++;
								break;
							}
						case 2:
							{
								float Pack_Voltage_V = Stack_Voltage / 1000.0f;
								float currentA = Pack_Current / 100.0f;
								DWIN_SendPackVoltage(Pack_Voltage_V);
								DWIN_SendTemperature(Temperature[0]);
								DWIN_SendCurrent(currentA);
								DWIN_SendSOC(SOC);
								DWIN_SendSOH(SOH);
								DWIN_SetSOCIcon(SOC);
								DWIN_BSetSOCIcon(SOC);
								DWIN_SendSlavePackVoltage();
								DWIN_SendSlaveCurrent();
								DWIN_SendSlaveSOC();               
								DWIN_SendSlaveSOH();               
								DWIN_SendSlaveRemainCapacity();    
								DWIN_SendSlaveCycleCount();
								DWIN_SetSlaveSOCIcon();
								DWIN_BSetSlaveSOCIcon();
								DWIN_SendSlaveRemainEnergy();
								DWIN_SendSystemTotalVoltage();      
								DWIN_SendSystemAvgSOH();            
								DWIN_SendSystemAvgSOC();            
								DWIN_SendSystemAvgCycles();         
								DWIN_SendSystemTotalCurrent();
								DWIN_SetSystemAvgSOCIcon();
								DWIN_BSetSystemAvgSOCIcon();
								DWIN_SendSystemTemperature();
								DWIN_SendSystemReCapa();
								DWIN_SendSystemRemainEnergy();
								dwin_step++;
								break;
							}
						case 3:
							{
								float RemainEnergy = Calculate_RemainEnergy(SOC, SOH);
								DWIN_SendRemainEnergy(RemainEnergy);
								DWIN_SendReCapa(remain_capa_Ah);
								DWIN_SendCycle(cycle_count);
								dwin_step++;
								break;
							}
						case 4:
							{
								DWIN_SendTemperature1(Temperature[0]);
								DWIN_SendTemperature2(Temperature[1]);
								DWIN_SendTemperature3(Temperature[2]);
								DWIN_SendTemperature4(Temperature[3]);
								DWIN_SendTemperature5(ntc_1);
								DWIN_SendTemperature6(ntc_2);
								DWIN_SendTemperatureFET(ntc_temp);
								DWIN_SendTemperatureIC(Temperature[6]);
								DWIN_SendSlaveTemperature0();
								dwin_step++;
								break;
							}
						case 5:
							{
								DWIN_UpdateAnyAlarm();
								DWIN_UpdateAlarms();
								DWIN_UpdateAnyProtection();
								DWIN_UpdateProtections();
								DWIN_ShowDualPackConnectionStatus();
								DWIN_UpdateSpecialProtection();
								DWIN_SendFETStatusDetail();
								DWIN_UpdatePermanentFail();        
								DWIN_UpdatePF_IndividualIcons();   
								DWIN_Control_SlavePageTouch();     
								last_soc_x100 = curr_soc_x100;
								last_soh_x100 = curr_soh_x100;
								last_current  = curr_current;
								last_stack_v  = curr_stack_v;
								dwin_step = 0;
								dwin_need_update = 0;
								break;
							}
						default:
							{
								dwin_step++;
								break;
							}
					}
			}
					static uint32_t last_force_save_tick = 0;
          if (last_force_save_tick == 0) last_force_save_tick = current_tick_2;
          uint8_t need_save = 0;
          if (bms_state != BMS_STATE_FAULT) 
          {
							uint16_t current_soc_threshold = adv_soc_clamp_active ? 500 : FLASH_CHANGE_THRESHOLD;
              if (abs((int16_t)(curr_soc_x100 - last_saved_soc_x100)) >= current_soc_threshold) need_save = 1;    
              if (abs((int16_t)(curr_soh_x100 - last_saved_soh_x100)) >= FLASH_CHANGE_THRESHOLD) need_save = 1;					
              if (cycle_count != last_saved_cycle) need_save = 1;					
              if (current_tick_2 - last_force_save_tick >= FLASH_SAVE_INTERVAL_MS) 
              {
                  need_save = 1;
                  last_force_save_tick = current_tick_2;
              }
							static uint8_t last_stop_state = 0;
              if (adv_stop_discharge_active != last_stop_state) {
                  if (adv_stop_discharge_active) need_save = 1;
                  last_stop_state = adv_stop_discharge_active;
              }
          }
          if (need_save && flash_init_success) 
          {
              BMS_Save_Data_To_Flash();
              need_save = 0;
          }
				}
			Safe_Delay_ms(5);
		#endif
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	/* The CAN filter configuration */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* The data will be received in FIFO0 */
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
	
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
 /* Starting the CAN peripheral */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
 /* Activate CAN RX notification on FIFO0 */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 312;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  __HAL_RCC_AFIO_CLK_ENABLE(); 
  __HAL_AFIO_REMAP_SWJ_NOJTAG(); 
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, POWER_HOLD_Pin|LED5_Pin|LED4_Pin|LED3_Pin
                          |LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RST_SHUT_Pin|POW_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin|CFETOFF_Pin|SRN_Pin|SRP_Pin
                          |APTOMAT_SWITCH_Pin|RS485_DE_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|DRY_CONTACT_LOWBAT_Pin|DRY_CONTACT_FAULT_Pin|DN_OP_Pin
                          |EN_PRECHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin|RS485_DE_RE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED9_Pin|LED6_Pin|LED7_Pin|LED8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_STATE_Pin */
  GPIO_InitStruct.Pin = BUTTON_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_HOLD_Pin RST_SHUT_Pin POW_EN_Pin LED5_Pin
                           LED4_Pin LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = POWER_HOLD_Pin|RST_SHUT_Pin|POW_EN_Pin|LED5_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DDSG_Pin */
  GPIO_InitStruct.Pin = DDSG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DDSG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCHG_Pin */
  GPIO_InitStruct.Pin = DCHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DCHG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DFETOFF_Pin CFETOFF_Pin SRN_Pin SRP_Pin
                           APTOMAT_SWITCH_Pin LED1_Pin */
  GPIO_InitStruct.Pin = DFETOFF_Pin|CFETOFF_Pin|SRN_Pin|SRP_Pin
                          |APTOMAT_SWITCH_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ALERT_Pin */
  GPIO_InitStruct.Pin = ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin DRY_CONTACT_LOWBAT_Pin DRY_CONTACT_FAULT_Pin DN_OP_Pin
                           EN_PRECHARGE_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|DRY_CONTACT_LOWBAT_Pin|DRY_CONTACT_FAULT_Pin|DN_OP_Pin
                          |EN_PRECHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_SWITCH_Pin K1_Pin K2_Pin K3_Pin
                           K4_Pin K5_Pin K6_Pin */
  GPIO_InitStruct.Pin = RST_SWITCH_Pin|K1_Pin|K2_Pin|K3_Pin
                          |K4_Pin|K5_Pin|K6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_INT_A_Pin */
  GPIO_InitStruct.Pin = RTC_INT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RTC_INT_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TEST_LED_Pin LED9_Pin LED6_Pin LED7_Pin
                           LED8_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin|LED9_Pin|LED6_Pin|LED7_Pin
                          |LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_RE_Pin */
  GPIO_InitStruct.Pin = RS485_DE_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_DE_RE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_RE1_Pin */
  GPIO_InitStruct.Pin = RS485_DE_RE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_DE_RE1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UP_IN_Pin */
  GPIO_InitStruct.Pin = UP_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(UP_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
# if IS_BOOTLOADER == 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
    if (htim->Instance == TIM2) 
    {
        LED_Task_1ms();
        Buzzer_Task_1ms();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == RTC_INT_A_Pin) {
        rtc_read_flag = 1;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    if (CanHandle == &hcan) 
    {
        if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        {
            Error_Handler();
        }

				if (Ota_data.start_ota == 1) {
        return;
				}
				
        if (RxHeader.IDE == CAN_ID_STD && RxHeader.StdId == 0x305) {
            last_inverter_alive_tick = HAL_GetTick();
            inverter_comm_fault = 0;
        }

        if (RxHeader.IDE == CAN_ID_EXT) {
            uint32_t ext_id = RxHeader.ExtId;

            if ((ext_id & 0xFF000000) == 0x04000000) {
                uint8_t req_group = (ext_id >> 16) & 0xFF; 
                uint8_t req_pack  = (ext_id >> 8) & 0xFF;  

                if (req_group == 1 && req_pack >= 1 && req_pack <= MAX_SLAVES + 1) {
                    
                    CAN_TxHeaderTypeDef ExtTxHeader;
                    uint8_t ExtTxData[8] = {0};
                    uint32_t ExtTxMailbox;
                    
                    ExtTxHeader.IDE = CAN_ID_EXT;
                    ExtTxHeader.RTR = CAN_RTR_DATA;
                    ExtTxHeader.DLC = 8;
                    
                    uint16_t p_min_cv = 0, p_max_cv = 0, p_vol = 0;
                    int16_t p_curr = 0, p_max_t = 0, p_min_t = 0, p_mos_t = 0, p_bms_t = 0;
                    uint8_t p_soc = 0, p_soh = 0;

                    if (req_pack == 1) { 

                        p_min_cv = sys_min_cell_v; 
                        p_max_cv = sys_max_cell_v;
                        p_curr = Pack_Current / 10;        
                        p_vol = Stack_Voltage / 10;        
                        p_soc = (uint8_t)SOC; 
                        p_soh = (uint8_t)SOH;
                        p_max_t = (system_max_temp_C * 10);  
                        p_min_t = (system_min_temp_C * 10);
                        p_mos_t = (int16_t)(ntc_temp * 10);
                        p_bms_t = (int16_t)(Temperature[6] * 10); 
                    } else {             

                        uint8_t s_idx = req_pack - 2; 
                        if (slave_online_status[s_idx] > 0) {
                            p_curr = slave_analog_data[s_idx].current / 10; 
                            p_vol = slave_analog_data[s_idx].total_voltage / 10;

                            uint32_t s_rem = (slave_analog_data[s_idx].remain_cap_2[0] << 16) | (slave_analog_data[s_idx].remain_cap_2[1] << 8) | slave_analog_data[s_idx].remain_cap_2[2];
                            uint32_t s_tot = (slave_analog_data[s_idx].total_cap_2[0] << 16) | (slave_analog_data[s_idx].total_cap_2[1] << 8) | slave_analog_data[s_idx].total_cap_2[2];
                            if (slave_analog_data[s_idx].user_def_count < 4) {
                                s_rem = slave_analog_data[s_idx].remain_cap_1 * 100;
                                s_tot = slave_analog_data[s_idx].total_cap_1 * 100;
                            }
                            p_soc = (s_tot > 0) ? (uint8_t)((s_rem * 100) / s_tot) : 100;
                            p_soh = (uint8_t)system_avg_soh; 

                            p_min_cv = 0xFFFF; p_max_cv = 0;
                            for (int c = 0; c < slave_analog_data[s_idx].cell_count; c++) {
                                uint16_t cv = slave_analog_data[s_idx].cell_voltages[c];
                                if (cv > p_max_cv) p_max_cv = cv;
                                if (cv < p_min_cv && cv > 500) p_min_cv = cv;
                            }

                            p_min_t = 0x7FFF; p_max_t = -0x7FFF;
                            for (int t = 0; t < slave_analog_data[s_idx].temp_count; t++) {
                                int16_t tv = slave_analog_data[s_idx].temperatures[t] - 2731;
                                if (tv > p_max_t) p_max_t = tv;
                                if (tv < p_min_t) p_min_t = tv;
                            }
                            p_mos_t = p_max_t; 
                            p_bms_t = p_max_t;
                        }
                    }

                    ExtTxHeader.ExtId = 0x04000001 + (req_pack << 8) + (req_group << 16);
                    ExtTxData[0] = p_min_cv & 0xFF; ExtTxData[1] = (p_min_cv >> 8) & 0xFF;
                    ExtTxData[2] = p_max_cv & 0xFF; ExtTxData[3] = (p_max_cv >> 8) & 0xFF;
                    ExtTxData[4] = p_curr & 0xFF;   ExtTxData[5] = (p_curr >> 8) & 0xFF;
                    ExtTxData[6] = p_vol & 0xFF;    ExtTxData[7] = (p_vol >> 8) & 0xFF;
                    if (HAL_CAN_GetTxMailboxesFreeLevel(CanHandle) > 0) {
                        HAL_CAN_AddTxMessage(CanHandle, &ExtTxHeader, ExtTxData, &ExtTxMailbox);
                    }

                    ExtTxHeader.ExtId = 0x04000002 + (req_pack << 8) + (req_group << 16);
                    ExtTxData[0] = p_max_t & 0xFF; ExtTxData[1] = (p_max_t >> 8) & 0xFF;
                    ExtTxData[2] = p_min_t & 0xFF; ExtTxData[3] = (p_min_t >> 8) & 0xFF;
                    ExtTxData[4] = p_mos_t & 0xFF; ExtTxData[5] = (p_mos_t >> 8) & 0xFF;
                    ExtTxData[6] = p_bms_t & 0xFF; ExtTxData[7] = (p_bms_t >> 8) & 0xFF;
                    if (HAL_CAN_GetTxMailboxesFreeLevel(CanHandle) > 0) {
                        HAL_CAN_AddTxMessage(CanHandle, &ExtTxHeader, ExtTxData, &ExtTxMailbox);
                    }

                    ExtTxHeader.ExtId = 0x04000003 + (req_pack << 8) + (req_group << 16);
                    memset(ExtTxData, 0, 8);
                    ExtTxData[2] = p_soc & 0xFF; 
                    ExtTxData[3] = (p_soc >> 8) & 0xFF; 
                    ExtTxData[4] = p_soh;
                    uint16_t pack_nom_cap;
                    if (req_pack == 1) {
                        pack_nom_cap = (uint16_t)(FullChargeCapacity_mAh / 1000);
                    } else {
											pack_nom_cap = (uint16_t)(slave_analog_data[req_pack-2].total_cap_1); 
                    }
                    ExtTxData[6] = pack_nom_cap & 0xFF; 
                    ExtTxData[7] = (pack_nom_cap >> 8) & 0xFF;
                    if (HAL_CAN_GetTxMailboxesFreeLevel(CanHandle) > 0) {
                        HAL_CAN_AddTxMessage(CanHandle, &ExtTxHeader, ExtTxData, &ExtTxMailbox);
                    }

                    ExtTxHeader.ExtId = 0x04000004 + (req_pack << 8) + (req_group << 16);
                    memset(ExtTxData, 0, 8);
                    if (HAL_CAN_GetTxMailboxesFreeLevel(CanHandle) > 0) {
                        HAL_CAN_AddTxMessage(CanHandle, &ExtTxHeader, ExtTxData, &ExtTxMailbox);
                    }
                }
            }
        }
    }
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{		
//    if (huart->Instance == USART1) {
//        (void)ring_push_head(&vrts_ringbuffer_Data, (uint8_t *)&rx_byte);
//        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);     
//    }
    if (Ota_data.start_ota == 1) {
        return;
    }
		#if(IS_BOOTLOADER == 0)
    if (huart->Instance == UART4) {
        BQ_Process_UART_Byte(&huart4, rx_byte_u4);
        HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_byte_u4, 1);
    }
    else if (huart->Instance == UART5) {
        BQ_Process_UART_Byte(&huart5, rx_byte_u5);
        HAL_UART_Receive_IT(&huart5, (uint8_t *)&rx_byte_u5, 1);
    }
    #endif	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

//        if (huart->Instance == USART1) {
//            HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
//            __HAL_UART_CLEAR_OREFLAG(&huart1);			
//        }
				#if(IS_BOOTLOADER == 0)
        if (Ota_data.start_ota == 0) {
            if (huart->Instance == UART4) {
                HAL_UART_Receive_IT(&huart4, (uint8_t *)&rx_byte_u4, 1);
            }
            else if (huart->Instance == UART5) {
                HAL_UART_Receive_IT(&huart5, (uint8_t *)&rx_byte_u5, 1);
            }
            else if (huart->Instance == USART3) {
                HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_byte_u3, 1);
                __HAL_UART_CLEAR_OREFLAG(&huart3);
            }
        }
        #endif
    }
}
#if(IS_BOOTLOADER == 0)
void HAL_PWR_PVD_Callback(void) {
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO)) {
				__disable_irq();
        if (flash_init_success)
        {
						flash_handle.delay_ms = Emergency_Delay;
            if (wait_flash_ready(1000) == 0) 
            {
                bms_flash_data_t emergency_data = {0};               
                emergency_data.soc_x100         = (uint16_t)roundf(SOC * 100.0f);
                emergency_data.soh_x100         = (uint16_t)roundf(SOH * 100.0f);
                emergency_data.cycle_count      = cycle_count;
                emergency_data.full_capa_mAh    = FullChargeCapacity_mAh;                
                current_sequence++; 
                emergency_data.sequence_number  = current_sequence;              
                emergency_data.mAh_at_last_sync           = mAh_at_last_sync;
                emergency_data.accumulated_discharge_mAh  = accumulated_discharge_mAh;
                emergency_data.last_full_charge_day       = last_full_charge_day;
                memcpy(emergency_data.pf_status, current_pf_status, 4);
                emergency_data.device_hash           = Calculate_Device_Hash();
                emergency_data.pylon_protocol_addr   = pylon_protocol_addr;
                emergency_data.is_master             = is_master;
                emergency_data.crc = calculate_crc32((uint8_t*)&emergency_data, sizeof(bms_flash_data_t) - sizeof(uint32_t));
                uint32_t write_addr = FLASH_EMERGENCY_START + emergency_write_offset;
                if (w25qxx_write(&flash_handle, write_addr, (uint8_t*)&emergency_data, 
                                sizeof(bms_flash_data_t)) == 0)
                {
                    emergency_write_offset += FLASH_EMERGENCY_PAGE;
                    if (emergency_write_offset >= FLASH_EMERGENCY_SIZE)
                    {
                        emergency_write_offset = 0;
                    }
                }
            }
        }
        HAL_GPIO_WritePin(GPIOD, TEST_LED_Pin, GPIO_PIN_SET);
				if (data_saved_by_button || system_is_shutting_down || 1) {
            HAL_GPIO_WritePin(GPIOC, POWER_HOLD_Pin, GPIO_PIN_RESET);
        }
				while(1) {
            Emergency_Delay(10);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}
#endif	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  for(int j = 0; j < 10; j++) {
      HAL_GPIO_TogglePin(GPIOD, TEST_LED_Pin); 
      for(volatile int i = 0; i < 500000; i++); 
  }
  NVIC_SystemReset(); 
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
