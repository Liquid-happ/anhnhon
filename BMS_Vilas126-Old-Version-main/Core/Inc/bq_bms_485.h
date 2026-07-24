#ifndef BQ_BMS_485_H
#define BQ_BMS_485_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "BQ769x2Header.h"
#include "bms_state.h"

/* --- PYLONTECH PROTOCOL CONSTANTS --- */
#define PYLON_SOI                       0x7E    // '~' Start Flag
#define PYLON_EOI                       0x0D    // '\r' End Flag
#define PYLON_VER                       0x20    // Version 2.0 (ASCII '20')
#define PYLON_CID1_BATTERY              0x46    // Battery Data (46H)
#define PYLON_DEFAULT_ADR               0x02    // Master Address Default

/* --- CID2 COMMANDS --- */
// Standard Commands (V3.3)
#define PYLON_CID2_MASTER_ANALOG 				0xA0		// Master Analog Data
#define PYLON_CID2_SOFT_RESET           0x9A		// Master Request Reset 
#define PYLON_CID2_ASSIGN_ID            0x41    // Custom Command
#define PYLON_CID2_ANALOG_DATA          0x42    // Get Analog Values
#define PYLON_CID2_ALARM_INFO           0x44    // Get Alarm Info
#define PYLON_CID2_SYSTEM_PARAM         0x47    // Get System Parameters
#define PYLON_CID2_PROTOCOL_VER         0x4F    // Get Protocol Version
#define PYLON_CID2_MANUFACTURER         0x51    // Get Manufacturer Info
#define PYLON_CID2_CHG_DIS_MGMT         0x92    // Get Charge/Discharge Mgmt
#define PYLON_CID2_GET_SN               0x93    // Get Serial Number
#define PYLON_CID2_SET_MGMT             0x94    // Set Mgmt Info
#define PYLON_CID2_TURN_OFF             0x95    // Turn Off Battery
#define PYLON_CID2_GET_FW               0x96    // Get Firmware Info

// System Level Commands (V3.5)
#define PYLON_CID2_SYS_BASIC            0x60    // System Basic Info
#define PYLON_CID2_SYS_ANALOG           0x61    // System Analog Info
#define PYLON_CID2_SYS_ALARM            0x62    // System Alarm Info
#define PYLON_CID2_SYS_MGMT             0x63    // System Mgmt Info
#define PYLON_CID2_SYS_SHUTDOWN         0x64    // System Shutdown

/* --- RETURN CODES --- */
#define PYLON_RTN_NORMAL                0x00    // No Error
#define PYLON_RTN_VER_ERR               0x01    // Version Error
#define PYLON_RTN_CHKSUM_ERR            0x02    // Checksum Error
#define PYLON_RTN_LCHKSUM_ERR           0x03    // LCHKSUM Error
#define PYLON_RTN_CID2_ERR              0x04    // Invalid CID2
#define PYLON_RTN_CMD_ERR               0x05    // Command Format Error
#define PYLON_RTN_DATA_ERR              0x06    // Invalid Information Data
#define PYLON_RTN_ADR_ERR               0x90    // Address Error
#define PYLON_RTN_COMM_ERR              0x91    // Internal Communication Error

#define PYLON_LCHK_MASK                 0xF000
#define PYLON_LENID_MASK                0x0FFF
#define MAX_SLAVES 											3
#define NOMINAL_CAPACITY_mAh            314000.0f

#pragma pack(push, 1)

/* =========================================================================
 * V3.3: PACK LEVEL COMMUNICATION STRUCTS
 * ========================================================================= */
 
typedef struct {
    char    	battery_name[10];
    uint16_t 	soft_version;      // Fixed to uint16_t per datasheet (2 bytes hex)
    char    	manufacturer_name[20];
} pylon_rs485_manufacturer_t;

typedef struct {
    uint8_t  command_value;
    uint8_t  cell_count;         
    uint16_t cell_voltages[16];  // V (mV)
    uint8_t  temp_count;         
    int16_t  temperatures[8];    // K (0.1K)
    int16_t  current;            // A (100mA signed)           
    uint16_t total_voltage;      // V (mV)
    uint16_t remain_cap_1;       // Ah
    uint8_t  user_def_count;     // 2 (<65Ah) or 4 (>65Ah)
    uint16_t total_cap_1;        // Ah 
    uint16_t cycle_count;
    uint8_t  remain_cap_2[3];    // Ah (Valid if user_def_count == 4)
    uint8_t  total_cap_2[3];     // Ah 
} pylon_rs485_analog_t;

typedef struct {
    uint16_t    cell_high_v_limit;
    uint16_t    cell_low_v_limit;
    uint16_t    cell_under_v_limit;
    int16_t     charge_high_temp;   
    int16_t     charge_low_temp;     
    int16_t     charge_current_lim;
    uint16_t    module_high_v_lim;
    uint16_t    module_low_v_lim;
    uint16_t    module_under_v_lim;
    int16_t     dischg_high_temp;   
    int16_t     dischg_low_temp;   
    int16_t     dischg_current_lim;
} pylon_rs485_system_param_t;

typedef struct {
    uint8_t module_ov    : 1;   // Bit 0: Module Over Voltage
    uint8_t cell_uv      : 1;   // Bit 1: Cell Under Voltage
    uint8_t charge_oc    : 1;   // Bit 2: Charge Over Current
    uint8_t reserved     : 1;   // Bit 3: Empty
    uint8_t discharge_oc : 1;   // Bit 4: Discharge Over Current
    uint8_t discharge_ot : 1;   // Bit 5: Discharge Over Temp
    uint8_t charge_ot    : 1;   // Bit 6: Charge Over Temp
    uint8_t module_uv    : 1;   // Bit 7: Module Under Voltage
} pylon_status1_t;

typedef struct {
    uint8_t pre_mosfet   : 1;   // Bit 0: Pre MOSFET
    uint8_t charge_mos   : 1;   // Bit 1: Charge MOS
    uint8_t discharge_mos: 1;   // Bit 2: Discharge MOS
    uint8_t module_power : 1;   // Bit 3: Using Pack Power
    uint8_t reserved     : 4;   // Bit 4-7
} pylon_status2_t;

typedef struct {
    uint8_t buzzer       : 1;   // Bit 0: Buzzer
    uint8_t reserved1    : 2;   // Bit 1-2: Empty
    uint8_t fully_charged: 1;   // Bit 3: Fully Charged (SOC=100%)
    uint8_t system_error : 1;   // Bit 4: (PF/System Error)
    uint8_t heater       : 1;   // Bit 5: Heater
    uint8_t eff_dischg_curr: 1; // Bit 6: Effective Discharge Current
    uint8_t eff_charge_curr: 1; // Bit 7: Effective Charge Current
} pylon_status3_t;

typedef struct {
    uint8_t cell1 			 : 1;
    uint8_t cell2 			 : 1;
    uint8_t cell3 			 : 1;
    uint8_t cell4  			 : 1;
    uint8_t cell5 			 : 1;
    uint8_t cell6 			 : 1;
    uint8_t cell7 			 : 1;
    uint8_t cell8 			 : 1;
} pylon_status4_t;

typedef struct {
    uint8_t cell9  			 : 1;
    uint8_t cell10 			 : 1;
    uint8_t cell11 			 : 1;
    uint8_t cell12 			 : 1;
    uint8_t cell13 			 : 1;
    uint8_t cell14 			 : 1;
    uint8_t cell15 			 : 1;
    uint8_t cell16 			 : 1;
} pylon_status5_t;

typedef struct {
    uint8_t  command_value;
    uint8_t  cell_count;
    uint8_t  cell_status[16];     
    uint8_t  temp_count;
    uint8_t  temp_status[8];
    uint8_t  charge_curr_status;
    uint8_t  module_volt_status;
    uint8_t  discharge_curr_status;
    pylon_status1_t s1;           // State 1
    pylon_status2_t s2;           // State 2
    pylon_status3_t s3;           // State 3
    pylon_status4_t s4;           // State 4 
    pylon_status5_t s5;           // State 5        
} pylon_rs485_alarm_t;

typedef struct {
    uint8_t reserved         : 2;   // Bit 0-1
		uint8_t precharge_en		 : 1;		// Bit 2
    uint8_t full_chg_req     : 1;   // Bit 3
    uint8_t force_chg_ii     : 1;   // Bit 4
    uint8_t force_chg_i      : 1;   // Bit 5
    uint8_t discharge_en     : 1;   // Bit 6
    uint8_t charge_en        : 1;   // Bit 7
} pylon_rs485_chg_dsg_status_t;

typedef struct {
    uint8_t  command_value;
    uint16_t charge_voltage_limit;    // mV
    uint16_t discharge_voltage_limit; // mV
    int16_t  charge_current_limit;    // 100mA
    int16_t  discharge_current_limit; // 100mA
    pylon_rs485_chg_dsg_status_t stt; 
} pylon_rs485_chg_dis_mgmt_t;

typedef struct {
    uint8_t  command_value;  
    char     sn_number[16];    
} pylon_rs485_sn_t;

typedef struct {
    uint8_t  command_value;
    uint16_t set_charge_v_limit;    
    uint16_t set_discharge_v_limit;   
    int16_t  set_charge_i_limit;     
    int16_t  set_discharge_i_limit;
		pylon_rs485_chg_dsg_status_t stt;
} pylon_rs485_set_mgmt_t;

typedef struct {
    uint8_t  command_value;              
    uint8_t  manufacturer_version[2]; 
    uint8_t  main_line_version[3];      
} pylon_rs485_soft_ver_full_t;

/* =========================================================================
 * V3.5: SYSTEM LEVEL COMMUNICATION STRUCTS (MASTER ONLY)
 * ========================================================================= */

typedef struct {
    char     battery_name[10];      
    char     manufacturer_name[20]; 
    uint16_t software_version;      
    uint8_t  battery_number;        
    char     barcodes[15][16];      // Support up to 15 modules safely
} pylon_rs485_sys_basic_t;

typedef struct {
    uint16_t sys_total_voltage; 
    int16_t  sys_total_current; 
    uint8_t  sys_soc;           
    uint16_t avg_cycles;
    uint16_t max_cycles;
    uint8_t  avg_soh;
    uint8_t  min_soh;
    uint16_t max_cell_v;
    uint16_t max_cell_v_mod;
    uint16_t min_cell_v;
    uint16_t min_cell_v_mod;
    int16_t  avg_cell_temp;
    int16_t  max_cell_temp;
    uint16_t max_cell_temp_mod;
    int16_t  min_cell_temp;
    uint16_t min_cell_temp_mod;
    int16_t  avg_mos_temp;       
    int16_t  max_mos_temp;      
    uint16_t max_mos_temp_mod;   
    int16_t  min_mos_temp;       
    uint16_t min_mos_temp_mod;   
    int16_t  avg_bms_temp;       
    int16_t  max_bms_temp;       
    uint16_t max_bms_temp_mod;   
    int16_t  min_bms_temp;       
    uint16_t min_bms_temp_mod;   
} pylon_rs485_sys_analog_t;

typedef struct {
    uint8_t cell_v_inconsistency : 1; 
    uint8_t mos_high_temp        : 1; 
    uint8_t cell_low_temp        : 1; 
    uint8_t cell_high_temp       : 1; 
    uint8_t cell_low_v           : 1; 
    uint8_t cell_high_v          : 1; 
    uint8_t module_low_v         : 1; 
    uint8_t module_high_v        : 1; 
} pylon_rs485_sys_alarm1_t;

typedef struct {
    uint8_t reserved             : 4; 
    uint8_t internal_comm_err    : 1; 
    uint8_t discharge_oc_alarm   : 1; 
    uint8_t charge_oc_alarm      : 1; 
    uint8_t cell_t_inconsistency : 1; 
} pylon_rs485_sys_alarm2_t;

typedef struct {
    uint8_t reserved             : 1; 
    uint8_t mos_over_temp        : 1; 
    uint8_t cell_under_temp      : 1; 
    uint8_t cell_over_temp       : 1; 
    uint8_t cell_under_v         : 1; 
    uint8_t cell_over_v          : 1; 
    uint8_t module_under_v       : 1; 
    uint8_t module_over_v        : 1; 
} pylon_rs485_sys_protect1_t;

typedef struct {
    uint8_t reserved             : 3; 
    uint8_t system_fault         : 1; 
    uint8_t reserved1            : 1; 
    uint8_t discharge_oc_prot    : 1; 
    uint8_t charge_oc_prot       : 1; 
    uint8_t reserved2            : 1; 
} pylon_rs485_sys_protect2_t;

typedef struct {
    pylon_rs485_sys_alarm1_t      alarm1;
    pylon_rs485_sys_alarm2_t      alarm2;      
    pylon_rs485_sys_protect1_t    protect1;    
    pylon_rs485_sys_protect2_t    protect2;    
} pylon_rs485_sys_alarm_t;

typedef struct {
    uint8_t reserved         : 4;   // Bit 0-3
    uint8_t full_chg_req     : 1;   // Bit 4
    uint8_t force_chg        : 1;   // Bit 5
    uint8_t discharge_en     : 1;   // Bit 6
    uint8_t charge_en        : 1;   // Bit 7
} pylon_rs485_sys_chg_dsg_status_t;

typedef struct {
    uint16_t sys_charge_v_limit;      
    uint16_t sys_discharge_v_limit;    
    int16_t  sys_charge_i_limit; 
    int16_t  sys_discharge_i_limit; 
    pylon_rs485_sys_chg_dsg_status_t sys_status;   
} pylon_rs485_sys_chg_dis_mgmt_t;

#pragma pack(pop)

typedef enum {
    AUTO_CODE_START = 0,        
    AUTO_CODE_WAIT_UP_IN,        
    AUTO_CODE_IS_MASTER,         
    AUTO_CODE_MASTER_ASSIGNING,  
    AUTO_CODE_WAIT_ADDRESS_CMD,  
    AUTO_CODE_DONE               
} Auto_Code_State_t;

extern Auto_Code_State_t auto_code_state;
extern uint8_t is_auto_coding;
extern volatile uint8_t assigned_n;
extern uint8_t ProtectionsTriggered;
extern uint8_t value_SafetyStatusA;
extern uint8_t value_SafetyStatusB;
extern uint8_t value_SafetyStatusC;
extern uint8_t value_SafetyAlertA;
extern uint8_t value_SafetyAlertB;
extern uint8_t value_SafetyAlertC;
extern uint8_t value_PFStatusA;
extern uint8_t value_PFStatusB;
extern uint8_t value_PFStatusC;
extern uint8_t value_PFStatusD;
extern uint8_t UV_Fault;
extern uint8_t OV_Fault;
extern uint8_t COVL_Fault;
extern uint8_t OCC_Fault;
extern uint8_t OCD_Fault;
extern uint8_t SCD_Fault;
extern uint8_t OTC_Fault;
extern uint8_t OTD_Fault;
extern uint8_t OTF_Fault;
extern uint8_t UTC_Fault;
extern uint8_t UTD_Fault;
extern uint8_t OCDL_Fault;
extern uint8_t SCDL_Fault;
extern uint8_t COVL_Fault;
extern uint8_t OCD_Fault1;
extern uint8_t OTINT_Fault;
extern uint8_t UTINT_Fault;
extern uint8_t sys_protect_ov;
extern uint8_t sys_protect_uv;
extern uint8_t sys_protect_occ;
extern uint8_t sys_protect_ocd;
extern uint8_t sys_protect_ot;
extern uint8_t sys_protect_ut;
extern uint8_t sys_alarm_ov;
extern uint8_t sys_alarm_uv;
extern uint8_t sys_alarm_occ;
extern uint8_t sys_alarm_ocd;
extern uint8_t sys_alarm_ot;
extern uint8_t sys_alarm_ut;
extern uint8_t PFErrorsTriggered;
/* --- EXTERNAL VARIABLES --- */

extern BuzzerMode_t current_buzzer_mode;
extern uint8_t bms_device_address;
extern uint8_t pylon_protocol_addr;
extern uint8_t current_assigning_index;
extern float FullChargeCapacity_mAh;  
extern uint8_t is_master;
extern uint16_t CellVoltage[16];
extern uint16_t Stack_Voltage;
extern int16_t Pack_Current;
extern float SOC;
extern float SOH;
extern uint32_t cycle_count;
extern int32_t  system_total_current_mA;
extern uint16_t system_avg_soc;
extern uint16_t system_avg_soh;
extern uint16_t system_avg_cycles;
extern int16_t system_max_temp_C;
extern int16_t system_min_temp_C;
extern uint8_t  active_packs_count;
extern uint16_t local_max_cell_v;
extern uint16_t local_min_cell_v;
extern uint8_t  local_max_cell_id;
extern uint8_t  local_min_cell_id;
extern uint16_t sys_max_cell_v;
extern uint16_t sys_min_cell_v;
extern uint8_t  sys_max_v_pack_id;
extern uint8_t  sys_max_v_cell_id;
extern uint8_t  sys_min_v_pack_id;
extern uint8_t  sys_min_v_cell_id;
extern uint8_t  sys_max_t_pack_id;
extern uint8_t  sys_max_t_cell_id;
extern uint8_t  sys_min_t_pack_id;
extern uint8_t  sys_min_t_cell_id;
extern uint32_t total_system_remain_capacity_mAh;
extern uint32_t total_system_full_capacity_mAh;
extern uint16_t system_min_cell_voltage;
extern uint16_t system_max_cell_voltage;
extern int32_t  system_total_current_01A;
extern float Temperature[9];
extern float FET_Temperature;
extern float ntc_temp;
extern float ntc_1;
extern float ntc_2;
extern uint8_t system_is_shutting_down;
extern uint8_t CHG; 
extern uint8_t DSG;
extern uint8_t m;
extern uint16_t system_charge_limit_A;
extern uint16_t system_discharge_limit_A;
extern uint16_t system_charge_v_limit_mV;
extern uint16_t system_discharge_v_limit_mV;
extern uint8_t full_charge_request;
extern uint8_t force_charge;
extern uint8_t uv_recovery_locked;
extern uint8_t ov_recovery_locked;
extern pylon_rs485_analog_t master_analog_data;
extern pylon_rs485_set_mgmt_t last_master_cmd;
extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
extern pylon_rs485_chg_dis_mgmt_t slave_mgmt_data[MAX_SLAVES];
extern uint8_t slave_online_status[MAX_SLAVES];
extern pylon_rs485_alarm_t slave_alarm_data[MAX_SLAVES];

uint16_t pylon_485_calc_chksum(const uint8_t* payload, uint16_t len);
uint16_t pylon_485_calc_lchksum(uint16_t lenid);
extern uint32_t last_master_cmd_tick;
extern uint8_t has_received_master_cmd;

int pylon_rs485_analog_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_analog_t *src);
int pylon_rs485_alarm_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_alarm_t *src);
int pylon_rs485_system_param_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_system_param_t *src);
int pylon_rs485_manufacturer_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_manufacturer_t *src);
int pylon_rs485_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_chg_dis_mgmt_t *src);
int pylon_rs485_sn_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sn_t *src);
int pylon_rs485_set_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_set_mgmt_t *src);
int pylon_rs485_turn_off_pack(uint8_t *dst_ascii, uint16_t max_len, uint8_t command_value);
int pylon_rs485_soft_ver_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_soft_ver_full_t *src);
int pylon_rs485_sys_basic_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_basic_t *src);
int pylon_rs485_sys_analog_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_analog_t *src);
int pylon_rs485_sys_alarm_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_alarm_t *src);
int pylon_rs485_sys_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_chg_dis_mgmt_t *src);
int pylon_rs485_sys_shutdown_pack(uint8_t *dst_ascii, uint16_t max_len, uint8_t turn_off_flag);
void Pylon_Send_Assign_ID(UART_HandleTypeDef *huart, uint8_t id_to_assign);
void BQ_Process_UART_Byte(UART_HandleTypeDef *huart, uint8_t byte);
HAL_StatusTypeDef Pylon_Request_Data(UART_HandleTypeDef *huart, uint8_t target_adr, uint8_t cid2);
HAL_StatusTypeDef Pylon_Request_Set_Mgmt(UART_HandleTypeDef *huart, uint8_t target_adr, pylon_rs485_set_mgmt_t *cmd);
HAL_StatusTypeDef Pylon_Request_Soft_Reset(UART_HandleTypeDef *huart, uint8_t target_adr, uint8_t payload);

#endif /* BQ_BMS_485_H */
