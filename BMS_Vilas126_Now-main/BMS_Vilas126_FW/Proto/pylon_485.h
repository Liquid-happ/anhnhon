#ifndef PYLON_485_H
#define PYLON_485_H

#include "Cfg/feat_cfg.h"

#if IS_BOOTLOADER == 0

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* --- PYLONTECH PROTOCOL CONSTANTS --- */
#define PYLON_SOI                       0x7E    // '~' Start Flag
#define PYLON_EOI                       0x0D    // '\r' End Flag
#define PYLON_VER                       0x20    // Version 2.0 (ASCII '20')
#define PYLON_CID1_BATTERY              0x46    // Battery Data (46H)
#define PYLON_DEFAULT_ADR               0x02    // Master Address Default

/* --- CID2 COMMANDS --- */
// Standard Commands (V3.3)
#define PYLON_CID2_MASTER_ANALOG        0xA0    // Master Analog Data
#define PYLON_CID2_SOFT_RESET           0x9A    // Master Request Reset 
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
#define MAX_SLAVES                      3
#define NOMINAL_CAPACITY_mAh            314000.0f

#pragma pack(push, 1)

/* =========================================================================
 * V3.3: PACK LEVEL COMMUNICATION STRUCTS
 * ========================================================================= */
 
typedef struct {
    char        battery_name[10];
    uint16_t    soft_version;      // Fixed to uint16_t per datasheet (2 bytes hex)
    char        manufacturer_name[20];
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
    uint8_t cell1            : 1;
    uint8_t cell2            : 1;
    uint8_t cell3            : 1;
    uint8_t cell4            : 1;
    uint8_t cell5            : 1;
    uint8_t cell6            : 1;
    uint8_t cell7            : 1;
    uint8_t cell8            : 1;
} pylon_status4_t;

typedef struct {
    uint8_t cell9            : 1;
    uint8_t cell10           : 1;
    uint8_t cell11           : 1;
    uint8_t cell12           : 1;
    uint8_t cell13           : 1;
    uint8_t cell14           : 1;
    uint8_t cell15           : 1;
    uint8_t cell16           : 1;
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
    uint8_t precharge_en     : 1;   // Bit 2
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

/* --- PROTOCOL SERIALIZATION UTILITIES --- */
uint32_t pylon_ascii_hex_to_uint(const uint8_t *buf, uint8_t len);
uint16_t pylon_485_calc_chksum(const uint8_t* payload, uint16_t len);
uint16_t pylon_485_calc_lchksum(uint16_t lenid);

/* --- FORMATING UTILITIES --- */
void pylon_rs485_byte_to_hex(uint8_t val, uint8_t *buf);
void pylon_rs485_uint16_to_hex(uint16_t val, uint8_t *buf);

/* --- FRAME BUILDER --- */
int pylon_rs485_build_frame(uint8_t *dst_frame, uint16_t max_len, uint8_t ver, uint8_t adr, uint8_t cid1, uint8_t rtn_cid2, const uint8_t *info_ascii, uint16_t info_len);

/* --- PACKING FUNCTIONS --- */
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

/* --- UNPACKING FUNCTIONS --- */
void pylon_rs485_analog_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_analog_t *dst);
void pylon_rs485_alarm_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_alarm_t *dst);
void pylon_rs485_system_param_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_system_param_t *dst);
void pylon_rs485_manufacturer_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_manufacturer_t *dst);
void pylon_rs485_mgmt_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_chg_dis_mgmt_t *dst);
void pylon_rs485_sn_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sn_t *dst);
void pylon_rs485_set_mgmt_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_set_mgmt_t *dst);
int pylon_rs485_turn_off_unpack(const uint8_t *info_ascii, uint16_t info_len, uint8_t *dst_command_value);
void pylon_rs485_soft_ver_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_soft_ver_full_t *dst);
void pylon_rs485_sys_basic_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_basic_t *dst);
void pylon_rs485_sys_analog_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_analog_t *dst);
void pylon_rs485_sys_alarm_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_alarm_t *dst);
void pylon_rs485_sys_mgmt_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_chg_dis_mgmt_t *dst);
int pylon_rs485_sys_shutdown_unpack(const uint8_t *info_ascii, uint16_t info_len, uint8_t *dst_turn_off_flag);

#endif /* IS_BOOTLOADER == 0 */
#endif /* PYLON_485_H */
