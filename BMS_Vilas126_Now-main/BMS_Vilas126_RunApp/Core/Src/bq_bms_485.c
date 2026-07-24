#include "bq_bms_485.h"

#if IS_BOOTLOADER == 0

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SLAVES 3

/* --- GLOBAL ARRAYS --- */
pylon_rs485_analog_t master_analog_data 								= {0};
pylon_rs485_set_mgmt_t last_master_cmd					 				= {0};
pylon_rs485_analog_t slave_analog_data[MAX_SLAVES] 			= {0};
pylon_rs485_chg_dis_mgmt_t slave_mgmt_data[MAX_SLAVES] 	= {0};
uint8_t slave_online_status[MAX_SLAVES] 								= {0};
pylon_rs485_alarm_t slave_alarm_data[MAX_SLAVES] 				= {0};

uint32_t last_master_cmd_tick = 0;
uint8_t has_received_master_cmd = 0;

/* --- UART BUFFERS --- */
extern volatile uint8_t rx_byte_u4; 
extern volatile uint8_t rx_byte_u5;

#define RX_BUFFER_SIZE 512
static uint8_t rx_buff_uart4[RX_BUFFER_SIZE];
volatile uint16_t rx_idx_uart4 = 0;
static uint8_t rx_buff_uart5[RX_BUFFER_SIZE];
volatile uint16_t rx_idx_uart5 = 0;

volatile uint32_t err_len_field = 0;
volatile uint32_t err_chksum_alarm = 0;
volatile uint32_t err_chksum_mgmt = 0;
volatile uint8_t rs485_rx_success_flag = 0;

/* --- GPIO MACROS --- */
#define RS485_UART4_TX_EN() HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_SET)
#define RS485_UART4_RX_EN() HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET)
#define RS485_UART5_TX_EN() HAL_GPIO_WritePin(GPIOD, RS485_DE_RE1_Pin, GPIO_PIN_SET)
#define RS485_UART5_RX_EN() HAL_GPIO_WritePin(GPIOD, RS485_DE_RE1_Pin, GPIO_PIN_RESET)

/* --- SERIAL NUMBER (16 Chars) --- */
static const char BMS_SERIAL_NUMBER[17] = "PYLNBMS314AH0001";

/* --- UTILITY FUNCTIONS --- */
uint32_t pylon_ascii_hex_to_uint(const uint8_t *buf, uint8_t len) {
    uint32_t val = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        uint8_t digit = 0;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else continue;     
        val = (val << 4) | digit;
    }
    return val;
}

static int16_t ascii_hex_to_int16(const uint8_t *buf) {
    return (int16_t)pylon_ascii_hex_to_uint(buf, 4);
}

static void byte_to_ascii_hex(uint8_t val, uint8_t *buf) {
    static const char hex_chars[] = "0123456789ABCDEF";
    buf[0] = (uint8_t)hex_chars[(val >> 4) & 0x0F];
    buf[1] = (uint8_t)hex_chars[val & 0x0F];
}

static void uint16_to_ascii_hex(uint16_t val, uint8_t *buf) {
    byte_to_ascii_hex((uint8_t)(val >> 8), buf);      
    byte_to_ascii_hex((uint8_t)(val & 0xFF), buf + 2); 
}

static uint16_t pylon_checksum_internal(const uint8_t *data, uint16_t len) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return (uint16_t)((~sum + 1) & 0xFFFF);
}

uint16_t pylon_485_calc_lchksum(uint16_t len_val) {
    uint16_t len_12bit = len_val & 0x0FFF;
    uint16_t sum = ((len_12bit >> 8) & 0x000F) + ((len_12bit >> 4) & 0x000F) + (len_12bit & 0x000F);
    uint16_t lchk = (~(sum % 16) + 1) & 0x000F;
    return (lchk << 12) | len_12bit;
}

uint16_t pylon_485_calc_chksum(const uint8_t* payload, uint16_t len) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint16_t)((~sum + 1) & 0xFFFF);
}

/* --- PACKING FUNCTIONS --- */
int pylon_rs485_analog_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_analog_t *src) {
    if (max_len < 138) return -1;
    uint16_t pos = 0;
		byte_to_ascii_hex(0x00, &dst_ascii[pos]); pos += 2; 
    byte_to_ascii_hex(src->command_value, &dst_ascii[pos]); pos += 2;    
    byte_to_ascii_hex(src->cell_count, &dst_ascii[pos]); pos += 2;
    for(int i = 0; i < src->cell_count; i++) {
        uint16_to_ascii_hex(src->cell_voltages[i], &dst_ascii[pos]); pos += 4;
    }   
    byte_to_ascii_hex(src->temp_count, &dst_ascii[pos]); pos += 2;
    for(int i = 0; i < src->temp_count; i++) {
        uint16_to_ascii_hex((uint16_t)src->temperatures[i], &dst_ascii[pos]); pos += 4;
    }
    uint16_to_ascii_hex((uint16_t)src->current, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->total_voltage, &dst_ascii[pos]); pos += 4;
		if (src->user_def_count == 4) {
        uint16_to_ascii_hex(0xFFFF, &dst_ascii[pos]); pos += 4; 
        byte_to_ascii_hex(src->user_def_count, &dst_ascii[pos]); pos += 2;
        uint16_to_ascii_hex(0xFFFF, &dst_ascii[pos]); pos += 4; 
    } else {
        uint16_to_ascii_hex(src->remain_cap_1, &dst_ascii[pos]); pos += 4;
        byte_to_ascii_hex(src->user_def_count, &dst_ascii[pos]); pos += 2;
        uint16_to_ascii_hex(src->total_cap_1, &dst_ascii[pos]); pos += 4;
    }
    uint16_to_ascii_hex(src->cycle_count, &dst_ascii[pos]); pos += 4; 
    if (src->user_def_count == 4) {
        byte_to_ascii_hex(src->remain_cap_2[0], &dst_ascii[pos]); pos += 2;
        byte_to_ascii_hex(src->remain_cap_2[1], &dst_ascii[pos]); pos += 2;
        byte_to_ascii_hex(src->remain_cap_2[2], &dst_ascii[pos]); pos += 2;
        byte_to_ascii_hex(src->total_cap_2[0], &dst_ascii[pos]); pos += 2;
        byte_to_ascii_hex(src->total_cap_2[1], &dst_ascii[pos]); pos += 2;
        byte_to_ascii_hex(src->total_cap_2[2], &dst_ascii[pos]); pos += 2;
    }
    return (int)pos;
}

int pylon_rs485_alarm_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_alarm_t *src) {
    if (max_len < 74) return -1;
    uint16_t pos = 0;
		byte_to_ascii_hex(0x00, &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->cell_count, &dst_ascii[pos]); pos += 2;
    for (int i = 0; i < src->cell_count; i++) {
        byte_to_ascii_hex(src->cell_status[i], &dst_ascii[pos]); pos += 2;
    }        
    byte_to_ascii_hex(src->temp_count, &dst_ascii[pos]); pos += 2;
		byte_to_ascii_hex(0x00, &dst_ascii[pos]); pos += 2; 
    for (int i = 0; i < src->temp_count; i++) {
        byte_to_ascii_hex(src->temp_status[i], &dst_ascii[pos]); pos += 2;
    }       
    byte_to_ascii_hex(src->charge_curr_status, &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->module_volt_status, &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->discharge_curr_status, &dst_ascii[pos]); pos += 2;

    uint8_t s1 = 0;
    if(src->s1.module_ov)    s1 |= 0x01;
    if(src->s1.cell_uv)      s1 |= 0x02;
    if(src->s1.charge_oc)    s1 |= 0x04;
    if(src->s1.discharge_oc) s1 |= 0x10;
    if(src->s1.discharge_ot) s1 |= 0x20;
    if(src->s1.charge_ot)    s1 |= 0x40;
    if(src->s1.module_uv)    s1 |= 0x80;
    byte_to_ascii_hex(s1, &dst_ascii[pos]); pos += 2;

    uint8_t s2 = 0;
    if(src->s2.pre_mosfet)    s2 |= 0x01;
    if(src->s2.charge_mos)    s2 |= 0x02;
    if(src->s2.discharge_mos) s2 |= 0x04;
    if(src->s2.module_power)  s2 |= 0x08;
    byte_to_ascii_hex(s2, &dst_ascii[pos]); pos += 2;

    uint8_t s3 = 0;
    if(src->s3.buzzer)          s3 |= 0x01;
    if(src->s3.fully_charged)   s3 |= 0x08;
		if(src->s3.system_error)    s3 |= 0x10;
    if(src->s3.heater)          s3 |= 0x20;
    if(src->s3.eff_dischg_curr) s3 |= 0x40;
    if(src->s3.eff_charge_curr) s3 |= 0x80;
    byte_to_ascii_hex(s3, &dst_ascii[pos]); pos += 2;

    uint8_t s4 = 0;
    if(src->s4.cell1) s4 |= 0x01; if(src->s4.cell2) s4 |= 0x02;
    if(src->s4.cell3) s4 |= 0x04; if(src->s4.cell4) s4 |= 0x08;
    if(src->s4.cell5) s4 |= 0x10; if(src->s4.cell6) s4 |= 0x20;
    if(src->s4.cell7) s4 |= 0x40; if(src->s4.cell8) s4 |= 0x80;
    byte_to_ascii_hex(s4, &dst_ascii[pos]); pos += 2;

    uint8_t s5 = 0;
    if(src->s5.cell9)  s5 |= 0x01; if(src->s5.cell10) s5 |= 0x02;
    if(src->s5.cell11) s5 |= 0x04; if(src->s5.cell12) s5 |= 0x08;
    if(src->s5.cell13) s5 |= 0x10; if(src->s5.cell14) s5 |= 0x20;
    if(src->s5.cell15) s5 |= 0x40; if(src->s5.cell16) s5 |= 0x80;
    byte_to_ascii_hex(s5, &dst_ascii[pos]); pos += 2;

    return (int)pos;
}

int pylon_rs485_system_param_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_system_param_t *src) {
    if (max_len < 50) return -1;
    uint16_t pos = 0;
		byte_to_ascii_hex(0x00, &dst_ascii[pos]); pos += 2;
    uint16_to_ascii_hex(src->cell_high_v_limit,  &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->cell_low_v_limit,   &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->cell_under_v_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->charge_high_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->charge_low_temp,  &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->charge_current_lim, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->module_high_v_lim,  &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->module_low_v_lim,   &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->module_under_v_lim, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->dischg_high_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->dischg_low_temp,  &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->dischg_current_lim, &dst_ascii[pos]); pos += 4;
    return (int)pos;
}

int pylon_rs485_manufacturer_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_manufacturer_t *src) {
    if (max_len < 64) return -1;
    uint16_t pos = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t c = (src->battery_name[i] != '\0') ? (uint8_t)src->battery_name[i] : 0x20;
        byte_to_ascii_hex(c, &dst_ascii[pos]); pos += 2;
    }
    uint16_to_ascii_hex(src->soft_version, &dst_ascii[pos]); pos += 4;
    for (int i = 0; i < 20; i++) {
        uint8_t c = (src->manufacturer_name[i] != '\0') ? (uint8_t)src->manufacturer_name[i] : 0x20;
        byte_to_ascii_hex(c, &dst_ascii[pos]); pos += 2;
    }
    return (int)pos; 
}

int pylon_rs485_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_chg_dis_mgmt_t *src) {
    if (max_len < 20) return -1;
    uint16_t pos = 0;

    byte_to_ascii_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    uint16_to_ascii_hex(src->charge_voltage_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->discharge_voltage_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->charge_current_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->discharge_current_limit, &dst_ascii[pos]); pos += 4;

    uint8_t st = 0;
		if (src->stt.precharge_en) st |= 0x04; 
    if (src->stt.full_chg_req) st |= 0x08; 
    if (src->stt.force_chg_ii) st |= 0x10;
    if (src->stt.force_chg_i)  st |= 0x20; 
    if (src->stt.discharge_en) st |= 0x40; 
    if (src->stt.charge_en)    st |= 0x80; 
    byte_to_ascii_hex(st, &dst_ascii[pos]); pos += 2;
    return (int)pos;
}

int pylon_rs485_sn_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sn_t *src) {
    if (max_len < 34) return -1;
    uint16_t pos = 0;
    byte_to_ascii_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    for(int i = 0; i < 16; i++) {
        uint8_t c = (src->sn_number[i] != '\0') ? (uint8_t)src->sn_number[i] : 0x20;
        byte_to_ascii_hex(c, &dst_ascii[pos]); pos += 2;
    }
    return (int)pos;
}

int pylon_rs485_set_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_set_mgmt_t *src) {
    if (max_len < 20) return -1;
    uint16_t pos = 0;
    byte_to_ascii_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    uint16_to_ascii_hex(src->set_charge_v_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->set_discharge_v_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->set_charge_i_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->set_discharge_i_limit, &dst_ascii[pos]); pos += 4;
		
		uint8_t st = 0;
    if (src->stt.precharge_en) st |= 0x04; 
    if (src->stt.full_chg_req) st |= 0x08; 
    if (src->stt.force_chg_ii) st |= 0x10;
    if (src->stt.force_chg_i)  st |= 0x20; 
    if (src->stt.discharge_en) st |= 0x40; 
    if (src->stt.charge_en)    st |= 0x80; 
    byte_to_ascii_hex(st, &dst_ascii[pos]); pos += 2;
	
    return (int)pos;
}

int pylon_rs485_turn_off_pack(uint8_t *dst_ascii, uint16_t max_len, uint8_t command_value) {
    if (max_len < 2) return -1;
    byte_to_ascii_hex(command_value, dst_ascii);
    return 2;
}

int pylon_rs485_soft_ver_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_soft_ver_full_t *src) {
    if (max_len < 12) return -1;
    uint16_t pos = 0;
    byte_to_ascii_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->manufacturer_version[0], &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->manufacturer_version[1], &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->main_line_version[0], &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->main_line_version[1], &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->main_line_version[2], &dst_ascii[pos]); pos += 2;
    return (int)pos;
}

int pylon_rs485_sys_basic_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_basic_t *src) {
		uint16_t required_len = 66 + (src->battery_number * 32);
    if (max_len < required_len) return -1; 
    uint16_t pos = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t c = (src->battery_name[i] != '\0') ? (uint8_t)src->battery_name[i] : 0x20;
        byte_to_ascii_hex(c, &dst_ascii[pos]); pos += 2;
    }
    for (int i = 0; i < 20; i++) {
        uint8_t c = (src->manufacturer_name[i] != '\0') ? (uint8_t)src->manufacturer_name[i] : 0x20;
        byte_to_ascii_hex(c, &dst_ascii[pos]); pos += 2;
    }
    uint16_to_ascii_hex(src->software_version, &dst_ascii[pos]); pos += 4;
    byte_to_ascii_hex(src->battery_number, &dst_ascii[pos]); pos += 2;

    for (int i = 0; i < src->battery_number && i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            uint8_t c = (src->barcodes[i][j] != '\0') ? (uint8_t)src->barcodes[i][j] : 0x20;
            byte_to_ascii_hex(c, &dst_ascii[pos]); pos += 2;
        }
    }
    return (int)pos;
}

int pylon_rs485_sys_analog_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_analog_t *src) {
    if (max_len < 98) return -1;
    uint16_t pos = 0;
    uint16_to_ascii_hex(src->sys_total_voltage, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->sys_total_current, &dst_ascii[pos]); pos += 4;
    byte_to_ascii_hex(src->sys_soc, &dst_ascii[pos]); pos += 2;
    uint16_to_ascii_hex(src->avg_cycles, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->max_cycles, &dst_ascii[pos]); pos += 4;
    byte_to_ascii_hex(src->avg_soh, &dst_ascii[pos]); pos += 2;
    byte_to_ascii_hex(src->min_soh, &dst_ascii[pos]); pos += 2;
    uint16_to_ascii_hex(src->max_cell_v, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->max_cell_v_mod, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->min_cell_v, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->min_cell_v_mod, &dst_ascii[pos]); pos += 4;  
    uint16_to_ascii_hex((uint16_t)src->avg_cell_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->max_cell_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->max_cell_temp_mod, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->min_cell_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->min_cell_temp_mod, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->avg_mos_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->max_mos_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->max_mos_temp_mod, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->min_mos_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->min_mos_temp_mod, &dst_ascii[pos]); pos += 4; 
    uint16_to_ascii_hex((uint16_t)src->avg_bms_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->max_bms_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->max_bms_temp_mod, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->min_bms_temp, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->min_bms_temp_mod, &dst_ascii[pos]); pos += 4;
    return (int)pos;
}

int pylon_rs485_sys_alarm_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_alarm_t *src) {
    if (max_len < 8) return -1;
    uint16_t pos = 0;
    uint8_t a1 = 0;
    if(src->alarm1.cell_v_inconsistency) a1 |= 0x01;
    if(src->alarm1.mos_high_temp)        a1 |= 0x02;
    if(src->alarm1.cell_low_temp)        a1 |= 0x04;
    if(src->alarm1.cell_high_temp)       a1 |= 0x08;
    if(src->alarm1.cell_low_v)           a1 |= 0x10;
    if(src->alarm1.cell_high_v)          a1 |= 0x20;
    if(src->alarm1.module_low_v)         a1 |= 0x40;
    if(src->alarm1.module_high_v)        a1 |= 0x80;
    byte_to_ascii_hex(a1, &dst_ascii[pos]); pos += 2;
    
    uint8_t a2 = 0;
    if(src->alarm2.internal_comm_err)    a2 |= 0x10;
    if(src->alarm2.discharge_oc_alarm)   a2 |= 0x20;
    if(src->alarm2.charge_oc_alarm)      a2 |= 0x40;
    if(src->alarm2.cell_t_inconsistency) a2 |= 0x80;
    byte_to_ascii_hex(a2, &dst_ascii[pos]); pos += 2;
    
    uint8_t p1 = 0;
    if(src->protect1.mos_over_temp)      p1 |= 0x02;
    if(src->protect1.cell_under_temp)    p1 |= 0x04;
    if(src->protect1.cell_over_temp)     p1 |= 0x08;
    if(src->protect1.cell_under_v)       p1 |= 0x10;
    if(src->protect1.cell_over_v)        p1 |= 0x20;
    if(src->protect1.module_under_v)     p1 |= 0x40;
    if(src->protect1.module_over_v)      p1 |= 0x80;
    byte_to_ascii_hex(p1, &dst_ascii[pos]); pos += 2;
    
    uint8_t p2 = 0;
    if(src->protect2.system_fault)       p2 |= 0x08;
    if(src->protect2.discharge_oc_prot)  p2 |= 0x20;
    if(src->protect2.charge_oc_prot)     p2 |= 0x40;
    byte_to_ascii_hex(p2, &dst_ascii[pos]); pos += 2; 

    return (int)pos; 
}

int pylon_rs485_sys_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_chg_dis_mgmt_t *src) {
    if (max_len < 18) return -1;
    uint16_t pos = 0;
    uint16_to_ascii_hex(src->sys_charge_v_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex(src->sys_discharge_v_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->sys_charge_i_limit, &dst_ascii[pos]); pos += 4;
    uint16_to_ascii_hex((uint16_t)src->sys_discharge_i_limit, &dst_ascii[pos]); pos += 4;
	
    uint8_t st = 0;
    if(src->sys_status.full_chg_req) st |= 0x10;
    if(src->sys_status.force_chg)    st |= 0x20;
    if(src->sys_status.discharge_en) st |= 0x40;
    if(src->sys_status.charge_en)    st |= 0x80;
    byte_to_ascii_hex(st, &dst_ascii[pos]); pos += 2;
    return (int)pos;
}

int pylon_rs485_sys_shutdown_pack(uint8_t *dst_ascii, uint16_t max_len, uint8_t turn_off_flag) {
    return 0; 
}

/* --- UNPACKING FUNCTIONS --- */
void pylon_rs485_analog_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_analog_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 138) return;
    pylon_rs485_analog_t temp;    
    memset(&temp, 0, sizeof(pylon_rs485_analog_t));    
    uint16_t pos = 2;     
    temp.command_value = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.cell_count 	 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    for (int i = 0; i < temp.cell_count && i < 16; i++) {
        temp.cell_voltages[i] = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    }   
    if (pos + 2 <= info_len) {
        temp.temp_count = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        for (int i = 0; i < temp.temp_count && i < 8; i++) {
            temp.temperatures[i] = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
        }
    }    
    if (pos + 12 <= info_len) {
        temp.current = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;      
        temp.total_voltage = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;       
        temp.remain_cap_1 = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    }   
    if (pos + 10 <= info_len) {
        temp.user_def_count = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        temp.total_cap_1 = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
        temp.cycle_count = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;      
        if (temp.user_def_count >= 4 && (pos + 12 <= info_len)) {
            temp.remain_cap_2[0] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
            temp.remain_cap_2[1] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
            temp.remain_cap_2[2] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
            temp.total_cap_2[0]  = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
            temp.total_cap_2[1]  = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
            temp.total_cap_2[2]  = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        }
    }
    memcpy(dst, &temp, sizeof(pylon_rs485_analog_t));
}

void pylon_rs485_alarm_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_alarm_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 74) return;
    pylon_rs485_alarm_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_alarm_t));
    uint16_t pos = 2;
    temp.command_value = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.cell_count    = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    for (int i = 0; i < temp.cell_count && i < 16; i++) {
        if (pos + 2 > info_len) break;
        temp.cell_status[i] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    }
    if (pos + 2 <= info_len) {
        temp.temp_count = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        if (pos + 2 <= info_len) pos += 2;      
        for (int i = 0; i < temp.temp_count && i < 8; i++) {
            if (pos + 2 > info_len) break;
            temp.temp_status[i] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        }
    }
    
    if (pos + 2 <= info_len) { temp.charge_curr_status    = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2; }
    if (pos + 2 <= info_len) { temp.module_volt_status    = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2; }
    if (pos + 2 <= info_len) { temp.discharge_curr_status = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2; }

    uint8_t s1 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.s1.module_ov    = (s1 & 0x01) ? 1 : 0;
    temp.s1.cell_uv      = (s1 & 0x02) ? 1 : 0;
    temp.s1.charge_oc    = (s1 & 0x04) ? 1 : 0;
    temp.s1.discharge_oc = (s1 & 0x10) ? 1 : 0;
    temp.s1.discharge_ot = (s1 & 0x20) ? 1 : 0;
    temp.s1.charge_ot    = (s1 & 0x40) ? 1 : 0;
    temp.s1.module_uv    = (s1 & 0x80) ? 1 : 0;

    uint8_t s2 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.s2.pre_mosfet    = (s2 & 0x01) ? 1 : 0;
    temp.s2.charge_mos    = (s2 & 0x02) ? 1 : 0;
    temp.s2.discharge_mos = (s2 & 0x04) ? 1 : 0;
    temp.s2.module_power  = (s2 & 0x08) ? 1 : 0;

    uint8_t s3 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.s3.buzzer          = (s3 & 0x01) ? 1 : 0;
    temp.s3.fully_charged   = (s3 & 0x08) ? 1 : 0;
		temp.s3.system_error    = (s3 & 0x10) ? 1 : 0;
    temp.s3.heater          = (s3 & 0x20) ? 1 : 0;
    temp.s3.eff_dischg_curr = (s3 & 0x40) ? 1 : 0;
    temp.s3.eff_charge_curr = (s3 & 0x80) ? 1 : 0;

    if (pos + 4 <= info_len) {
        uint8_t s4 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        temp.s4.cell1 = (s4 & 0x01) ? 1 : 0; temp.s4.cell2 = (s4 & 0x02) ? 1 : 0;
        temp.s4.cell3 = (s4 & 0x04) ? 1 : 0; temp.s4.cell4 = (s4 & 0x08) ? 1 : 0;
        temp.s4.cell5 = (s4 & 0x10) ? 1 : 0; temp.s4.cell6 = (s4 & 0x20) ? 1 : 0;
        temp.s4.cell7 = (s4 & 0x40) ? 1 : 0; temp.s4.cell8 = (s4 & 0x80) ? 1 : 0;

        uint8_t s5 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        temp.s5.cell9  = (s5 & 0x01) ? 1 : 0; temp.s5.cell10 = (s5 & 0x02) ? 1 : 0;
        temp.s5.cell11 = (s5 & 0x04) ? 1 : 0; temp.s5.cell12 = (s5 & 0x08) ? 1 : 0;
        temp.s5.cell13 = (s5 & 0x10) ? 1 : 0; temp.s5.cell14 = (s5 & 0x20) ? 1 : 0;
        temp.s5.cell15 = (s5 & 0x40) ? 1 : 0; temp.s5.cell16 = (s5 & 0x80) ? 1 : 0;
    }
    memcpy(dst, &temp, sizeof(pylon_rs485_alarm_t));
}

void pylon_rs485_system_param_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_system_param_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 50) return;
    pylon_rs485_system_param_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_system_param_t));
    uint16_t pos = 2;
    temp.cell_high_v_limit   = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.cell_low_v_limit    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.cell_under_v_limit  = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.charge_high_temp    = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.charge_low_temp     = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.charge_current_lim  = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.module_high_v_lim   = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.module_low_v_lim    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.module_under_v_lim  = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.dischg_high_temp    = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.dischg_low_temp     = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.dischg_current_lim  = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    memcpy(dst, &temp, sizeof(pylon_rs485_system_param_t));
}

void pylon_rs485_manufacturer_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_manufacturer_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 64) return;
    pylon_rs485_manufacturer_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_manufacturer_t));
    uint16_t pos = 0; 
    for (int i = 0; i < 10; i++) {
        temp.battery_name[i] = (char)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    }
    temp.soft_version = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    for (int i = 0; i < 20; i++) {
        temp.manufacturer_name[i] = (char)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    }
    memcpy(dst, &temp, sizeof(pylon_rs485_manufacturer_t));
}

void pylon_rs485_mgmt_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_chg_dis_mgmt_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 20) return;
    pylon_rs485_chg_dis_mgmt_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_chg_dis_mgmt_t));
    uint16_t pos = 0;
	
    temp.command_value           = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.charge_voltage_limit    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.discharge_voltage_limit = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.charge_current_limit    = (int16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.discharge_current_limit = (int16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    
    if (pos + 2 <= info_len) {
        uint8_t stt_val = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2);
				temp.stt.precharge_en = (stt_val & 0x04) ? 1 : 0;
				temp.stt.full_chg_req = (stt_val & 0x08) ? 1 : 0;
				temp.stt.force_chg_ii = (stt_val & 0x10) ? 1 : 0;
				temp.stt.force_chg_i  = (stt_val & 0x20) ? 1 : 0;
				temp.stt.discharge_en = (stt_val & 0x40) ? 1 : 0;
				temp.stt.charge_en    = (stt_val & 0x80) ? 1 : 0;
    } 
    memcpy(dst, &temp, sizeof(pylon_rs485_chg_dis_mgmt_t));
}

void pylon_rs485_sn_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sn_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 34) return;
    pylon_rs485_sn_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_sn_t));
    uint16_t pos = 0;
    temp.command_value = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    for(int i = 0; i < 16; i++) {
        temp.sn_number[i] = (char)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    }
    memcpy(dst, &temp, sizeof(pylon_rs485_sn_t));
}

void pylon_rs485_set_mgmt_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_set_mgmt_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 20) return;
    pylon_rs485_set_mgmt_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_set_mgmt_t));
    uint16_t pos = 0;
    temp.command_value = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.set_charge_v_limit    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.set_discharge_v_limit = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.set_charge_i_limit    = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.set_discharge_i_limit = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
	
		if (pos + 2 <= info_len) {
        uint8_t st = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2);
        temp.stt.precharge_en = (st & 0x04) ? 1 : 0;
        temp.stt.full_chg_req = (st & 0x08) ? 1 : 0;
        temp.stt.force_chg_ii = (st & 0x10) ? 1 : 0;
        temp.stt.force_chg_i  = (st & 0x20) ? 1 : 0;
        temp.stt.discharge_en = (st & 0x40) ? 1 : 0;
        temp.stt.charge_en    = (st & 0x80) ? 1 : 0;
    }
    memcpy(dst, &temp, sizeof(pylon_rs485_set_mgmt_t));
}

int pylon_rs485_turn_off_unpack(const uint8_t *info_ascii, uint16_t info_len, uint8_t *dst_command_value) {
    if (info_ascii == NULL || dst_command_value == NULL || info_len < 2) {
        return -1;
    }
    *dst_command_value = (uint8_t)pylon_ascii_hex_to_uint(info_ascii, 2);
    return 0;
}

void pylon_rs485_soft_ver_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_soft_ver_full_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 12) return;
    pylon_rs485_soft_ver_full_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_soft_ver_full_t));
    uint16_t pos = 0;
    temp.command_value = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.manufacturer_version[0] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.manufacturer_version[1] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.main_line_version[0] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.main_line_version[1] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.main_line_version[2] = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    memcpy(dst, &temp, sizeof(pylon_rs485_soft_ver_full_t));
}

void pylon_rs485_sys_basic_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_basic_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 66) return;
    pylon_rs485_sys_basic_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_sys_basic_t));
    uint16_t pos = 0;

    for (int i = 0; i < 10; i++) {
        temp.battery_name[i] = (char)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    }
    for (int i = 0; i < 20; i++) {
        temp.manufacturer_name[i] = (char)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    }
    temp.software_version = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.battery_number   = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;

		uint16_t required_len = 66 + (temp.battery_number * 32);
    if (info_len < required_len) {
        return; 
    }
		
    for (int i = 0; i < temp.battery_number && i < 15; i++) {
        for (int j = 0; j < 16; j++) {
            temp.barcodes[i][j] = (char)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
        }
    }
    memcpy(dst, &temp, sizeof(pylon_rs485_sys_basic_t));
}

void pylon_rs485_sys_analog_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_analog_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 98) return;
    pylon_rs485_sys_analog_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_sys_analog_t));
    uint16_t pos = 0;
    
    temp.sys_total_voltage = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.sys_total_current = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.sys_soc           = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.avg_cycles        = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.max_cycles        = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.avg_soh           = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.min_soh           = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    
    temp.max_cell_v        = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.max_cell_v_mod    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.min_cell_v        = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.min_cell_v_mod    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;  
    
    temp.avg_cell_temp     = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.max_cell_temp     = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.max_cell_temp_mod = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.min_cell_temp     = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.min_cell_temp_mod = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    
    temp.avg_mos_temp      = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.max_mos_temp      = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.max_mos_temp_mod  = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.min_mos_temp      = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.min_mos_temp_mod  = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4; 
    
    temp.avg_bms_temp      = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.max_bms_temp      = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.max_bms_temp_mod  = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.min_bms_temp      = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.min_bms_temp_mod  = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    
    memcpy(dst, &temp, sizeof(pylon_rs485_sys_analog_t));
}

void pylon_rs485_sys_alarm_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_alarm_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 8) return;
    pylon_rs485_sys_alarm_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_sys_alarm_t));
    uint16_t pos = 0;

    uint8_t a1 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.alarm1.cell_v_inconsistency = (a1 & 0x01) ? 1 : 0;
    temp.alarm1.mos_high_temp        = (a1 & 0x02) ? 1 : 0;
    temp.alarm1.cell_low_temp        = (a1 & 0x04) ? 1 : 0;
    temp.alarm1.cell_high_temp       = (a1 & 0x08) ? 1 : 0;
    temp.alarm1.cell_low_v           = (a1 & 0x10) ? 1 : 0;
    temp.alarm1.cell_high_v          = (a1 & 0x20) ? 1 : 0;
    temp.alarm1.module_low_v         = (a1 & 0x40) ? 1 : 0;
    temp.alarm1.module_high_v        = (a1 & 0x80) ? 1 : 0;
    
    uint8_t a2 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.alarm2.internal_comm_err    = (a2 & 0x10) ? 1 : 0;
    temp.alarm2.discharge_oc_alarm   = (a2 & 0x20) ? 1 : 0;
    temp.alarm2.charge_oc_alarm      = (a2 & 0x40) ? 1 : 0;
    temp.alarm2.cell_t_inconsistency = (a2 & 0x80) ? 1 : 0;
    
    uint8_t p1 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.protect1.mos_over_temp      = (p1 & 0x02) ? 1 : 0;
    temp.protect1.cell_under_temp    = (p1 & 0x04) ? 1 : 0;
    temp.protect1.cell_over_temp     = (p1 & 0x08) ? 1 : 0;
    temp.protect1.cell_under_v       = (p1 & 0x10) ? 1 : 0;
    temp.protect1.cell_over_v        = (p1 & 0x20) ? 1 : 0;
    temp.protect1.module_under_v     = (p1 & 0x40) ? 1 : 0;
    temp.protect1.module_over_v      = (p1 & 0x80) ? 1 : 0;
    
    uint8_t p2 = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.protect2.system_fault       = (p2 & 0x08) ? 1 : 0;
    temp.protect2.discharge_oc_prot  = (p2 & 0x20) ? 1 : 0;
    temp.protect2.charge_oc_prot     = (p2 & 0x40) ? 1 : 0;

    memcpy(dst, &temp, sizeof(pylon_rs485_sys_alarm_t));
}

void pylon_rs485_sys_mgmt_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_sys_chg_dis_mgmt_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 18) return;
    pylon_rs485_sys_chg_dis_mgmt_t temp;
    memset(&temp, 0, sizeof(pylon_rs485_sys_chg_dis_mgmt_t));
    uint16_t pos = 0;
    
    temp.sys_charge_v_limit    = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.sys_discharge_v_limit = (uint16_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 4); pos += 4;
    temp.sys_charge_i_limit    = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    temp.sys_discharge_i_limit = ascii_hex_to_int16(&info_ascii[pos]); pos += 4;
    
    uint8_t st = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.sys_status.full_chg_req = (st & 0x10) ? 1 : 0;
    temp.sys_status.force_chg    = (st & 0x20) ? 1 : 0;
    temp.sys_status.discharge_en = (st & 0x40) ? 1 : 0;
    temp.sys_status.charge_en    = (st & 0x80) ? 1 : 0;
    
    memcpy(dst, &temp, sizeof(pylon_rs485_sys_chg_dis_mgmt_t));
}

int pylon_rs485_sys_shutdown_unpack(const uint8_t *info_ascii, uint16_t info_len, uint8_t *dst_turn_off_flag) {
    if (info_ascii == NULL || dst_turn_off_flag == NULL || info_len < 2) {
        return -1;
    }
    *dst_turn_off_flag = (uint8_t)pylon_ascii_hex_to_uint(info_ascii, 2);
    return 0;
}

/* --- COMMUNICATION FUNCTIONS --- */
static void pylon_send_response(UART_HandleTypeDef *huart, uint8_t ver, uint8_t adr, 
                                uint8_t cid1, uint8_t rtn, 
                                const uint8_t *info_ascii, uint16_t info_ascii_len) {
    static uint8_t tx[512]; 
    uint16_t pos = 0;
    
    tx[pos++] = PYLON_SOI;
    byte_to_ascii_hex(ver, &tx[pos]); pos += 2;
    byte_to_ascii_hex(adr, &tx[pos]); pos += 2;
    byte_to_ascii_hex(cid1, &tx[pos]); pos += 2;
    byte_to_ascii_hex(rtn, &tx[pos]); pos += 2;

    uint16_t len_field = pylon_485_calc_lchksum(info_ascii_len);
    byte_to_ascii_hex((uint8_t)(len_field >> 8), &tx[pos]); pos += 2;
    byte_to_ascii_hex((uint8_t)(len_field & 0xFF), &tx[pos]); pos += 2;
    
    if (info_ascii != NULL && info_ascii_len > 0) {
        if (pos + info_ascii_len < 500) { 
            memcpy(&tx[pos], info_ascii, info_ascii_len);
            pos += info_ascii_len;
        }
    }
    
    uint16_t chksum = pylon_checksum_internal(&tx[1], pos - 1);
    byte_to_ascii_hex((uint8_t)(chksum >> 8), &tx[pos]); pos += 2;
    byte_to_ascii_hex((uint8_t)(chksum & 0xFF), &tx[pos]); pos += 2;
    tx[pos++] = PYLON_EOI;
    
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    if (huart->Instance == UART4) RS485_UART4_TX_EN(); else RS485_UART5_TX_EN();
    __set_PRIMASK(primask_bit);
    
    HAL_UART_Transmit(huart, tx, pos, 1000);
    
    uint32_t tx_timeout = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
        if (HAL_GetTick() - tx_timeout > 50) break;
    }

    primask_bit = __get_PRIMASK();
    __disable_irq();
    if (huart->Instance == UART4) RS485_UART4_RX_EN(); else RS485_UART5_RX_EN();
    __set_PRIMASK(primask_bit);
}
                                                                
HAL_StatusTypeDef Pylon_Request_Data(UART_HandleTypeDef *huart, uint8_t target_adr, uint8_t cid2) {
    uint8_t info_ascii[2] = {0};
    uint16_t info_ascii_len = 0;
    if (cid2 == PYLON_CID2_ANALOG_DATA   || 
        cid2 == PYLON_CID2_ALARM_INFO    || 
        cid2 == PYLON_CID2_CHG_DIS_MGMT  || 
        cid2 == PYLON_CID2_GET_SN        || 
        cid2 == PYLON_CID2_TURN_OFF      ||
        cid2 == PYLON_CID2_SYSTEM_PARAM  || 
        cid2 == PYLON_CID2_MANUFACTURER  || 
        cid2 == PYLON_CID2_GET_FW        || 
        cid2 == PYLON_CID2_SET_MGMT      ||
        cid2 == PYLON_CID2_PROTOCOL_VER) 
    {
        byte_to_ascii_hex(target_adr, info_ascii);
        info_ascii_len = 2;
    }
    pylon_send_response(huart, PYLON_VER, target_adr, PYLON_CID1_BATTERY, cid2, info_ascii, info_ascii_len);    
    return HAL_OK;
}

HAL_StatusTypeDef Pylon_Request_Set_Mgmt(UART_HandleTypeDef *huart, uint8_t target_adr, pylon_rs485_set_mgmt_t *cmd) {
    uint8_t info_ascii[64] = {0};
    int out_len = pylon_rs485_set_mgmt_pack(info_ascii, 64, cmd);   
    if (out_len > 0) {
        pylon_send_response(huart, PYLON_VER, target_adr, PYLON_CID1_BATTERY, PYLON_CID2_SET_MGMT, info_ascii, out_len);
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef Pylon_Request_Soft_Reset(UART_HandleTypeDef *huart, uint8_t target_adr, uint8_t payload) {
    uint8_t info_ascii[2];
    byte_to_ascii_hex(payload, info_ascii);
    pylon_send_response(huart, PYLON_VER, target_adr, PYLON_CID1_BATTERY, PYLON_CID2_SOFT_RESET, info_ascii, 2);
    return HAL_OK;
}

void Pylon_Broadcast_Master_Analog(UART_HandleTypeDef *huart) {
    if (is_master != 1) return;
    uint8_t info_out[512];
    pylon_rs485_analog_t master_ana = {0};    
    master_ana.command_value = 0xFF;
    master_ana.cell_count = 16;
    for (int i = 0; i < 16; i++) {
        master_ana.cell_voltages[i] = CellVoltage[i];
    }
    master_ana.temp_count = 8;
    master_ana.temperatures[0] = (int16_t)(Temperature[0] * 10 + 2731);
    master_ana.temperatures[1] = (int16_t)(Temperature[1] * 10 + 2731);
    master_ana.temperatures[2] = (int16_t)(Temperature[2] * 10 + 2731);
    master_ana.temperatures[3] = (int16_t)(Temperature[3] * 10 + 2731);
    master_ana.temperatures[4] = (int16_t)(ntc_1 * 10 + 2731);
    master_ana.temperatures[5] = (int16_t)(ntc_2 * 10 + 2731);
    master_ana.temperatures[6] = (int16_t)(Temperature[6] * 10 + 2731);
    master_ana.temperatures[7] = (int16_t)(ntc_temp * 10 + 2731);
    master_ana.current 				 = (int16_t)(Pack_Current); 
    master_ana.total_voltage 	 = Stack_Voltage;   
    uint32_t rem_cap_full 		 = (uint32_t)(FullChargeCapacity_mAh * (SOC / 100.0f)); 
    uint32_t tot_cap_full 		 = (uint32_t)(FullChargeCapacity_mAh);   
    master_ana.remain_cap_1 	 = 0xFFFF;
    master_ana.user_def_count  = 4; 
    master_ana.total_cap_1  	 = 0xFFFF;
    master_ana.cycle_count 		 = (uint16_t)cycle_count;    
    master_ana.remain_cap_2[0] = (uint8_t)((rem_cap_full >> 16) & 0xFF);
    master_ana.remain_cap_2[1] = (uint8_t)((rem_cap_full >> 8) & 0xFF);
    master_ana.remain_cap_2[2] = (uint8_t)(rem_cap_full & 0xFF);
    master_ana.total_cap_2[0]  = (uint8_t)((tot_cap_full >> 16) & 0xFF);
    master_ana.total_cap_2[1]  = (uint8_t)((tot_cap_full >> 8) & 0xFF);
    master_ana.total_cap_2[2]  = (uint8_t)(tot_cap_full & 0xFF);
    int out_len = pylon_rs485_analog_pack(info_out, 512, &master_ana);   
    if (out_len > 0) {
        pylon_send_response(huart, PYLON_VER, 0xFF, PYLON_CID1_BATTERY, PYLON_CID2_MASTER_ANALOG, info_out, out_len);
    }
}

void Pylon_Send_Assign_ID(UART_HandleTypeDef *huart, uint8_t id_to_assign) {
    uint8_t assign_info[2];
    byte_to_ascii_hex(id_to_assign, assign_info);
    pylon_send_response(huart, PYLON_VER, 0xFF, PYLON_CID1_BATTERY, PYLON_CID2_ASSIGN_ID, assign_info, 2);
}

void BQ_Process_UART_Byte(UART_HandleTypeDef *huart, uint8_t byte) {
    uint8_t *buff = (huart->Instance == UART4) ? rx_buff_uart4 : ((huart->Instance == UART5) ? rx_buff_uart5 : NULL);
    volatile uint16_t *idx = (huart->Instance == UART4) ? &rx_idx_uart4 : ((huart->Instance == UART5) ? &rx_idx_uart5 : NULL);

    if (buff == NULL || idx == NULL) return;
    
    if (*idx == 0) {
        if (byte != PYLON_SOI) return;
    }
    buff[*idx] = byte;
    (*idx)++;
        
    if (*idx >= 512) { 
        *idx = 0;
        return;
    }

		if (byte == PYLON_EOI && *idx > 16) {
        if (buff[0] != PYLON_SOI) { *idx = 0; return; }
                
        uint8_t ver      = (uint8_t)pylon_ascii_hex_to_uint(&buff[1], 2);
        uint8_t adr      = (uint8_t)pylon_ascii_hex_to_uint(&buff[3], 2);
        uint8_t cid1     = (uint8_t)pylon_ascii_hex_to_uint(&buff[5], 2);
        uint8_t rtn_cid2 = (uint8_t)pylon_ascii_hex_to_uint(&buff[7], 2);
        uint16_t len_field = (uint16_t)pylon_ascii_hex_to_uint(&buff[9], 4);
        uint16_t info_len = len_field & 0x0FFF;
        uint16_t expected_total = 1 + 12 + info_len + 4 + 1;
                
        if (*idx != expected_total) { *idx = 0; return; }
                
				if (len_field != pylon_485_calc_lchksum(info_len)) { 
					err_len_field++;
					*idx = 0; return; 
				}
                
				uint16_t chk_rx = (uint16_t)pylon_ascii_hex_to_uint(&buff[13 + info_len], 4);
					if (chk_rx != pylon_checksum_internal(&buff[1], 12 + info_len)) { 
							if (rtn_cid2 == PYLON_CID2_ALARM_INFO) err_chksum_alarm++;
							if (rtn_cid2 == PYLON_CID2_CHG_DIS_MGMT) err_chksum_mgmt++;
							*idx = 0; return; 
				}
				
				if (is_auto_coding == 1 && auto_code_state == AUTO_CODE_WAIT_ADDRESS_CMD) {
						if (adr == 0xFF && cid1 == PYLON_CID1_BATTERY && rtn_cid2 == PYLON_CID2_ASSIGN_ID) {
								if (info_len >= 2) {
										assigned_n = (uint8_t)pylon_ascii_hex_to_uint(&buff[13], 2);
										pylon_protocol_addr = assigned_n;
										__HAL_RCC_PWR_CLK_ENABLE();     
										__HAL_RCC_BKP_CLK_ENABLE();   
										HAL_PWR_EnableBkUpAccess();     
										BKP->DR1 = pylon_protocol_addr;
										uint8_t ack_info[2]; 
										byte_to_ascii_hex(assigned_n, ack_info);
										pylon_send_response(huart, ver, assigned_n, cid1, PYLON_RTN_NORMAL, ack_info, 2);
										auto_code_state = AUTO_CODE_DONE;
										HAL_GPIO_WritePin(DN_OP_GPIO_Port, DN_OP_Pin, GPIO_PIN_SET);               
										*idx = 0; return;
								}
						}
				}
					
				if (is_master == 1 && huart->Instance == UART5) {
						if (adr >= 0x03 && adr <= (0x02 + MAX_SLAVES)) {
								uint8_t slave_idx = adr - 0x03; 
								if (slave_idx < MAX_SLAVES) {
										slave_online_status[slave_idx] = 10;							
										if (rtn_cid2 == PYLON_RTN_NORMAL) { 
												if (info_len >= 100) { 
														pylon_rs485_analog_unpack(&buff[13], info_len, &slave_analog_data[slave_idx]);
														rs485_rx_success_flag = 1;
												} 
												else if (info_len >= 40 && info_len < 100) {
														pylon_rs485_alarm_unpack(&buff[13], info_len, &slave_alarm_data[slave_idx]);
														rs485_rx_success_flag = 1;
												} 
												else if (info_len >= 18 && info_len < 40) { 
														pylon_rs485_mgmt_unpack(&buff[13], info_len, &slave_mgmt_data[slave_idx]);
														rs485_rx_success_flag = 1;
												}
										}
								}
								*idx = 0; return;
						}
				}
				
				if (adr != pylon_protocol_addr && adr != 0xFF && adr != 0x02 && adr != 0x12) {
            *idx = 0; return; 
        }
				
				if (adr == 0xFF) {
            if (is_master == 0 && rtn_cid2 == PYLON_CID2_MASTER_ANALOG) {
                pylon_rs485_analog_unpack(&buff[13], info_len, &master_analog_data);
            }
            *idx = 0; 
            return; 
        }
				
        uint8_t reply_addr = pylon_protocol_addr;
        uint8_t info_out[512];
        int out_len = 0;
        uint8_t rtn = PYLON_RTN_NORMAL;
		
        switch (rtn_cid2) {						
            case PYLON_CID2_ANALOG_DATA: 
                {
                    pylon_rs485_analog_t ana = {0};
                    ana.command_value = reply_addr;
                    ana.cell_count = 16;
                    for (int i = 0; i < 16; i++) {
                        ana.cell_voltages[i] = CellVoltage[i];
                    }
                    ana.temp_count = 8;
                    ana.temperatures[0] = (int16_t)(Temperature[0] * 10 + 2731);
                    ana.temperatures[1] = (int16_t)(Temperature[1] * 10 + 2731);
                    ana.temperatures[2] = (int16_t)(Temperature[2] * 10 + 2731);
                    ana.temperatures[3] = (int16_t)(Temperature[3] * 10 + 2731);
                    ana.temperatures[4] = (int16_t)(ntc_1 * 10 + 2731);
                    ana.temperatures[5] = (int16_t)(ntc_2 * 10 + 2731);
                    ana.temperatures[6] = (int16_t)(Temperature[6] * 10 + 2731);
                    ana.temperatures[7] = (int16_t)(ntc_temp * 10 + 2731);
                    ana.current = (int16_t)(Pack_Current); 
                    ana.total_voltage = Stack_Voltage;
                    
                    uint32_t rem_cap_full = (uint32_t)(FullChargeCapacity_mAh * (SOC / 100.0f)); 
                    uint32_t tot_cap_full = (uint32_t)(FullChargeCapacity_mAh);
                    
                    ana.remain_cap_1 = 0xFFFF;
										ana.user_def_count = 4; 
                    ana.total_cap_1  = 0xFFFF;
                    ana.cycle_count = (uint16_t)cycle_count;
                    
                    ana.remain_cap_2[0] = (uint8_t)((rem_cap_full >> 16) & 0xFF);
                    ana.remain_cap_2[1] = (uint8_t)((rem_cap_full >> 8) & 0xFF);
                    ana.remain_cap_2[2] = (uint8_t)(rem_cap_full & 0xFF);
                    ana.total_cap_2[0]  = (uint8_t)((tot_cap_full >> 16) & 0xFF);
                    ana.total_cap_2[1]  = (uint8_t)((tot_cap_full >> 8) & 0xFF);
                    ana.total_cap_2[2]  = (uint8_t)(tot_cap_full & 0xFF);

                    out_len = pylon_rs485_analog_pack(info_out, 512, &ana);
                    break;
                }
            
            case PYLON_CID2_ALARM_INFO: 
                {
                    pylon_rs485_alarm_t alarm = {0};
                    alarm.command_value = reply_addr;
                    alarm.cell_count = 16;
                    alarm.temp_count = 8;
                    
                    uint16_t min_cv = 0xFFFF, max_cv = 0;
                    uint32_t sum_cv = 0;
                    uint8_t cell_uv_flag = 0;
                    uint8_t s4_s5_flags[16] = {0};

                    for (int i = 0; i < 16; i++) {
                        if (CellVoltage[i] < min_cv) min_cv = CellVoltage[i];
                        if (CellVoltage[i] > max_cv) max_cv = CellVoltage[i];
                        sum_cv += CellVoltage[i];

                        if (CellVoltage[i] >= 3600) {
                            alarm.cell_status[i] = 0x02;
                        } else if (CellVoltage[i] <= 2800) {
                            alarm.cell_status[i] = 0x01;
                            cell_uv_flag = 1;
                        } else {
                            alarm.cell_status[i] = 0x00;
                        }

                        if (CellVoltage[i] < 2700 || CellVoltage[i] > 3650) {
                            s4_s5_flags[i] = 1;
                        }
                    }

                    if ((max_cv - min_cv) > 1000) {
                        uint16_t avg_cv = sum_cv / 16;
                        uint16_t diff_min = (avg_cv > min_cv) ? (avg_cv - min_cv) : 0;
                        uint16_t diff_max = (max_cv > avg_cv) ? (max_cv - avg_cv) : 0;                      
                        for (int i = 0; i < 16; i++) {
                            if (diff_min > diff_max && CellVoltage[i] == min_cv) s4_s5_flags[i] = 1;
                            else if (diff_max >= diff_min && CellVoltage[i] == max_cv) s4_s5_flags[i] = 1;
                        }
                    }

                    float t_arr[8] = {Temperature[0], Temperature[1], Temperature[2], Temperature[3], 
                                      ntc_1, ntc_2, Temperature[6], ntc_temp};

                    for (int i = 0; i < 8; i++) {
                        if (t_arr[i] > 60.0f) { 
                            alarm.temp_status[i] = 0x02;
                        } else if (t_arr[i] < 5.0f) { 
                            alarm.temp_status[i] = 0x01; 
                        } else {
                            alarm.temp_status[i] = 0x00; 
                        }
                    }
                    alarm.charge_curr_status    = bms_alarms.OCC_Alarm ? 0x02 : 0x00;
										if (Stack_Voltage >= 57600) alarm.module_volt_status = 0x02;
                    else if (Stack_Voltage <= 44800) alarm.module_volt_status = 0x01;
                    else alarm.module_volt_status = 0x00;
                    alarm.discharge_curr_status = bms_alarms.OCD1_Alarm ? 0x02 : 0x00;
                    
                    alarm.s1.module_ov    = bms_alarms.Stack_OV_Alarm;
										alarm.s1.cell_uv      = cell_uv_flag;
                    alarm.s1.charge_oc    = bms_alarms.OCC_Alarm;
										alarm.s1.discharge_oc = bms_alarms.OCD1_Alarm;
                    alarm.s1.discharge_ot = bms_alarms.OTD_Alarm;										
                    alarm.s1.charge_ot    = bms_alarms.OTC_Alarm; 
										alarm.s1.module_uv    = bms_alarms.Stack_UV_Alarm;
										
                    alarm.s2.discharge_mos = DSG; 
                    alarm.s2.charge_mos    = CHG;
                    alarm.s2.pre_mosfet    = (HAL_GPIO_ReadPin(GPIOB, EN_PRECHARGE_Pin) == GPIO_PIN_SET) ? 1 : 0; 
                    alarm.s2.module_power  = 1;

                    alarm.s3.buzzer = (current_buzzer_mode != BUZZER_MODE_OFF) ? 1 : 0;
                    alarm.s3.system_error = (PFErrorsTriggered || cell_failure_locked) ? 1 : 0;
										
                    uint32_t rem_cap_full = (uint32_t)(FullChargeCapacity_mAh * (SOC / 100.0f)); 
                    uint32_t tot_cap_full = (uint32_t)(FullChargeCapacity_mAh);
                    alarm.s3.fully_charged = (rem_cap_full >= tot_cap_full) ? 1 : 0;
                    
                    alarm.s3.eff_charge_curr = (Pack_Current > 50) ? 1 : 0;
                    alarm.s3.eff_dischg_curr = (Pack_Current < -50) ? 1 : 0;
                    alarm.s3.heater          = 0;

                    alarm.s4.cell1 = s4_s5_flags[0]; alarm.s4.cell2 = s4_s5_flags[1];
                    alarm.s4.cell3 = s4_s5_flags[2]; alarm.s4.cell4 = s4_s5_flags[3];
                    alarm.s4.cell5 = s4_s5_flags[4]; alarm.s4.cell6 = s4_s5_flags[5];
                    alarm.s4.cell7 = s4_s5_flags[6]; alarm.s4.cell8 = s4_s5_flags[7];

                    alarm.s5.cell9  = s4_s5_flags[8];  alarm.s5.cell10 = s4_s5_flags[9];
                    alarm.s5.cell11 = s4_s5_flags[10]; alarm.s5.cell12 = s4_s5_flags[11];
                    alarm.s5.cell13 = s4_s5_flags[12]; alarm.s5.cell14 = s4_s5_flags[13];
                    alarm.s5.cell15 = s4_s5_flags[14]; alarm.s5.cell16 = s4_s5_flags[15];
                    
                    out_len = pylon_rs485_alarm_pack(info_out, 512, &alarm);
                    break;
                }

						case PYLON_CID2_CHG_DIS_MGMT: 
                {	
                    pylon_rs485_chg_dis_mgmt_t mgmt_tmp = {0};
                    mgmt_tmp.command_value = reply_addr;
                    mgmt_tmp.charge_voltage_limit    = 57600; 
                    mgmt_tmp.discharge_voltage_limit = 44800;
                    mgmt_tmp.charge_current_limit    = (int16_t)(system_charge_limit_A);
                    mgmt_tmp.discharge_current_limit = (int16_t)(system_discharge_limit_A);
										
										mgmt_tmp.stt.precharge_en = (HAL_GPIO_ReadPin(GPIOB, EN_PRECHARGE_Pin) == GPIO_PIN_SET) ? 1 : 0;
                    mgmt_tmp.stt.full_chg_req = full_charge_request;
										mgmt_tmp.stt.force_chg_ii = force_charge;
										mgmt_tmp.stt.force_chg_i  = 0;
										mgmt_tmp.stt.discharge_en = !uv_recovery_locked;
                    mgmt_tmp.stt.charge_en    = !ov_recovery_locked;
                    
										for(volatile int wait_bus = 0; wait_bus < 10000; wait_bus++) 
											{
												__NOP();
											}

                    out_len = pylon_rs485_mgmt_pack(info_out, 512, &mgmt_tmp);
                    break;
                }

            case PYLON_CID2_GET_SN:
                {
                    pylon_rs485_sn_t sn = {0};
                    sn.command_value = reply_addr;
                    memcpy(sn.sn_number, BMS_SERIAL_NUMBER, 16);
                    out_len = pylon_rs485_sn_pack(info_out, 512, &sn);
                    break;
                }
            
						case PYLON_CID2_SET_MGMT: 
								{
										pylon_rs485_set_mgmt_unpack(&buff[13], info_len, &last_master_cmd);
										last_master_cmd_tick = HAL_GetTick();																	
										has_received_master_cmd = 1; 
										pylon_send_response(huart, ver, reply_addr, cid1, PYLON_RTN_NORMAL, NULL, 0);
										break;
								}

            case PYLON_CID2_SYSTEM_PARAM:
                {   
                    pylon_rs485_system_param_t sp = {0};
                    sp.cell_high_v_limit   = 3650; sp.cell_low_v_limit  = 2800; sp.cell_under_v_limit = 2700;
                    sp.charge_high_temp    = 600;  sp.charge_low_temp   = 0;    sp.charge_current_lim = 12000; 
                    sp.module_high_v_lim   = 57600; sp.module_low_v_lim = 44800; sp.module_under_v_lim = 44800;
                    sp.dischg_high_temp    = 650;  sp.dischg_low_temp   = -200; sp.dischg_current_lim = 12000;
                    out_len = pylon_rs485_system_param_pack(info_out, 512, &sp);
                    break;
                }    

            case PYLON_CID2_MANUFACTURER: 
								{
										pylon_rs485_manufacturer_t mfg = {0};
										memset(mfg.battery_name, 0x20, 10);
										memcpy(mfg.battery_name, "PYLON", 5);
										mfg.soft_version = 0x2000;
										memset(mfg.manufacturer_name, 0x20, 20);
										memcpy(mfg.manufacturer_name, "PYLONTECH", 9);
										out_len = pylon_rs485_manufacturer_pack(info_out, 512, &mfg);
										break;
								}
						
						case PYLON_CID2_GET_FW:
                {
                    pylon_rs485_soft_ver_full_t fw = {0};
                    fw.command_value = reply_addr;
                    fw.manufacturer_version[0] = 0x02; 
                    fw.manufacturer_version[1] = 0x00;
                    fw.main_line_version[0] = 0x03;    
                    fw.main_line_version[1] = 0x03;
                    fw.main_line_version[2] = 0x00;
                    out_len = pylon_rs485_soft_ver_pack(info_out, 512, &fw);
                    break;
                }
								
						case PYLON_CID2_TURN_OFF:
                {
                    out_len = 0;
                    system_is_shutting_down = 1;
										CommandSubcommands(ALL_FETS_OFF); 
                    HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
                    break;
                }
								
            case PYLON_CID2_SYS_BASIC:
                {
                    pylon_rs485_sys_basic_t basic = {0};
                    memset(basic.battery_name, 0x20, 10);            
                    memcpy(basic.battery_name, "PYLON", 5);          
                    memset(basic.manufacturer_name, 0x20, 20);      
                    memcpy(basic.manufacturer_name, "PYLONTECH", 9); 
                    basic.software_version = 0x2000;                  
                    basic.battery_number = (active_packs_count > 0) ? active_packs_count : 1;                                     
                    for(int i = 0; i < basic.battery_number; i++) {
                        memcpy(basic.barcodes[i], BMS_SERIAL_NUMBER, 16); 
                    }                               
                    out_len = pylon_rs485_sys_basic_pack(info_out, 512, &basic); 
                    break;                        
                }

            case PYLON_CID2_SYS_ANALOG:
                {
                    pylon_rs485_sys_analog_t sys = {0};
                    sys.sys_total_voltage = Stack_Voltage;
                    sys.sys_total_current = (int16_t)system_total_current_01A;
                    sys.sys_soc           = system_avg_soc;
                    sys.avg_cycles        = system_avg_cycles;
                    sys.max_cycles        = system_avg_cycles;
                    sys.avg_soh           = system_avg_soh;
                    sys.min_soh           = system_avg_soh;
										uint8_t group_id 		  = 0x01;
                    sys.max_cell_v        = sys_max_cell_v;
                    sys.max_cell_v_mod    = (group_id << 8) | sys_max_v_pack_id;
                    sys.min_cell_v        = sys_min_cell_v;
                    sys.min_cell_v_mod    = (group_id << 8) | sys_min_v_pack_id;
                    int16_t master_temp_K = (int16_t)(system_max_temp_C * 10 + 2731);
                    int16_t master_min_temp_K = (int16_t)(system_min_temp_C * 10 + 2731);
                    sys.avg_cell_temp     = master_temp_K; 
                    sys.max_cell_temp     = master_temp_K;
                    sys.min_cell_temp     = master_min_temp_K;
                    sys.max_cell_temp_mod = (group_id << 8) | sys_max_t_pack_id;
                    sys.min_cell_temp_mod = (group_id << 8) | sys_min_t_pack_id;               
                    sys.avg_mos_temp      = master_temp_K; 
                    sys.max_mos_temp      = master_temp_K;
                    sys.min_mos_temp      = master_min_temp_K;
                    sys.max_mos_temp_mod  = (group_id << 8) | sys_max_t_pack_id;
                    sys.min_mos_temp_mod  = (group_id << 8) | sys_min_t_pack_id;                
                    sys.avg_bms_temp      = master_temp_K; 
                    sys.max_bms_temp      = master_temp_K;
                    sys.min_bms_temp      = master_min_temp_K;
                    sys.max_bms_temp_mod  = (group_id << 8) | sys_max_t_pack_id;
                    sys.min_bms_temp_mod  = (group_id << 8) | sys_min_t_pack_id;

                    out_len = pylon_rs485_sys_analog_pack(info_out, 512, &sys);
                    break;
                }

            case PYLON_CID2_SYS_ALARM:
                {
                    pylon_rs485_sys_alarm_t s_alarm = {0};
                    if (sys_protect_ov) s_alarm.protect1.cell_over_v = 1;
                    if (sys_protect_uv) s_alarm.protect1.cell_under_v = 1;
                    if (sys_protect_occ) s_alarm.protect2.charge_oc_prot = 1;
                    if (sys_protect_ocd) s_alarm.protect2.discharge_oc_prot = 1;
                    if (sys_protect_ot)  s_alarm.protect1.cell_over_temp = 1;
                    if (sys_protect_ut)  s_alarm.protect1.cell_under_temp = 1;
                    if (OTF_Fault)       s_alarm.protect1.mos_over_temp = 1;

                    if (sys_alarm_ov) s_alarm.alarm1.cell_high_v = 1;
                    if (sys_alarm_uv) s_alarm.alarm1.cell_low_v = 1;
                    if (sys_alarm_ot) s_alarm.alarm1.cell_high_temp = 1;
                    if (sys_alarm_ut) s_alarm.alarm1.cell_low_temp = 1;
                    if (sys_alarm_occ) s_alarm.alarm2.charge_oc_alarm = 1;
                    if (sys_alarm_ocd) s_alarm.alarm2.discharge_oc_alarm = 1;
                    if (PFErrorsTriggered) s_alarm.protect2.system_fault = 1;
                    if (active_packs_count < 2 && is_master) s_alarm.alarm2.internal_comm_err = 1;
                    
                    out_len = pylon_rs485_sys_alarm_pack(info_out, 512, &s_alarm);
                    break;
                }
								
            case PYLON_CID2_SYS_MGMT:
                {
                    pylon_rs485_sys_chg_dis_mgmt_t s_mgmt = {0};
                    s_mgmt.sys_charge_v_limit    = 57600;
                    s_mgmt.sys_discharge_v_limit = 44800; 
                    s_mgmt.sys_charge_i_limit    = (int16_t)(system_charge_limit_A); 
                    s_mgmt.sys_discharge_i_limit = (int16_t)(system_discharge_limit_A); 
                    s_mgmt.sys_status.charge_en    = CHG;
                    s_mgmt.sys_status.discharge_en = DSG;
                    s_mgmt.sys_status.full_chg_req = full_charge_request;
                    
                    out_len = pylon_rs485_sys_mgmt_pack(info_out, 512, &s_mgmt);
                    break;
                }
								
            case PYLON_CID2_SYS_SHUTDOWN: 
                {
                    out_len = pylon_rs485_sys_shutdown_pack(info_out, 512, 1);
                    system_is_shutting_down = 1;
										CommandSubcommands(ALL_FETS_OFF); 
                    HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
                    break;
                }
								
            case PYLON_CID2_SOFT_RESET:
								{
										pylon_send_response(huart, ver, reply_addr, cid1, PYLON_RTN_NORMAL, NULL, 0);									
										if (info_len >= 2) {
												uint8_t reset_type = (uint8_t)pylon_ascii_hex_to_uint(&buff[13], 2);
												Trigger_Remote_Reset_Task(reset_type);
										}							
										out_len = -1;
										break;
								}						
            default:
                rtn = PYLON_RTN_CID2_ERR;
                out_len = 0;
                break;
        }
                
        pylon_send_response(huart, ver, reply_addr, cid1, rtn, info_out, out_len);
        *idx = 0;
    }
}

#endif
