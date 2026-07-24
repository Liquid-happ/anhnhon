#include "Proto/pylon_485.h"

#if IS_BOOTLOADER == 0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- PROTOCOL SERIALIZATION UTILITIES --- */
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

void pylon_rs485_byte_to_hex(uint8_t val, uint8_t *buf) {
    static const char hex_chars[] = "0123456789ABCDEF";
    buf[0] = (uint8_t)hex_chars[(val >> 4) & 0x0F];
    buf[1] = (uint8_t)hex_chars[val & 0x0F];
}

void pylon_rs485_uint16_to_hex(uint16_t val, uint8_t *buf) {
    pylon_rs485_byte_to_hex((uint8_t)(val >> 8), buf);      
    pylon_rs485_byte_to_hex((uint8_t)(val & 0xFF), buf + 2); 
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

/* --- FRAME BUILDER --- */
int pylon_rs485_build_frame(uint8_t *dst_frame, uint16_t max_len, uint8_t ver, uint8_t adr, uint8_t cid1, uint8_t rtn_cid2, const uint8_t *info_ascii, uint16_t info_len) {
    if (max_len < 1 + 12 + info_len + 4 + 1) return -1;
    uint16_t pos = 0;
    
    dst_frame[pos++] = PYLON_SOI;
    pylon_rs485_byte_to_hex(ver, &dst_frame[pos]); pos += 2;
    pylon_rs485_byte_to_hex(adr, &dst_frame[pos]); pos += 2;
    pylon_rs485_byte_to_hex(cid1, &dst_frame[pos]); pos += 2;
    pylon_rs485_byte_to_hex(rtn_cid2, &dst_frame[pos]); pos += 2;

    uint16_t len_field = pylon_485_calc_lchksum(info_len);
    pylon_rs485_byte_to_hex((uint8_t)(len_field >> 8), &dst_frame[pos]); pos += 2;
    pylon_rs485_byte_to_hex((uint8_t)(len_field & 0xFF), &dst_frame[pos]); pos += 2;
    
    if (info_ascii != NULL && info_len > 0) {
        memcpy(&dst_frame[pos], info_ascii, info_len);
        pos += info_len;
    }
    
    uint16_t chksum = pylon_checksum_internal(&dst_frame[1], pos - 1);
    pylon_rs485_byte_to_hex((uint8_t)(chksum >> 8), &dst_frame[pos]); pos += 2;
    pylon_rs485_byte_to_hex((uint8_t)(chksum & 0xFF), &dst_frame[pos]); pos += 2;
    dst_frame[pos++] = PYLON_EOI;
    
    return (int)pos;
}

/* --- PACKING FUNCTIONS --- */
int pylon_rs485_analog_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_analog_t *src) {
    if (max_len < 138) return -1;
    uint16_t pos = 0;
    pylon_rs485_byte_to_hex(0x00, &dst_ascii[pos]); pos += 2; 
    pylon_rs485_byte_to_hex(src->command_value, &dst_ascii[pos]); pos += 2;    
    pylon_rs485_byte_to_hex(src->cell_count, &dst_ascii[pos]); pos += 2;
    for(int i = 0; i < src->cell_count; i++) {
        pylon_rs485_uint16_to_hex(src->cell_voltages[i], &dst_ascii[pos]); pos += 4;
    }   
    pylon_rs485_byte_to_hex(src->temp_count, &dst_ascii[pos]); pos += 2;
    for(int i = 0; i < src->temp_count; i++) {
        pylon_rs485_uint16_to_hex((uint16_t)src->temperatures[i], &dst_ascii[pos]); pos += 4;
    }
    pylon_rs485_uint16_to_hex((uint16_t)src->current, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->total_voltage, &dst_ascii[pos]); pos += 4;
    if (src->user_def_count == 4) {
        pylon_rs485_uint16_to_hex(0xFFFF, &dst_ascii[pos]); pos += 4; 
        pylon_rs485_byte_to_hex(src->user_def_count, &dst_ascii[pos]); pos += 2;
        pylon_rs485_uint16_to_hex(0xFFFF, &dst_ascii[pos]); pos += 4; 
    } else {
        pylon_rs485_uint16_to_hex(src->remain_cap_1, &dst_ascii[pos]); pos += 4;
        pylon_rs485_byte_to_hex(src->user_def_count, &dst_ascii[pos]); pos += 2;
        pylon_rs485_uint16_to_hex(src->total_cap_1, &dst_ascii[pos]); pos += 4;
    }
    pylon_rs485_uint16_to_hex(src->cycle_count, &dst_ascii[pos]); pos += 4; 
    if (src->user_def_count == 4) {
        pylon_rs485_byte_to_hex(src->remain_cap_2[0], &dst_ascii[pos]); pos += 2;
        pylon_rs485_byte_to_hex(src->remain_cap_2[1], &dst_ascii[pos]); pos += 2;
        pylon_rs485_byte_to_hex(src->remain_cap_2[2], &dst_ascii[pos]); pos += 2;
        pylon_rs485_byte_to_hex(src->total_cap_2[0], &dst_ascii[pos]); pos += 2;
        pylon_rs485_byte_to_hex(src->total_cap_2[1], &dst_ascii[pos]); pos += 2;
        pylon_rs485_byte_to_hex(src->total_cap_2[2], &dst_ascii[pos]); pos += 2;
    }
    return (int)pos;
}

int pylon_rs485_alarm_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_alarm_t *src) {
    if (max_len < 74) return -1;
    uint16_t pos = 0;
    pylon_rs485_byte_to_hex(0x00, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->cell_count, &dst_ascii[pos]); pos += 2;
    for (int i = 0; i < src->cell_count; i++) {
        pylon_rs485_byte_to_hex(src->cell_status[i], &dst_ascii[pos]); pos += 2;
    }        
    pylon_rs485_byte_to_hex(src->temp_count, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(0x00, &dst_ascii[pos]); pos += 2; 
    for (int i = 0; i < src->temp_count; i++) {
        pylon_rs485_byte_to_hex(src->temp_status[i], &dst_ascii[pos]); pos += 2;
    }       
    pylon_rs485_byte_to_hex(src->charge_curr_status, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->module_volt_status, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->discharge_curr_status, &dst_ascii[pos]); pos += 2;

    uint8_t s1 = 0;
    if(src->s1.module_ov)    s1 |= 0x01;
    if(src->s1.cell_uv)      s1 |= 0x02;
    if(src->s1.charge_oc)    s1 |= 0x04;
    if(src->s1.discharge_oc) s1 |= 0x10;
    if(src->s1.discharge_ot) s1 |= 0x20;
    if(src->s1.charge_ot)    s1 |= 0x40;
    if(src->s1.module_uv)    s1 |= 0x80;
    pylon_rs485_byte_to_hex(s1, &dst_ascii[pos]); pos += 2;

    uint8_t s2 = 0;
    if(src->s2.pre_mosfet)    s2 |= 0x01;
    if(src->s2.charge_mos)    s2 |= 0x02;
    if(src->s2.discharge_mos) s2 |= 0x04;
    if(src->s2.module_power)  s2 |= 0x08;
    pylon_rs485_byte_to_hex(s2, &dst_ascii[pos]); pos += 2;

    uint8_t s3 = 0;
    if(src->s3.buzzer)          s3 |= 0x01;
    if(src->s3.fully_charged)   s3 |= 0x08;
    if(src->s3.system_error)    s3 |= 0x10;
    if(src->s3.heater)          s3 |= 0x20;
    if(src->s3.eff_dischg_curr) s3 |= 0x40;
    if(src->s3.eff_charge_curr) s3 |= 0x80;
    pylon_rs485_byte_to_hex(s3, &dst_ascii[pos]); pos += 2;

    uint8_t s4 = 0;
    if(src->s4.cell1) s4 |= 0x01; if(src->s4.cell2) s4 |= 0x02;
    if(src->s4.cell3) s4 |= 0x04; if(src->s4.cell4) s4 |= 0x08;
    if(src->s4.cell5) s4 |= 0x10; if(src->s4.cell6) s4 |= 0x20;
    if(src->s4.cell7) s4 |= 0x40; if(src->s4.cell8) s4 |= 0x80;
    pylon_rs485_byte_to_hex(s4, &dst_ascii[pos]); pos += 2;

    uint8_t s5 = 0;
    if(src->s5.cell9)  s5 |= 0x01; if(src->s5.cell10) s5 |= 0x02;
    if(src->s5.cell11) s5 |= 0x04; if(src->s5.cell12) s5 |= 0x08;
    if(src->s5.cell13) s5 |= 0x10; if(src->s5.cell14) s5 |= 0x20;
    if(src->s5.cell15) s5 |= 0x40; if(src->s5.cell16) s5 |= 0x80;
    pylon_rs485_byte_to_hex(s5, &dst_ascii[pos]); pos += 2;

    return (int)pos;
}

int pylon_rs485_system_param_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_system_param_t *src) {
    if (max_len < 50) return -1;
    uint16_t pos = 0;
    pylon_rs485_byte_to_hex(0x00, &dst_ascii[pos]); pos += 2;
    pylon_rs485_uint16_to_hex(src->cell_high_v_limit,  &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->cell_low_v_limit,   &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->cell_under_v_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->charge_high_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->charge_low_temp,  &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->charge_current_lim, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->module_high_v_lim,  &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->module_low_v_lim,   &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->module_under_v_lim, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->dischg_high_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->dischg_low_temp,  &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->dischg_current_lim, &dst_ascii[pos]); pos += 4;
    return (int)pos;
}

int pylon_rs485_manufacturer_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_manufacturer_t *src) {
    if (max_len < 64) return -1;
    uint16_t pos = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t c = (src->battery_name[i] != '\0') ? (uint8_t)src->battery_name[i] : 0x20;
        pylon_rs485_byte_to_hex(c, &dst_ascii[pos]); pos += 2;
    }
    pylon_rs485_uint16_to_hex(src->soft_version, &dst_ascii[pos]); pos += 4;
    for (int i = 0; i < 20; i++) {
        uint8_t c = (src->manufacturer_name[i] != '\0') ? (uint8_t)src->manufacturer_name[i] : 0x20;
        pylon_rs485_byte_to_hex(c, &dst_ascii[pos]); pos += 2;
    }
    return (int)pos; 
}

int pylon_rs485_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_chg_dis_mgmt_t *src) {
    if (max_len < 20) return -1;
    uint16_t pos = 0;

    pylon_rs485_byte_to_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    pylon_rs485_uint16_to_hex(src->charge_voltage_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->discharge_voltage_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->charge_current_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->discharge_current_limit, &dst_ascii[pos]); pos += 4;

    uint8_t st = 0;
    if (src->stt.precharge_en) st |= 0x04; 
    if (src->stt.full_chg_req) st |= 0x08; 
    if (src->stt.force_chg_ii) st |= 0x10;
    if (src->stt.force_chg_i)  st |= 0x20; 
    if (src->stt.discharge_en) st |= 0x40; 
    if (src->stt.charge_en)    st |= 0x80; 
    pylon_rs485_byte_to_hex(st, &dst_ascii[pos]); pos += 2;
    return (int)pos;
}

int pylon_rs485_sn_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sn_t *src) {
    if (max_len < 34) return -1;
    uint16_t pos = 0;
    pylon_rs485_byte_to_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    for(int i = 0; i < 16; i++) {
        uint8_t c = (src->sn_number[i] != '\0') ? (uint8_t)src->sn_number[i] : 0x20;
        pylon_rs485_byte_to_hex(c, &dst_ascii[pos]); pos += 2;
    }
    return (int)pos;
}

int pylon_rs485_set_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_set_mgmt_t *src) {
    if (max_len < 20) return -1;
    uint16_t pos = 0;
    pylon_rs485_byte_to_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    pylon_rs485_uint16_to_hex(src->set_charge_v_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->set_discharge_v_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->set_charge_i_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->set_discharge_i_limit, &dst_ascii[pos]); pos += 4;
        
    uint8_t st = 0;
    if (src->stt.precharge_en) st |= 0x04; 
    if (src->stt.full_chg_req) st |= 0x08; 
    if (src->stt.force_chg_ii) st |= 0x10;
    if (src->stt.force_chg_i)  st |= 0x20; 
    if (src->stt.discharge_en) st |= 0x40; 
    if (src->stt.charge_en)    st |= 0x80; 
    pylon_rs485_byte_to_hex(st, &dst_ascii[pos]); pos += 2;
    
    return (int)pos;
}

int pylon_rs485_turn_off_pack(uint8_t *dst_ascii, uint16_t max_len, uint8_t command_value) {
    if (max_len < 2) return -1;
    pylon_rs485_byte_to_hex(command_value, dst_ascii);
    return 2;
}

int pylon_rs485_soft_ver_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_soft_ver_full_t *src) {
    if (max_len < 12) return -1;
    uint16_t pos = 0;
    pylon_rs485_byte_to_hex(src->command_value, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->manufacturer_version[0], &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->manufacturer_version[1], &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->main_line_version[0], &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->main_line_version[1], &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->main_line_version[2], &dst_ascii[pos]); pos += 2;
    return (int)pos;
}

int pylon_rs485_sys_basic_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_basic_t *src) {
    uint16_t required_len = 66 + (src->battery_number * 32);
    if (max_len < required_len) return -1; 
    uint16_t pos = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t c = (src->battery_name[i] != '\0') ? (uint8_t)src->battery_name[i] : 0x20;
        pylon_rs485_byte_to_hex(c, &dst_ascii[pos]); pos += 2;
    }
    for (int i = 0; i < 20; i++) {
        uint8_t c = (src->manufacturer_name[i] != '\0') ? (uint8_t)src->manufacturer_name[i] : 0x20;
        pylon_rs485_byte_to_hex(c, &dst_ascii[pos]); pos += 2;
    }
    pylon_rs485_uint16_to_hex(src->software_version, &dst_ascii[pos]); pos += 4;
    pylon_rs485_byte_to_hex(src->battery_number, &dst_ascii[pos]); pos += 2;

    for (int i = 0; i < src->battery_number && i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            uint8_t c = (src->barcodes[i][j] != '\0') ? (uint8_t)src->barcodes[i][j] : 0x20;
            pylon_rs485_byte_to_hex(c, &dst_ascii[pos]); pos += 2;
        }
    }
    return (int)pos;
}

int pylon_rs485_sys_analog_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_analog_t *src) {
    if (max_len < 98) return -1;
    uint16_t pos = 0;
    pylon_rs485_uint16_to_hex(src->sys_total_voltage, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->sys_total_current, &dst_ascii[pos]); pos += 4;
    pylon_rs485_byte_to_hex(src->sys_soc, &dst_ascii[pos]); pos += 2;
    pylon_rs485_uint16_to_hex(src->avg_cycles, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->max_cycles, &dst_ascii[pos]); pos += 4;
    pylon_rs485_byte_to_hex(src->avg_soh, &dst_ascii[pos]); pos += 2;
    pylon_rs485_byte_to_hex(src->min_soh, &dst_ascii[pos]); pos += 2;
    pylon_rs485_uint16_to_hex(src->max_cell_v, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->max_cell_v_mod, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->min_cell_v, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->min_cell_v_mod, &dst_ascii[pos]); pos += 4;  
    pylon_rs485_uint16_to_hex((uint16_t)src->avg_cell_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->max_cell_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->max_cell_temp_mod, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->min_cell_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->min_cell_temp_mod, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->avg_mos_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->max_mos_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->max_mos_temp_mod, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->min_mos_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->min_mos_temp_mod, &dst_ascii[pos]); pos += 4; 
    pylon_rs485_uint16_to_hex((uint16_t)src->avg_bms_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->max_bms_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->max_bms_temp_mod, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->min_bms_temp, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->min_bms_temp_mod, &dst_ascii[pos]); pos += 4;
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
    pylon_rs485_byte_to_hex(a1, &dst_ascii[pos]); pos += 2;
    
    uint8_t a2 = 0;
    if(src->alarm2.internal_comm_err)    a2 |= 0x10;
    if(src->alarm2.discharge_oc_alarm)   a2 |= 0x20;
    if(src->alarm2.charge_oc_alarm)      a2 |= 0x40;
    if(src->alarm2.cell_t_inconsistency) a2 |= 0x80;
    pylon_rs485_byte_to_hex(a2, &dst_ascii[pos]); pos += 2;
    
    uint8_t p1 = 0;
    if(src->protect1.mos_over_temp)      p1 |= 0x02;
    if(src->protect1.cell_under_temp)    p1 |= 0x04;
    if(src->protect1.cell_over_temp)     p1 |= 0x08;
    if(src->protect1.cell_under_v)       p1 |= 0x10;
    if(src->protect1.cell_over_v)        p1 |= 0x20;
    if(src->protect1.module_under_v)     p1 |= 0x40;
    if(src->protect1.module_over_v)      p1 |= 0x80;
    pylon_rs485_byte_to_hex(p1, &dst_ascii[pos]); pos += 2;
    
    uint8_t p2 = 0;
    if(src->protect2.system_fault)       p2 |= 0x08;
    if(src->protect2.discharge_oc_prot)  p2 |= 0x20;
    if(src->protect2.charge_oc_prot)     p2 |= 0x40;
    pylon_rs485_byte_to_hex(p2, &dst_ascii[pos]); pos += 2; 

    return (int)pos; 
}

int pylon_rs485_sys_mgmt_pack(uint8_t *dst_ascii, uint16_t max_len, const pylon_rs485_sys_chg_dis_mgmt_t *src) {
    if (max_len < 18) return -1;
    uint16_t pos = 0;
    pylon_rs485_uint16_to_hex(src->sys_charge_v_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex(src->sys_discharge_v_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->sys_charge_i_limit, &dst_ascii[pos]); pos += 4;
    pylon_rs485_uint16_to_hex((uint16_t)src->sys_discharge_i_limit, &dst_ascii[pos]); pos += 4;
        
    uint8_t st = 0;
    if(src->sys_status.full_chg_req) st |= 0x10;
    if(src->sys_status.force_chg)    st |= 0x20;
    if(src->sys_status.discharge_en) st |= 0x40;
    if(src->sys_status.charge_en)    st |= 0x80;
    pylon_rs485_byte_to_hex(st, &dst_ascii[pos]); pos += 2;
    return (int)pos;
}

int pylon_rs485_sys_shutdown_pack(uint8_t *dst_ascii, uint16_t max_len, uint8_t turn_off_flag) {
    (void)dst_ascii;
    (void)max_len;
    (void)turn_off_flag;
    return 0; 
}

/* --- UNPACKING FUNCTIONS --- */
void pylon_rs485_analog_unpack(const uint8_t *info_ascii, uint16_t info_len, pylon_rs485_analog_t *dst) {
    if (info_ascii == NULL || dst == NULL || info_len < 138) return;
    pylon_rs485_analog_t temp;    
    memset(&temp, 0, sizeof(pylon_rs485_analog_t));    
    uint16_t pos = 2;     
    temp.command_value = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
    temp.cell_count      = (uint8_t)pylon_ascii_hex_to_uint(&info_ascii[pos], 2); pos += 2;
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

#endif /* IS_BOOTLOADER == 0 */
