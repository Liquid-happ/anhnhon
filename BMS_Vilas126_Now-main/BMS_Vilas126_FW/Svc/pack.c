#include "Svc/pack.h"

#if IS_BOOTLOADER == 0

#include "Proto/pylon_485.h"
#include "Drv/rs485_drv.h"
#include "bms_state.h"
#include "main.h"
#include "BQ769x2Header.h"
#include <string.h>

// Global definitions for Svc/pack variables
Auto_Code_State_t auto_code_state = AUTO_CODE_START;
uint8_t is_auto_coding = 0;
volatile uint8_t assigned_n = 0;
uint8_t bms_device_address = 0x02;
uint8_t pylon_protocol_addr = 0x02;
uint8_t current_assigning_index = 0x03;
uint8_t is_master = 0;
uint8_t active_packs_count = 0;

uint32_t last_master_cmd_tick = 0;
uint8_t has_received_master_cmd = 0;
volatile uint8_t rs485_rx_success_flag = 0;

pylon_rs485_analog_t slave_analog_data[MAX_SLAVES] = {{0}};
pylon_rs485_chg_dis_mgmt_t slave_mgmt_data[MAX_SLAVES] = {{0}};
pylon_rs485_alarm_t slave_alarm_data[MAX_SLAVES] = {{0}};
uint8_t slave_online_status[MAX_SLAVES] = {0};

pylon_rs485_analog_t master_analog_data = {0};
pylon_rs485_set_mgmt_t last_master_cmd = {0};

uint32_t err_len_field = 0;
uint32_t err_chksum_alarm = 0;
uint32_t err_chksum_mgmt = 0;

// Extern variables representing the system's telemetry and metrics from Core/Src/main.c or elsewhere
extern uint32_t cycle_count;
extern float FullChargeCapacity_mAh;
extern int32_t system_total_current_01A;
extern uint8_t system_avg_soc;
extern uint8_t system_avg_soh;
extern uint16_t system_avg_cycles;
extern uint16_t sys_max_cell_v;
extern uint16_t sys_min_cell_v;
extern uint8_t sys_max_v_pack_id;
extern uint8_t sys_min_v_pack_id;
extern uint8_t sys_max_t_pack_id;
extern uint8_t sys_min_t_pack_id;
extern float system_max_temp_C;
extern float system_min_temp_C;

extern uint8_t sys_protect_ov;
extern uint8_t sys_protect_uv;
extern uint8_t sys_protect_occ;
extern uint8_t sys_protect_ocd;
extern uint8_t sys_protect_ot;
extern uint8_t sys_protect_ut;
extern uint8_t OTF_Fault;

extern uint8_t sys_alarm_ov;
extern uint8_t sys_alarm_uv;
extern uint8_t sys_alarm_ot;
extern uint8_t sys_alarm_ut;
extern uint8_t sys_alarm_occ;
extern uint8_t sys_alarm_ocd;

extern uint8_t PFErrorsTriggered;
extern uint8_t cell_failure_locked;
extern uint8_t full_charge_request;
extern uint8_t force_charge;
extern uint8_t system_is_shutting_down;

extern float system_charge_limit_A;
extern float system_discharge_limit_A;
extern uint8_t slave_isolated[MAX_SLAVES];

extern uint8_t CHG;
extern uint8_t DSG;

extern void CommandSubcommands(uint16_t subcommand);

#define RX_BUFFER_SIZE 512
static uint8_t rx_buff_uart4[RX_BUFFER_SIZE];
static uint16_t rx_idx_uart4 = 0;
static uint8_t rx_buff_uart5[RX_BUFFER_SIZE];
static uint16_t rx_idx_uart5 = 0;

static const char BMS_SERIAL_NUMBER[17] = "PYLNBMS314AH0001";

static void pylon_send_response_internal(uint8_t port, uint8_t ver, uint8_t adr, uint8_t cid1, uint8_t rtn_cid2_rtn, const uint8_t *info_ascii, uint16_t info_len) {
    static uint8_t tx_frame[512];
    int frame_len = pylon_rs485_build_frame(tx_frame, sizeof(tx_frame), ver, adr, cid1, rtn_cid2_rtn, info_ascii, info_len);
    if (frame_len > 0) {
        RS485_Drv_Transmit(port, tx_frame, (uint16_t)frame_len);
    }
}

void Pack_Svc_Init(void) {
    auto_code_state = AUTO_CODE_START;
    is_auto_coding = 0;
    assigned_n = 0;
    bms_device_address = 0x02;
    pylon_protocol_addr = 0x02;
    current_assigning_index = 0x03;
    is_master = 0;
    active_packs_count = 0;
    last_master_cmd_tick = 0;
    has_received_master_cmd = 0;
    rs485_rx_success_flag = 0;
    memset(slave_analog_data, 0, sizeof(slave_analog_data));
    memset(slave_mgmt_data, 0, sizeof(slave_mgmt_data));
    memset(slave_alarm_data, 0, sizeof(slave_alarm_data));
    memset(slave_online_status, 0, sizeof(slave_online_status));
    memset(&master_analog_data, 0, sizeof(master_analog_data));
    memset(&last_master_cmd, 0, sizeof(last_master_cmd));
    err_len_field = 0;
    err_chksum_alarm = 0;
    err_chksum_mgmt = 0;
}

void Pack_Svc_Process_Byte(uint8_t port, uint8_t byte) {
    uint8_t *buff = (port == RS485_PORT_UART4) ? rx_buff_uart4 : rx_buff_uart5;
    uint16_t *idx = (port == RS485_PORT_UART4) ? &rx_idx_uart4 : &rx_idx_uart5;

    if (*idx == 0) {
        if (byte != PYLON_SOI) return;
    }
    buff[*idx] = byte;
    (*idx)++;
        
    if (*idx >= RX_BUFFER_SIZE) { 
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
        uint16_t sum = 0;
        for (uint16_t i = 1; i < 13 + info_len; i++) sum += buff[i];
        uint16_t chk_calc = (uint16_t)((~sum + 1) & 0xFFFF);

        if (chk_rx != chk_calc) { 
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
                    pylon_rs485_byte_to_hex(assigned_n, ack_info);
                    pylon_send_response_internal(port, ver, assigned_n, cid1, PYLON_RTN_NORMAL, ack_info, 2);
                    auto_code_state = AUTO_CODE_DONE;
                    HAL_GPIO_WritePin(DN_OP_GPIO_Port, DN_OP_Pin, GPIO_PIN_SET);               
                    *idx = 0; return;
                }
            }
        }
            
        if (is_master == 1 && port == RS485_PORT_UART5) {
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

                    out_len = pylon_rs485_analog_pack(info_out, sizeof(info_out), &ana);
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
                    
                    out_len = pylon_rs485_alarm_pack(info_out, sizeof(info_out), &alarm);
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

                    out_len = pylon_rs485_mgmt_pack(info_out, sizeof(info_out), &mgmt_tmp);
                    break;
                }

            case PYLON_CID2_GET_SN:
                {
                    pylon_rs485_sn_t sn = {0};
                    sn.command_value = reply_addr;
                    memcpy(sn.sn_number, BMS_SERIAL_NUMBER, 16);
                    out_len = pylon_rs485_sn_pack(info_out, sizeof(info_out), &sn);
                    break;
                }
            
            case PYLON_CID2_SET_MGMT: 
                {
                    pylon_rs485_set_mgmt_unpack(&buff[13], info_len, &last_master_cmd);
                    last_master_cmd_tick = HAL_GetTick();                                                                   
                    has_received_master_cmd = 1; 
                    pylon_send_response_internal(port, ver, reply_addr, cid1, PYLON_RTN_NORMAL, NULL, 0);
                    break;
                }

            case PYLON_CID2_SYSTEM_PARAM:
                {   
                    pylon_rs485_system_param_t sp = {0};
                    sp.cell_high_v_limit   = 3650; sp.cell_low_v_limit  = 2800; sp.cell_under_v_limit = 2700;
                    sp.charge_high_temp    = 600;  sp.charge_low_temp   = 0;    sp.charge_current_lim = 12000; 
                    sp.module_high_v_lim   = 57600; sp.module_low_v_lim = 44800; sp.module_under_v_lim = 44800;
                    sp.dischg_high_temp    = 650;  sp.dischg_low_temp   = -200; sp.dischg_current_lim = 12000;
                    out_len = pylon_rs485_system_param_pack(info_out, sizeof(info_out), &sp);
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
                    out_len = pylon_rs485_manufacturer_pack(info_out, sizeof(info_out), &mfg);
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
                    out_len = pylon_rs485_soft_ver_pack(info_out, sizeof(info_out), &fw);
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
                    out_len = pylon_rs485_sys_basic_pack(info_out, sizeof(info_out), &basic); 
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
                    uint8_t group_id          = 0x01;
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

                    out_len = pylon_rs485_sys_analog_pack(info_out, sizeof(info_out), &sys);
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
                    
                    out_len = pylon_rs485_sys_alarm_pack(info_out, sizeof(info_out), &s_alarm);
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
                    
                    out_len = pylon_rs485_sys_mgmt_pack(info_out, sizeof(info_out), &s_mgmt);
                    break;
                }
                                
            case PYLON_CID2_SYS_SHUTDOWN: 
                {
                    out_len = pylon_rs485_sys_shutdown_pack(info_out, sizeof(info_out), 1);
                    system_is_shutting_down = 1;
                    CommandSubcommands(ALL_FETS_OFF); 
                    HAL_GPIO_WritePin(GPIOA, CFETOFF_Pin, GPIO_PIN_SET); 
                    HAL_GPIO_WritePin(GPIOA, DFETOFF_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOB, EN_PRECHARGE_Pin, GPIO_PIN_RESET);
                    break;
                }
                                
            case PYLON_CID2_SOFT_RESET:
                {
                    pylon_send_response_internal(port, ver, reply_addr, cid1, PYLON_RTN_NORMAL, NULL, 0);                                    
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
                
        if (out_len >= 0) {
            pylon_send_response_internal(port, ver, reply_addr, cid1, rtn, info_out, (uint16_t)out_len);
        }
        *idx = 0;
    }
}

void Pack_Svc_Broadcast_Master_Analog(void) {
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
    master_ana.current                  = (int16_t)(Pack_Current); 
    master_ana.total_voltage     = Stack_Voltage;   
    uint32_t rem_cap_full          = (uint32_t)(FullChargeCapacity_mAh * (SOC / 100.0f)); 
    uint32_t tot_cap_full          = (uint32_t)(FullChargeCapacity_mAh);   
    master_ana.remain_cap_1      = 0xFFFF;
    master_ana.user_def_count  = 4; 
    master_ana.total_cap_1       = 0xFFFF;
    master_ana.cycle_count               = (uint16_t)cycle_count;    
    master_ana.remain_cap_2[0] = (uint8_t)((rem_cap_full >> 16) & 0xFF);
    master_ana.remain_cap_2[1] = (uint8_t)((rem_cap_full >> 8) & 0xFF);
    master_ana.remain_cap_2[2] = (uint8_t)(rem_cap_full & 0xFF);
    master_ana.total_cap_2[0]  = (uint8_t)((tot_cap_full >> 16) & 0xFF);
    master_ana.total_cap_2[1]  = (uint8_t)((tot_cap_full >> 8) & 0xFF);
    master_ana.total_cap_2[2]  = (uint8_t)(tot_cap_full & 0xFF);
    int out_len = pylon_rs485_analog_pack(info_out, sizeof(info_out), &master_ana);   
    if (out_len > 0) {
        pylon_send_response_internal(RS485_PORT_UART5, PYLON_VER, 0xFF, PYLON_CID1_BATTERY, PYLON_CID2_MASTER_ANALOG, info_out, (uint16_t)out_len);
    }
}

void Pack_Svc_Send_Assign_ID(uint8_t id_to_assign) {
    uint8_t assign_info[2];
    pylon_rs485_byte_to_hex(id_to_assign, assign_info);
    pylon_send_response_internal(RS485_PORT_UART5, PYLON_VER, 0xFF, PYLON_CID1_BATTERY, PYLON_CID2_ASSIGN_ID, assign_info, 2);
}

int Pack_Svc_Request_Data(uint8_t target_adr, uint8_t cid2) {
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
        pylon_rs485_byte_to_hex(target_adr, info_ascii);
        info_ascii_len = 2;
    }
    pylon_send_response_internal(RS485_PORT_UART5, PYLON_VER, target_adr, PYLON_CID1_BATTERY, cid2, info_ascii, info_ascii_len);    
    return 0;
}

int Pack_Svc_Request_Set_Mgmt(uint8_t target_adr, pylon_rs485_set_mgmt_t *cmd) {
    uint8_t info_ascii[64] = {0};
    int out_len = pylon_rs485_set_mgmt_pack(info_ascii, sizeof(info_ascii), cmd);   
    if (out_len > 0) {
        pylon_send_response_internal(RS485_PORT_UART5, PYLON_VER, target_adr, PYLON_CID1_BATTERY, PYLON_CID2_SET_MGMT, info_ascii, (uint16_t)out_len);
        return 0;
    }
    return -1;
}

int Pack_Svc_Request_Soft_Reset(uint8_t target_adr, uint8_t payload) {
    uint8_t info_ascii[2];
    pylon_rs485_byte_to_hex(payload, info_ascii);
    pylon_send_response_internal(RS485_PORT_UART5, PYLON_VER, target_adr, PYLON_CID1_BATTERY, PYLON_CID2_SOFT_RESET, info_ascii, 2);
    return 0;
}

void Pack_Svc_Update(void) {
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
                Pack_Svc_Request_Data(0x03 + i, PYLON_CID2_TURN_OFF);
            }
        }
    }
}

#endif /* IS_BOOTLOADER == 0 */
