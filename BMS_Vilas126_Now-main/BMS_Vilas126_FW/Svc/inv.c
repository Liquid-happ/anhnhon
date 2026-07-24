#include "Svc/inv.h"
#include "Drv/can_drv.h"
#include "Proto/pylon_can.h"
#include "Core/Inc/bms_state.h"
#include "main.h"

#if IS_BOOTLOADER == 0

#include <string.h>

/* --- PRIVATE HELPER --- */
static void Format_ASCII_Address(uint8_t pack_id, uint8_t cell_id, uint8_t *out_data) {
    uint8_t group_id = 1;
    out_data[0] = '0' + (group_id / 10);
    out_data[1] = '0' + (group_id % 10);
    out_data[2] = '0' + (pack_id / 10);
    out_data[3] = '0' + (pack_id % 10);
    out_data[4] = 0x00;
    out_data[5] = 0x00;
    out_data[6] = 0x00; 
    out_data[7] = 0x00; 
}

/* --- SERVICE FUNCTIONS --- */

void Inv_Init(void) {
    inverter_comm_fault = 0;
    last_inverter_alive_tick = 0;
}

void Inv_Transmit_Status(void) {
    if (is_master == 0) return;

    uint8_t tx_data[8];

    // ===== FRAME 0x351 - LIMITS =====
    struct pylon_can_battery_limits_t battery_limits = {
        .battery_charge_voltage           = system_charge_v_limit_final / 100, 
        .battery_discharge_voltage        = system_discharge_v_limit_final / 100, 
        .battery_charge_current_limit     = (int16_t)(system_charge_limit_A_final * 10),
        .battery_discharge_current_limit  = (int16_t)(system_discharge_limit_A_final * 10)         
    };
    pylon_can_battery_limits_pack(tx_data, &battery_limits, 8);
    Can_Drv_Transmit(PYLON_CAN_BATTERY_LIMITS_FRAME_ID, 0, 0, tx_data, 8);

    // ===== FRAME 0x355 - SOC/SOH =====
    struct pylon_can_battery_so_c_so_h_t battery_soc_soh = {
        .so_c = system_avg_soc, 
        .so_h = system_avg_soh          
    };
    pylon_can_battery_so_c_so_h_pack(tx_data, &battery_soc_soh, 8);
    Can_Drv_Transmit(PYLON_CAN_BATTERY_SO_C_SO_H_FRAME_ID, 0, 0, tx_data, 8);

    // ===== FRAME 0x356 - ACTUAL VALUES =====
    struct pylon_can_battery_actual_values_u_it_t actual_values = {
        .battery_voltage     = (int16_t)(system_total_voltage_01V / 10),                                       
        .battery_current      = (int16_t)(system_total_current_01A / 10),
        .battery_temperature = (int16_t)(system_max_temp_C * 10)
    };
    pylon_can_battery_actual_values_u_it_pack(tx_data, &actual_values, 8);
    Can_Drv_Transmit(PYLON_CAN_BATTERY_ACTUAL_VALUES_U_IT_FRAME_ID, 0, 0, tx_data, 8);

    // ===== FRAME 0x35E - MANUFACTURER =====
    struct pylon_can_battery_manufacturer_t manufacturer = {
        .manufacturer_string = 0x2020204E4F4C5950ULL
    };
    pylon_can_battery_manufacturer_pack(tx_data, &manufacturer, 8);
    Can_Drv_Transmit(PYLON_CAN_BATTERY_MANUFACTURER_FRAME_ID, 0, 0, tx_data, 8);

    // ===== FRAME 0x35C - REQUEST =====
    uint8_t current_soc = system_avg_soc;
    if (current_soc <= 9) {
        force_charge = 1; 
    } 
    else if (current_soc >= 13) {
        force_charge = 0; 
    }
    uint8_t allow_charge = !(OV_Fault || OCC_Fault || OTC_Fault || SCD_Fault || SCDL_Fault || UTC_Fault || OTF_Fault || OTINT_Fault || UTINT_Fault || (bms_state == BMS_STATE_FAULT));
    uint8_t allow_discharge = !(UV_Fault || OCD_Fault || OCD_Fault1 || SCD_Fault || OTD_Fault || UTD_Fault || OTF_Fault || OTINT_Fault || UTINT_Fault || SCDL_Fault || OCDL_Fault || (bms_state == BMS_STATE_FAULT));       
    struct pylon_can_battery_request_t battery_request = {
        .full_charge_req     = full_charge_request,
        .force_charge_req_ii = force_charge,
        .force_charge_req_i  = 0,
        .charge_enable        = allow_charge,
        .discharge_enable     = allow_discharge
    };
    pylon_can_battery_request_pack(tx_data, &battery_request, 8);
    Can_Drv_Transmit(PYLON_CAN_BATTERY_REQUEST_FRAME_ID, 0, 0, tx_data, 8);

    // ===== FRAME 0x359 - ALARM & WARNING =====
    struct pylon_can_battery_error_warnings_t errors_warnings = {
        .overvoltage_err              = sys_protect_ov,
        .undervoltage_err             = sys_protect_uv,
        .overtemperature_err          = sys_protect_ot,
        .undertemperature_err         = sys_protect_ut,
        .overcurrent_discharge_err    = sys_protect_ocd,
        .charge_overcurrent_err       = sys_protect_occ,
        .system_error                 = PFErrorsTriggered || sys_system_error,
        .voltage_high_warn            = sys_alarm_ov,
        .voltage_low_warn             = sys_alarm_uv,
        .temperature_high_warn        = sys_alarm_ot,
        .temperature_low_warn         = sys_alarm_ut,
        .discharge_current_high_warn  = sys_alarm_ocd,
        .charge_current_high_warn     = sys_alarm_occ,
        .internal_error_warn          = inverter_comm_fault ? 1 : 0,
        .module_numbers               = active_packs_count
    };
    pylon_can_battery_error_warnings_pack(tx_data, &errors_warnings, 8);
    Can_Drv_Transmit(PYLON_CAN_BATTERY_ERROR_WARNINGS_FRAME_ID, 0, 0, tx_data, 8);
        
    static uint32_t last_ext_can_tick = 0;
    if (HAL_GetTick() - last_inverter_alive_tick < 5000) 
    {
        if (HAL_GetTick() - last_ext_can_tick >= 2000) 
        {
            last_ext_can_tick = HAL_GetTick();
            
            // --- 1. FRAME 0x350 - CUSTOM FLAGS ---
            memset(tx_data, 0, 8);
            if (fet_fail_timer > 0) {
                tx_data[0] |= 0xC0; // Bit 6, 7 (MOSFAIL)
            }
            Can_Drv_Transmit(PYLON_CAN_BATTERY_CUSTOM_FLAGS_FRAME_ID, 0, 0, tx_data, 8);

            // --- 2. FRAME 0x35A - ALARM 2-BIT ---
            memset(tx_data, 0, 8);
            tx_data[0] |= 0x02;                                  // Bit 0~1: Reserved (10)
            tx_data[0] |= sys_protect_ov ? 0x04 : 0x08;          // Bit 2~3: High Vol Protect
            tx_data[0] |= sys_protect_uv ? 0x10 : 0x20;          // Bit 4~5: Low Vol Protect
            tx_data[0] |= sys_protect_ot ? 0x40 : 0x80;          // Bit 6~7: High Temp Protect

            tx_data[1] |= sys_protect_ut ? 0x01 : 0x02;          // Bit 0~1: Low Temp Protect
            tx_data[1] |= 0x08;                                  // Bit 2~3: Charge High Temp (Reserved)
            tx_data[1] |= 0x20;                                  // Bit 4~5: Charge Low Temp (Reserved)
            tx_data[1] |= sys_protect_ocd ? 0x40 : 0x80;         // Bit 6~7: High Current Protect

            tx_data[2] |= sys_alarm_occ ? 0x01 : 0x02;           // Bit 0~1: Charge Current Alarm
            tx_data[2] |= (fet_fail_timer > 0) ? 0x04 : 0x08;    // Bit 2~3: Contactor Error (MOSFAIL)
            tx_data[2] |= sys_protect_ocd ? 0x10 : 0x20;         // Bit 4~5: Short Circuit (Dùng t?m OCD)
            tx_data[2] |= (PFErrorsTriggered || sys_system_error) ? 0x40 : 0x80; // Bit 6~7: BMS Protect

            tx_data[3] |= 0xAA;                                  // Byte 3 Reserved (10101010)
            Can_Drv_Transmit(PYLON_CAN_BATTERY_EXT_ALARM_FRAME_ID, 0, 0, tx_data, 8);

            // --- 3. FRAME 0x372 - MODULE STATUS ---
            memset(tx_data, 0, 8);
            tx_data[0] = active_packs_count & 0xFF;
            tx_data[1] = (active_packs_count >> 8) & 0xFF;

            uint16_t isolated_count = 0;
            for(int i = 0; i < 3; i++) {
                if(slave_isolated[i]) isolated_count++;
            }
            tx_data[6] = isolated_count & 0xFF;
            tx_data[7] = (isolated_count >> 8) & 0xFF;
            Can_Drv_Transmit(PYLON_CAN_BATTERY_MODULE_STATUS_FRAME_ID, 0, 0, tx_data, 8);

            // --- 4. FRAME 0x373 - EXTREMES ANALOG ---
            memset(tx_data, 0, 8);
            tx_data[0] = sys_min_cell_v & 0xFF;
            tx_data[1] = (sys_min_cell_v >> 8) & 0xFF;
            tx_data[2] = sys_max_cell_v & 0xFF;
            tx_data[3] = (sys_max_cell_v >> 8) & 0xFF;

            uint16_t min_temp_K = (system_min_temp_C) + 273;
            uint16_t max_temp_K = (system_max_temp_C) + 273;
            tx_data[4] = min_temp_K & 0xFF;
            tx_data[5] = (min_temp_K >> 8) & 0xFF;
            tx_data[6] = max_temp_K & 0xFF;
            tx_data[7] = (max_temp_K >> 8) & 0xFF;
            Can_Drv_Transmit(PYLON_CAN_BATTERY_EXTREMES_FRAME_ID, 0, 0, tx_data, 8);

            // --- 5. FRAME 0x374 ~ 0x377 - ADDRESS OF EXTREMES ---          
            memset(tx_data, 0, 8);
            Format_ASCII_Address(sys_min_v_pack_id, sys_min_v_cell_id, tx_data);
            Can_Drv_Transmit(PYLON_CAN_BATTERY_MIN_V_ADDR_FRAME_ID, 0, 0, tx_data, 8);

            memset(tx_data, 0, 8);
            Format_ASCII_Address(sys_max_v_pack_id, sys_max_v_cell_id, tx_data);
            Can_Drv_Transmit(PYLON_CAN_BATTERY_MAX_V_ADDR_FRAME_ID, 0, 0, tx_data, 8);

            memset(tx_data, 0, 8);
            Format_ASCII_Address(sys_min_t_pack_id, sys_min_t_cell_id, tx_data);
            Can_Drv_Transmit(PYLON_CAN_BATTERY_MIN_T_ADDR_FRAME_ID, 0, 0, tx_data, 8);

            memset(tx_data, 0, 8);
            Format_ASCII_Address(sys_max_t_pack_id, sys_max_t_cell_id, tx_data);
            Can_Drv_Transmit(PYLON_CAN_BATTERY_MAX_T_ADDR_FRAME_ID, 0, 0, tx_data, 8);

            // --- 6. FRAME 0x379 - TOTAL CAPACITY ---
            memset(tx_data, 0, 8);
            uint32_t total_capacity_Ah = system_total_capacity_mAh / 1000;
            tx_data[0] = 0x00; 
            tx_data[1] = 0x00;
            tx_data[2] = total_capacity_Ah & 0xFF;
            tx_data[3] = (total_capacity_Ah >> 8) & 0xFF;
            Can_Drv_Transmit(PYLON_CAN_BATTERY_TOTAL_CAPACITY_FRAME_ID, 0, 0, tx_data, 8);
        }
    }
}

void Inv_Handle_Rx_Frame(uint32_t std_id, uint32_t ext_id, uint8_t is_ext, uint8_t *data, uint8_t len) {
    (void)data;
    (void)len;
    if (!is_ext && std_id == 0x305) {
        last_inverter_alive_tick = HAL_GetTick();
        inverter_comm_fault = 0;
    }
    if (is_ext) {
        if ((ext_id & 0xFF000000) == 0x04000000) {
            uint8_t req_group = (ext_id >> 16) & 0xFF; 
            uint8_t req_pack  = (ext_id >> 8) & 0xFF;  

            if (req_group == 1 && req_pack >= 1 && req_pack <= MAX_SLAVES + 1) {
                uint16_t p_min_cv = 0, p_max_cv = 0, p_vol = 0;
                int16_t p_curr = 0, p_max_t = 0, p_min_t = 0, p_mos_t = 0, p_bms_t = 0;
                uint8_t p_soc = 0, p_soh = 0;

                // Import slave_online_status and slave_analog_data from pack service
                extern uint8_t slave_online_status[MAX_SLAVES];
                extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];

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

                uint8_t ext_tx_data[8] = {0};
                uint32_t resp_ext_id = 0x04000001 + (req_pack << 8) + (req_group << 16);
                ext_tx_data[0] = p_min_cv & 0xFF; ext_tx_data[1] = (p_min_cv >> 8) & 0xFF;
                ext_tx_data[2] = p_max_cv & 0xFF; ext_tx_data[3] = (p_max_cv >> 8) & 0xFF;
                ext_tx_data[4] = p_curr & 0xFF;   ext_tx_data[5] = (p_curr >> 8) & 0xFF;
                ext_tx_data[6] = p_vol & 0xFF;    ext_tx_data[7] = (p_vol >> 8) & 0xFF;
                Can_Drv_Transmit(0, resp_ext_id, 1, ext_tx_data, 8);

                resp_ext_id = 0x04000002 + (req_pack << 8) + (req_group << 16);
                ext_tx_data[0] = p_max_t & 0xFF; ext_tx_data[1] = (p_max_t >> 8) & 0xFF;
                ext_tx_data[2] = p_min_t & 0xFF; ext_tx_data[3] = (p_min_t >> 8) & 0xFF;
                ext_tx_data[4] = p_mos_t & 0xFF; ext_tx_data[5] = (p_mos_t >> 8) & 0xFF;
                ext_tx_data[6] = p_bms_t & 0xFF; ext_tx_data[7] = (p_bms_t >> 8) & 0xFF;
                Can_Drv_Transmit(0, resp_ext_id, 1, ext_tx_data, 8);

                resp_ext_id = 0x04000003 + (req_pack << 8) + (req_group << 16);
                memset(ext_tx_data, 0, 8);
                ext_tx_data[2] = p_soc & 0xFF; 
                ext_tx_data[3] = (p_soc >> 8) & 0xFF; 
                ext_tx_data[4] = p_soh;
                uint16_t pack_nom_cap;
                if (req_pack == 1) {
                    pack_nom_cap = (uint16_t)(FullChargeCapacity_mAh / 1000);
                } else {
                    extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
                    pack_nom_cap = (uint16_t)(slave_analog_data[req_pack-2].total_cap_1); 
                }
                ext_tx_data[6] = pack_nom_cap & 0xFF; 
                ext_tx_data[7] = (pack_nom_cap >> 8) & 0xFF;
                Can_Drv_Transmit(0, resp_ext_id, 1, ext_tx_data, 8);

                resp_ext_id = 0x04000004 + (req_pack << 8) + (req_group << 16);
                memset(ext_tx_data, 0, 8);
                Can_Drv_Transmit(0, resp_ext_id, 1, ext_tx_data, 8);
            }
        }
    }
}

/* --- COMPATIBILITY SHIM FOR CORE/SRC/MAIN.C / BQ_BMS_PYLON.C --- */
void Tx_BQ_BMS_Status_via_CAN(CAN_HandleTypeDef *hcan) {
    (void)hcan;
    Inv_Transmit_Status();
}

#endif /* IS_BOOTLOADER == 0 */
