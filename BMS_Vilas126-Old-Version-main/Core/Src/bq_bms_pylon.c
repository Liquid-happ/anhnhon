#include "stm32f1xx_hal.h"
#include "bq_bms_pylon.h"
#include "pylon_can.h"
#include "bq_bms_485.h"
#include "main.h"

extern CAN_TxHeaderTypeDef 					TxHeader;
static uint8_t 											TxData[8];
static uint32_t 										TxMailbox;
//static uint8_t alive_counter = 0;

void TX_CAN_Message(CAN_HandleTypeDef *hcan)
{
    uint32_t tickstart = HAL_GetTick();  
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        if ((HAL_GetTick() - tickstart) > 3) return;
    }
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        return; 
    }
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//        return;
//				tickstart = HAL_GetTick();
//    while (HAL_CAN_IsTxMessagePending(hcan, TxMailbox))
//    {
//        if ((HAL_GetTick() - tickstart) > 10) break;
//    }
}

void Format_ASCII_Address(uint8_t pack_id, uint8_t cell_id, uint8_t *out_data) {
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

void Tx_BQ_BMS_Status_via_CAN(CAN_HandleTypeDef *hcan) {
		if (is_master == 0)			
		{
    return; 
    }
    // ===== FRAME 0x351 - LIMITS =====
    struct pylon_can_battery_limits_t battery_limits = {
			  .battery_charge_voltage           = system_charge_v_limit_final / 100, 
        .battery_discharge_voltage        = system_discharge_v_limit_final / 100, 
        .battery_charge_current_limit     = system_charge_limit_A_final * 10,
				.battery_discharge_current_limit  = system_discharge_limit_A_final * 10			
    };
    Create_Limits_Frame(&TxHeader, TxData, &battery_limits);
    TX_CAN_Message(hcan);

    // ===== FRAME 0x355 - SOC/SOH =====
    struct pylon_can_battery_so_c_so_h_t battery_soc_soh = {
        .so_c = system_avg_soc, 
        .so_h = system_avg_soh  		
    };
    Create_SOH_SOC_Frame(&TxHeader, TxData, &battery_soc_soh);
    TX_CAN_Message(hcan);

    // ===== FRAME 0x356 - ACTUAL VALUES =====
    struct pylon_can_battery_actual_values_u_it_t actual_values = {
        .battery_voltage     = system_total_voltage_01V / 10,      		 							
				.battery_current 		 = (int16_t)(system_total_current_01A / 10),
        .battery_temperature = (int16_t)(system_max_temp_C * 10)
    };
    Create_Actual_Values_Frame(&TxHeader, TxData, &actual_values);
    TX_CAN_Message(hcan);

    // ===== FRAME 0x35E - MANUFACTURER =====
    Create_Manufacturer_Frame(&TxHeader, TxData);
    TX_CAN_Message(hcan);

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
				.charge_enable 			 = allow_charge,
        .discharge_enable 	 = allow_discharge
    };
    Create_Battery_Request_Frame(&TxHeader, TxData, &battery_request);
    TX_CAN_Message(hcan);

//    // ===== FRAME 0x305 - ALIVE =====
//    struct pylon_can_network_alive_msg_t alive_msg = { .alive_packet = alive_counter++ };
//    Create_Alive_Msg_Frame(&TxHeader, TxData, &alive_msg);
//    TX_CAN_Message(hcan);

    // ===== FRAME 0x359 - ALARM & WARNING (CHUAN PYLONTECH v1.2) =====
    struct pylon_can_battery_error_warnings_t errors_warnings = {
			  .overvoltage_err              = sys_protect_ov,
        .undervoltage_err             = sys_protect_uv,
        .overtemperature_err          = sys_protect_ot,
        .undertemperature_err         = sys_protect_ut,
        .overcurrent_discharge_err    = sys_protect_ocd,
        .charge_overcurrent_err       = sys_protect_occ,
				.system_error 								= PFErrorsTriggered || sys_system_error,
        .voltage_high_warn            = sys_alarm_ov,
        .voltage_low_warn             = sys_alarm_uv,
        .temperature_high_warn        = sys_alarm_ot,
        .temperature_low_warn         = sys_alarm_ut,
        .discharge_current_high_warn  = sys_alarm_ocd,
        .charge_current_high_warn     = sys_alarm_occ,
				.internal_error_warn          = inverter_comm_fault ? 1 : 0,
			  .module_numbers 						  = active_packs_count
    };
    Create_Errors_Warnings_Frame(&TxHeader, TxData, &errors_warnings);
    TX_CAN_Message(hcan);
		
		static uint32_t last_ext_can_tick = 0;
    if (HAL_GetTick() - last_inverter_alive_tick < 5000) 
    {
        if (HAL_GetTick() - last_ext_can_tick >= 2000) 
        {
            last_ext_can_tick = HAL_GetTick();
            
            // --- 1. FRAME 0x350 - CUSTOM FLAGS ---
            memset(TxData, 0, 8);
            if (fet_fail_timer > 0) {
                TxData[0] |= 0xC0; // Bit 6, 7 (MOSFAIL)
            }
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.StdId = PYLON_CAN_BATTERY_CUSTOM_FLAGS_FRAME_ID;
            TxHeader.DLC = PYLON_CAN_BATTERY_CUSTOM_FLAGS_LENGTH;
            TX_CAN_Message(hcan);

            // --- 2. FRAME 0x35A - ALARM 2-BIT ---
            memset(TxData, 0, 8);
            TxData[0] |= 0x02;                                  // Bit 0~1: Reserved (10)
            TxData[0] |= sys_protect_ov ? 0x04 : 0x08;          // Bit 2~3: High Vol Protect
            TxData[0] |= sys_protect_uv ? 0x10 : 0x20;          // Bit 4~5: Low Vol Protect
            TxData[0] |= sys_protect_ot ? 0x40 : 0x80;          // Bit 6~7: High Temp Protect

            TxData[1] |= sys_protect_ut ? 0x01 : 0x02;          // Bit 0~1: Low Temp Protect
            TxData[1] |= 0x08;                                  // Bit 2~3: Charge High Temp (Reserved)
            TxData[1] |= 0x20;                                  // Bit 4~5: Charge Low Temp (Reserved)
            TxData[1] |= sys_protect_ocd ? 0x40 : 0x80;         // Bit 6~7: High Current Protect

            TxData[2] |= sys_alarm_occ ? 0x01 : 0x02;           // Bit 0~1: Charge Current Alarm
            TxData[2] |= (fet_fail_timer > 0) ? 0x04 : 0x08;    // Bit 2~3: Contactor Error (MOSFAIL)
            TxData[2] |= sys_protect_ocd ? 0x10 : 0x20;         // Bit 4~5: Short Circuit (Dùng t?m OCD)
            TxData[2] |= (PFErrorsTriggered || sys_system_error) ? 0x40 : 0x80; // Bit 6~7: BMS Protect

            TxData[3] |= 0xAA;                                  // Toàn bo Byte 3 Reserved (10101010)

            TxHeader.StdId = PYLON_CAN_BATTERY_EXT_ALARM_FRAME_ID;
            TxHeader.DLC = PYLON_CAN_BATTERY_EXT_ALARM_LENGTH;
            TX_CAN_Message(hcan);

            // --- 3. FRAME 0x372 - MODULE STATUS ---
            memset(TxData, 0, 8);
            TxData[0] = active_packs_count & 0xFF;
            TxData[1] = (active_packs_count >> 8) & 0xFF;

            uint16_t isolated_count = 0;
            for(int i = 0; i < 3; i++) {
                if(slave_isolated[i]) isolated_count++;
            }
            TxData[6] = isolated_count & 0xFF;
            TxData[7] = (isolated_count >> 8) & 0xFF;

            TxHeader.StdId = PYLON_CAN_BATTERY_MODULE_STATUS_FRAME_ID;
            TxHeader.DLC = PYLON_CAN_BATTERY_MODULE_STATUS_LENGTH;
            TX_CAN_Message(hcan);

            // --- 4. FRAME 0x373 - EXTREMES ANALOG ---
            memset(TxData, 0, 8);
            TxData[0] = sys_min_cell_v & 0xFF;
            TxData[1] = (sys_min_cell_v >> 8) & 0xFF;
            TxData[2] = sys_max_cell_v & 0xFF;
            TxData[3] = (sys_max_cell_v >> 8) & 0xFF;

            uint16_t min_temp_K = (system_min_temp_C) + 273;
            uint16_t max_temp_K = (system_max_temp_C) + 273;
            TxData[4] = min_temp_K & 0xFF;
            TxData[5] = (min_temp_K >> 8) & 0xFF;
            TxData[6] = max_temp_K & 0xFF;
            TxData[7] = (max_temp_K >> 8) & 0xFF;
            
            TxHeader.StdId = PYLON_CAN_BATTERY_EXTREMES_FRAME_ID;
            TxHeader.DLC = PYLON_CAN_BATTERY_EXTREMES_LENGTH;
            TX_CAN_Message(hcan);

            // --- 5. FRAME 0x374 ~ 0x377 - ADDRESS OF EXTREMES ---          
            memset(TxData, 0, 8);
            Format_ASCII_Address(sys_min_v_pack_id, sys_min_v_cell_id, TxData);
            TxHeader.StdId = PYLON_CAN_BATTERY_MIN_V_ADDR_FRAME_ID;
            TxHeader.DLC = PYLON_CAN_BATTERY_MIN_V_ADDR_LENGTH;
            TX_CAN_Message(hcan);

            memset(TxData, 0, 8);
            Format_ASCII_Address(sys_max_v_pack_id, sys_max_v_cell_id, TxData);
            TxHeader.StdId = PYLON_CAN_BATTERY_MAX_V_ADDR_FRAME_ID;
            TX_CAN_Message(hcan);

            memset(TxData, 0, 8);
            Format_ASCII_Address(sys_min_t_pack_id, sys_min_t_cell_id, TxData);
            TxHeader.StdId = PYLON_CAN_BATTERY_MIN_T_ADDR_FRAME_ID;
            TX_CAN_Message(hcan);

            memset(TxData, 0, 8);
            Format_ASCII_Address(sys_max_t_pack_id, sys_max_t_cell_id, TxData);
            TxHeader.StdId = PYLON_CAN_BATTERY_MAX_T_ADDR_FRAME_ID;
            TX_CAN_Message(hcan);

            // --- 6. FRAME 0x379 - TOTAL CAPACITY ---
            memset(TxData, 0, 8);
            uint32_t total_capacity_Ah = system_total_capacity_mAh / 1000;

//						TxData[0] = total_capacity_Ah & 0xFF;    
//						TxData[1] = (total_capacity_Ah >> 8) & 0xFF;  
//						TxData[2] = (total_capacity_Ah >> 16) & 0xFF; 
//						TxData[3] = (total_capacity_Ah >> 24) & 0xFF; 

						TxData[0] = 0x00; 
            TxData[1] = 0x00;
            TxData[2] = total_capacity_Ah & 0xFF;
						TxData[3] = (total_capacity_Ah >> 8) & 0xFF;
						
						TxHeader.StdId = PYLON_CAN_BATTERY_TOTAL_CAPACITY_FRAME_ID;
						TxHeader.DLC = PYLON_CAN_BATTERY_TOTAL_CAPACITY_LENGTH;
						TX_CAN_Message(hcan);
        }
    }
}

//0x351 – 14 02 74 0E 74 0E CC 01 – Battery voltage + current limits
void Create_Limits_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_limits_t *battery_limits) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_BATTERY_LIMITS_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_battery_limits_pack(dst_p, battery_limits, PYLON_CAN_BATTERY_LIMITS_LENGTH);
}

//0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void Create_SOH_SOC_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_so_c_so_h_t *battery_soc_soh) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_BATTERY_SO_C_SO_H_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_battery_so_c_so_h_pack(dst_p, battery_soc_soh, PYLON_CAN_BATTERY_SO_C_SO_H_LENGTH);
}

//0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
void Create_Actual_Values_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_actual_values_u_it_t *actual_values) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_BATTERY_ACTUAL_VALUES_U_IT_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_battery_actual_values_u_it_pack(dst_p, actual_values, PYLON_CAN_BATTERY_ACTUAL_VALUES_U_IT_LENGTH);
}

//0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name (“PYLON “)
void Create_Manufacturer_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p) {
	struct pylon_can_battery_manufacturer_t manufacturer = {
			.manufacturer_string = 0x2020204E4F4C5950ULL
	};
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_BATTERY_MANUFACTURER_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_battery_manufacturer_pack(dst_p, &manufacturer, PYLON_CAN_BATTERY_MANUFACTURER_LENGTH);
}

//0x35C – C0 00 – Battery charge request flags
void Create_Battery_Request_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_request_t *battery_request) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_BATTERY_REQUEST_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_battery_request_pack(dst_p, battery_request, PYLON_CAN_BATTERY_REQUEST_LENGTH);
}

//0x305 – alive message
void Create_Alive_Msg_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_network_alive_msg_t *alive_msg) {
//void Create_Alive_Msg_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p) {
//	struct pylon_can_network_alive_msg_t alive_message = {
//		    .alive_packet = 33
//	};
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_NETWORK_ALIVE_MSG_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_network_alive_msg_pack(dst_p, alive_msg, PYLON_CAN_NETWORK_ALIVE_MSG_LENGTH); 
}

//0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
void Create_Errors_Warnings_Frame(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_error_warnings_t *errors_warnings) {
	TxHeader->IDE = CAN_ID_STD;
	TxHeader->StdId = PYLON_CAN_BATTERY_ERROR_WARNINGS_FRAME_ID;
	TxHeader->RTR = CAN_RTR_DATA;
	TxHeader->DLC = pylon_can_battery_error_warnings_pack(dst_p, errors_warnings, PYLON_CAN_BATTERY_ERROR_WARNINGS_LENGTH);
}
