#ifndef BQ_BMS_PYLON_H_
#define BQ_BMS_PYLON_H_

#include "../rd_ota/rd_control.h"

#if IS_BOOTLOADER == 0

#include "stm32f1xx_hal.h"
#include "pylon_can.h"
#include "bms_state.h"

extern uint8_t full_charge_request;
extern uint8_t force_charge;
extern uint8_t current_pf_status[4];
extern uint8_t bms_device_address;
extern uint8_t is_master;
extern uint16_t system_charge_limit_A;
extern uint16_t system_discharge_limit_A;
extern uint16_t system_charge_limit_A_final;
extern uint16_t system_discharge_limit_A_final;
extern uint16_t system_charge_v_limit_mV;
extern uint16_t system_discharge_v_limit_mV;
extern uint16_t system_charge_v_limit_final;
extern uint16_t system_discharge_v_limit_final;
extern int32_t system_total_current_mA;
extern int32_t system_total_current_01A;
extern uint16_t system_total_voltage_01V;
extern uint16_t system_avg_soc;          
extern uint16_t system_avg_soh;          
extern uint8_t active_packs_count;       
extern int16_t system_max_temp_C;
extern int16_t system_min_temp_C;         
extern uint16_t sys_min_cell_v;       
extern uint16_t sys_max_cell_v;
extern uint8_t sys_max_v_pack_id;
extern uint8_t sys_max_v_cell_id;
extern uint8_t sys_min_v_pack_id;
extern uint8_t sys_min_v_cell_id;
extern uint8_t sys_max_t_pack_id;
extern uint8_t sys_max_t_cell_id;
extern uint8_t sys_min_t_pack_id;
extern uint8_t sys_min_t_cell_id;
extern uint32_t fet_fail_timer;           
extern uint32_t last_inverter_alive_tick; 
extern uint8_t slave_isolated[];         
extern uint8_t PFErrorsTriggered; 
extern uint8_t inverter_comm_fault;
extern uint8_t uv_recovery_locked;
extern uint8_t ov_recovery_locked;
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
extern uint8_t sys_system_error;
extern uint32_t system_total_capacity_mAh;

void Tx_BQ_BMS_Status_via_CAN			(CAN_HandleTypeDef *hcan);
void Create_Limits_Frame					(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_limits_t 							*battery_limits);
void Create_SOH_SOC_Frame					(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_so_c_so_h_t 						*battery_soc_soh);
void Create_Actual_Values_Frame		(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_actual_values_u_it_t 	*actual_values);
void Create_Manufacturer_Frame		(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p);
void Create_Battery_Request_Frame	(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_request_t 							*battery_request);
//void Create_Alive_Msg_Frame				(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_network_alive_msg_t 						*alive_msg);
void Create_Errors_Warnings_Frame	(CAN_TxHeaderTypeDef *TxHeader, uint8_t *dst_p, const struct pylon_can_battery_error_warnings_t 			*errors_warnings);

#endif /* IS_BOOTLOADER == 0 */
#endif /* BQ_BMS_PYLON_H_ */
