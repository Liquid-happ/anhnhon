#ifndef SVC_PACK_H_
#define SVC_PACK_H_

#include "Cfg/feat_cfg.h"

#if IS_BOOTLOADER == 0

#include <stdint.h>
#include <stdbool.h>
#include "Proto/pylon_485.h"

#define MAX_SLAVES 15

typedef enum {
    AUTO_CODE_START,
    AUTO_CODE_WAIT_ADDRESS_CMD,
    AUTO_CODE_DONE
} Auto_Code_State_t;

// Extern variables representing the state of the pack service
extern Auto_Code_State_t auto_code_state;
extern uint8_t is_auto_coding;
extern volatile uint8_t assigned_n;
extern uint8_t bms_device_address;
extern uint8_t pylon_protocol_addr;
extern uint8_t current_assigning_index;
extern uint8_t is_master;
extern uint8_t active_packs_count;

extern uint32_t last_master_cmd_tick;
extern uint8_t has_received_master_cmd;
extern volatile uint8_t rs485_rx_success_flag;

// Slave metrics storage (for Master)
extern pylon_rs485_analog_t slave_analog_data[MAX_SLAVES];
extern pylon_rs485_chg_dis_mgmt_t slave_mgmt_data[MAX_SLAVES];
extern pylon_rs485_alarm_t slave_alarm_data[MAX_SLAVES];
extern uint8_t slave_online_status[MAX_SLAVES];

// Master parameters storage (for Slave)
extern pylon_rs485_analog_t master_analog_data;
extern pylon_rs485_set_mgmt_t last_master_cmd;

// Statistics and error counts
extern uint32_t err_len_field;
extern uint32_t err_chksum_alarm;
extern uint32_t err_chksum_mgmt;

/**
 * @brief Initializes the Pack Service layer.
 */
void Pack_Svc_Init(void);

/**
 * @brief Processes a received byte from RS485 interfaces (UART4 / UART5).
 * @param port Either RS485_PORT_UART4 or RS485_PORT_UART5.
 * @param byte The received data byte.
 */
void Pack_Svc_Process_Byte(uint8_t port, uint8_t byte);

/**
 * @brief Broadcasts master analog metrics to all slave packs.
 */
void Pack_Svc_Broadcast_Master_Analog(void);

/**
 * @brief Sends an ID assignment command to a slave.
 * @param id_to_assign The slave protocol address to assign.
 */
void Pack_Svc_Send_Assign_ID(uint8_t id_to_assign);

/**
 * @brief Requests standard telemetry/commands from a target pack.
 * @param target_adr The protocol address of the target pack.
 * @param cid2 Command ID 2.
 * @return 0 on success, non-zero on error.
 */
int Pack_Svc_Request_Data(uint8_t target_adr, uint8_t cid2);

/**
 * @brief Sends management parameter updates to a target slave pack.
 * @param target_adr The protocol address of the target pack.
 * @param cmd Pointer to the management configuration command struct.
 * @return 0 on success, non-zero on error.
 */
int Pack_Svc_Request_Set_Mgmt(uint8_t target_adr, pylon_rs485_set_mgmt_t *cmd);

/**
 * @brief Sends a soft reset command to a target pack.
 * @param target_adr The protocol address of the target pack.
 * @param payload Reset action payload byte.
 * @return 0 on success, non-zero on error.
 */
int Pack_Svc_Request_Soft_Reset(uint8_t target_adr, uint8_t payload);

/**
 * @brief Updates pack service states and performs periodic connection checks.
 *        Should be called in the main loop or timer task.
 */
void Pack_Svc_Update(void);

#endif /* IS_BOOTLOADER == 0 */
#endif /* SVC_PACK_H_ */
