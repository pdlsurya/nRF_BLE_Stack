/**
 * @file ble_controller_state_internal.h
 * @author Surya Poudel
 * @brief Controller-owned runtime state definitions for nRF BLE stack
 * @version 0.1
 * @date 2026-05-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_CONTROLLER_STATE_INTERNAL_H__
#define BLE_CONTROLLER_STATE_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gap.h"
#include "ble_ll_internal.h"

#define BLE_ADV_RX_WINDOW_US 1500U
#define BLE_CONN_EVENT_GUARD_US 5000U
#define BLE_CONN_TIMER_PRESCALER 4U
#define BLE_CONN_TIMER_IRQ_PRIORITY 0U
#define BLE_INITIATOR_WIN_OFFSET_UNITS 6U
#define BLE_INITIATOR_WIN_SIZE_UNITS 2U
#define BLE_INITIATOR_HOP_INCREMENT 5U
#define BLE_CONN_TX_L2CAP_QUEUE_DEPTH 4U

typedef enum
{
    BLE_ADV_RADIO_PHASE_IDLE = 0,
    BLE_ADV_RADIO_PHASE_WAIT_ADV_TX_DISABLED,
    BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED,
    BLE_ADV_RADIO_PHASE_WAIT_SCAN_RSP_TX_DISABLED,
} ble_adv_radio_phase_t;

typedef enum
{
    BLE_SCAN_RADIO_PHASE_IDLE = 0,
    BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED,
    BLE_SCAN_RADIO_PHASE_WAIT_SCAN_REQ_TX_DISABLED,
    BLE_SCAN_RADIO_PHASE_WAIT_SCAN_RSP_RX_DISABLED,
    BLE_SCAN_RADIO_PHASE_WAIT_CONNECT_TX_DISABLED,
} ble_scan_radio_phase_t;

typedef enum
{
    BLE_CONN_RADIO_PHASE_IDLE = 0,
    BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED,
    BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED,
} ble_conn_radio_phase_t;

typedef struct
{
    bool tx_acked;
    bool is_new_packet;
} ble_conn_bcmatch_state_t;

typedef enum
{
    BLE_CONN_PENDING_SOURCE_NONE = 0,
    BLE_CONN_PENDING_SOURCE_CONTROL_RESPONSE,
    BLE_CONN_PENDING_SOURCE_CONTROL,
    BLE_CONN_PENDING_SOURCE_L2CAP,
} ble_conn_pending_source_t;

typedef enum
{
    BLE_CENTRAL_CTRL_PROC_STATE_IDLE = 0,
    BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP,
    BLE_CENTRAL_CTRL_PROC_STATE_WAIT_INSTANT,
} ble_central_ctrl_proc_state_t;

typedef enum
{
    BLE_AUTO_CTRL_STAGE_IDLE = 0,
    BLE_AUTO_CTRL_STAGE_WAIT_FEATURES,
    BLE_AUTO_CTRL_STAGE_WAIT_DATA_LENGTH,
    BLE_AUTO_CTRL_STAGE_WAIT_PHY,
} ble_auto_ctrl_stage_t;

typedef struct
{
    ble_gap_ctrl_procedure_t procedure;
    ble_central_ctrl_proc_state_t state;
} ble_central_ctrl_proc_context_t;

typedef struct
{
    ble_ll_data_raw_pdu_t q[BLE_CONN_TX_L2CAP_QUEUE_DEPTH];
    uint8_t ridx;
    uint8_t widx;
    uint8_t count;
} ble_conn_l2cap_tx_queue_t;

typedef struct
{
    uint8_t addr[6];
    uint8_t txadd;
} ble_local_addr_runtime_t;

typedef struct
{
    ble_ll_adv_pdu_t adv_tx_pdu;
    ble_ll_scan_rsp_pdu_t scan_rsp_pdu;
    ble_ll_adv_req_pdu_t adv_rx_pdu;
    bool scan_rsp_tx_pending;
    bool connect_req_pending;
    ble_adv_radio_phase_t adv_radio_phase;
} ble_peripheral_ctrl_runtime_t;

typedef struct
{
    ble_ll_adv_pdu_t adv_rx_pdu;
    ble_ll_scan_req_pdu_t scan_req_pdu;
    ble_ll_scan_rsp_pdu_t scan_rsp_pdu;
    ble_ll_connect_req_pdu_t connect_req_pdu;
    uint8_t scan_channel_index;
    bool scanning;
    bool scan_req_tx_pending;
    bool connect_req_tx_pending;
    bool scan_filter_enabled;
    ble_gap_scan_filter_t scan_filter;
    bool connect_target_valid;
    ble_gap_addr_t connect_target;
    bool scan_rsp_target_valid;
    ble_gap_addr_t scan_rsp_target;
    bool scan_rsp_target_connectable;
    bool scan_rsp_connect_candidate_valid;
    ble_gap_addr_t scan_rsp_connect_candidate;
    ble_scan_radio_phase_t scan_radio_phase;
    ble_central_ctrl_proc_context_t central_ctrl_proc;
    ble_auto_ctrl_stage_t auto_ctrl_stage;
} ble_central_ctrl_runtime_t;

typedef struct
{
    ble_ll_data_raw_pdu_t conn_rx_pdu;
    ble_ll_data_raw_pdu_t pending_conn_ctrl_rsp_pdu;
    ble_ll_data_raw_pdu_t pending_conn_ctrl_pdu;
    ble_ll_data_raw_pdu_t conn_tx_pdu;
    ble_ll_data_raw_pdu_t last_conn_tx_pdu;
    ble_conn_l2cap_tx_queue_t l2cap_tx_queue;
    bool tx_unacked;
    bool has_pending_conn_ctrl_rsp_pdu;
    bool has_pending_conn_ctrl_pdu;
    bool conn_rx_process_pending;
    ble_conn_pending_source_t selected_conn_tx_source;
    ble_conn_radio_phase_t conn_radio_phase;
    ble_conn_bcmatch_state_t conn_bcmatch;
    uint32_t conn_next_event_tick_us;
} ble_conn_ctrl_runtime_t;

typedef struct
{
    ble_local_addr_runtime_t local_addr;
    ble_peripheral_ctrl_runtime_t peripheral;
    ble_central_ctrl_runtime_t central;
    ble_conn_ctrl_runtime_t conn;
} ble_ctrl_runtime_t;

#endif
