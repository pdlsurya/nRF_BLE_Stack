/* SPDX-License-Identifier: MIT */

#ifndef BLE_L2CAP_INTERNAL_H__
#define BLE_L2CAP_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gap.h"

#define BLE_L2CAP_CID_ATT 0x0004U
#define BLE_L2CAP_CID_SIGNALING 0x0005U
#define BLE_L2CAP_HDR_LEN 4U
#define BLE_L2CAP_SIG_HDR_LEN 4U
#define BLE_L2CAP_SIG_CONN_PARAM_UPDATE_REQ 0x12U
#define BLE_L2CAP_SIG_CONN_PARAM_UPDATE_RSP 0x13U
#define BLE_L2CAP_SIG_CONN_PARAM_ACCEPTED 0x0000U
#define BLE_L2CAP_SIG_CONN_PARAM_REJECTED 0x0001U

bool ble_l2cap_queue_conn_param_update_req(const ble_gap_conn_params_t *p_params);
void ble_l2cap_process_conn_data_pdu(const uint8_t *p_payload, uint8_t pdu_len);

#endif
