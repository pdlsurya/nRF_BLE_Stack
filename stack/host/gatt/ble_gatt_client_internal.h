/* SPDX-License-Identifier: MIT */

#ifndef BLE_GATT_CLIENT_INTERNAL_H__
#define BLE_GATT_CLIENT_INTERNAL_H__

#include <stdint.h>

void ble_gatt_client_reset_connection_state(void);
uint16_t ble_gatt_client_process_att_pdu(const uint8_t *p_att,
                                         uint16_t att_len,
                                         uint8_t *p_rsp,
                                         uint16_t rsp_max_len);

#endif /* BLE_GATT_CLIENT_INTERNAL_H__ */
