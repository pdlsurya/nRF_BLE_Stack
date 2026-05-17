/**
 * @file ble_gatt_server_internal.h
 * @author Surya Poudel
 * @brief Internal GATT server entry points for nRF BLE stack
 * @version 0.1
 * @date 2026-05-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_GATT_SERVER_INTERNAL_H__
#define BLE_GATT_SERVER_INTERNAL_H__

#include <stdint.h>

void ble_gatt_server_reset_connection_state(void);
uint16_t ble_gatt_server_process_att_pdu(const uint8_t *p_att,
                                         uint16_t att_len,
                                         uint8_t *p_rsp,
                                         uint16_t rsp_max_len);

#endif /* BLE_GATT_SERVER_INTERNAL_H__ */
