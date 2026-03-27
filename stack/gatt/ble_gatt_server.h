/**
 * @file ble_gatt_server.h
 * @author Surya Poudel
 * @brief ATT and GATT server interface for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_GATT_SERVER_H__
#define BLE_GATT_SERVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf_ble.h"

#define BLE_ATT_GATT_MAX_MTU 23U

bool ble_gatt_server_init(ble_gatt_service_t *p_services,
                          uint8_t service_count);
void ble_gatt_server_reset_connection_state(void);
bool ble_gatt_server_notifications_enabled(const ble_gatt_characteristic_t *p_characteristic);
uint16_t ble_gatt_server_build_notification(uint16_t value_handle, uint8_t *p_att, uint16_t max_len);

uint16_t ble_gatt_server_process_request(const uint8_t *p_att,
                                         uint16_t att_len,
                                         uint8_t *p_rsp,
                                         uint16_t rsp_max_len,
                                         bool *p_has_response);

#endif
