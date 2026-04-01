/**
 * @file nrf_ble.h
 * @author Surya Poudel
 * @brief Public API for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef NRF_BLE_H__
#define NRF_BLE_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gatt_server.h"

#define BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE  0x01U
#define BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE  0x02U
#define BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED  0x04U
#define BLE_GAP_ADV_FLAG_LE_BR_EDR_CONTROLLER  0x08U
#define BLE_GAP_ADV_FLAG_LE_BR_EDR_HOST        0x10U

#define BLE_ADV_INTERVAL_MS_DEFAULT 100U

typedef struct
{
    uint16_t uuid;
    uint8_t data;
} ble_service_data_t;

typedef struct
{
    uint8_t flags;
    int8_t tx_power;
    uint16_t interval_ms;
    uint16_t included_service_uuid;
    const ble_service_data_t *p_service_data;
} ble_adv_config_t;

typedef enum
{
    BLE_GAP_EVT_CONNECTED = 0,
    BLE_GAP_EVT_DISCONNECTED,
    BLE_GAP_EVT_SUPERVISION_TIMEOUT,
    BLE_GAP_EVT_CONN_UPDATE_IND,
    BLE_GAP_EVT_TERMINATE_IND,
    BLE_GATT_EVT_MTU_EXCHANGE
} ble_evt_type_t;

typedef struct
{
    ble_evt_type_t evt_type;
    uint16_t conn_interval_ms;
    uint16_t supervision_timeout_ms;
    uint16_t requested_mtu;
    uint16_t response_mtu;
    uint16_t effective_mtu;
} ble_evt_t;

typedef void (*ble_evt_handler_t)(const ble_evt_t *p_evt);

void ble_start_advertising(void);
void ble_stack_init(void);
void ble_register_evt_handler(ble_evt_handler_t handler);
void ble_adv_init(const ble_adv_config_t *p_config);
void ble_gap_set_device_name(const char *p_name);
bool ble_gatt_server_init(ble_gatt_service_t *p_services,
                          uint8_t service_count);
bool ble_is_connected(void);
bool ble_notify_characteristic(const ble_gatt_characteristic_t *p_characteristic);

#endif // NRF_BLE_H__
