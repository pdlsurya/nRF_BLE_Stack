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
#define BLE_GAP_PHY_1MBPS                      0x01U
#define BLE_GAP_PHY_2MBPS                      0x02U

#define BLE_ADV_INTERVAL_MS_DEFAULT 100U
#define MS_TO_1P25MS_UNITS(ms) \
    ((uint16_t)((((uint32_t)(ms)) * 1000U) / 1250U))
#define MS_TO_10MS_UNITS(ms) \
    ((uint16_t)(((uint32_t)(ms)) / 10U))
#define UNITS_1P25MS_TO_US(units) \
    ((uint32_t)(units) * 1250U)
#define UNITS_1P25MS_TO_MS(units) \
    ((uint16_t)(((uint32_t)(units) * 125U) / 100U))
#define UNITS_10MS_TO_MS(units) \
    ((uint16_t)((uint32_t)(units) * 10U))

typedef struct
{
    uint8_t flags;
    int8_t tx_power;
    uint16_t interval_ms;
    const ble_uuid_t *p_included_service_uuid;
} ble_adv_config_t;

typedef struct
{
    uint16_t min_conn_interval_1p25ms;
    uint16_t max_conn_interval_1p25ms;
    uint16_t slave_latency;
    uint16_t supervision_timeout_10ms;
} ble_gap_conn_params_t;

typedef enum
{
    BLE_GAP_EVT_CONNECTED = 0,
    BLE_GAP_EVT_DISCONNECTED,
    BLE_GAP_EVT_SUPERVISION_TIMEOUT,
    BLE_GAP_EVT_CONN_UPDATE_IND,
    BLE_GAP_EVT_PHY_UPDATE_IND,
    BLE_GAP_EVT_TERMINATE_IND,
    BLE_GATT_EVT_MTU_EXCHANGE
} ble_evt_type_t;

typedef struct
{
    uint16_t conn_interval_ms;
    uint16_t slave_latency;
    uint16_t supervision_timeout_ms;
    uint8_t tx_phy;
    uint8_t rx_phy;
} ble_gap_evt_params_t;

typedef struct
{
    uint16_t requested_mtu;
    uint16_t response_mtu;
    uint16_t effective_mtu;
} ble_gatt_evt_params_t;

typedef struct
{
    ble_evt_type_t evt_type;
    union
    {
        ble_gap_evt_params_t gap;
        ble_gatt_evt_params_t gatt;
    } params;
} ble_evt_t;

typedef void (*ble_evt_handler_t)(const ble_evt_t *p_evt);

void ble_start_advertising(void);
void ble_stack_init(void);
void ble_register_evt_handler(ble_evt_handler_t handler);
void ble_adv_init(const ble_adv_config_t *p_config);
void ble_gap_set_device_name(const char *p_name);
void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params);
bool ble_gap_update_conn_params(void);
void ble_uuid_set_vendor_base(const uint8_t uuid128[BLE_UUID128_LEN]);
bool ble_gatt_server_init(ble_gatt_service_t *p_services,
                          uint8_t service_count);
bool ble_is_connected(void);
bool ble_notify_characteristic(const ble_gatt_characteristic_t *p_characteristic);

#endif // NRF_BLE_H__
