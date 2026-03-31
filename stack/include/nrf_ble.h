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

#define BLE_GATT_MAX_VALUE_LEN 20U
#define BLE_GATT_MAX_SERVICES 4U
#define BLE_GATT_MAX_CHARACTERISTICS 8U
#define BLE_GATT_CHAR_PROP_READ 0x02U
#define BLE_GATT_CHAR_PROP_WRITE_NO_RESP 0x04U
#define BLE_GATT_CHAR_PROP_WRITE 0x08U
#define BLE_GATT_CHAR_PROP_NOTIFY 0x10U

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

typedef struct ble_gatt_characteristic_s ble_gatt_characteristic_t;

typedef enum
{
    BLE_GATT_CHAR_EVT_WRITE = 0,
    BLE_GATT_CHAR_EVT_NOTIFY_ENABLED,
    BLE_GATT_CHAR_EVT_NOTIFY_DISABLED
} ble_gatt_char_evt_type_t;

typedef struct
{
    ble_gatt_char_evt_type_t evt_type;
    ble_gatt_characteristic_t *p_characteristic;
    const uint8_t *p_data;
    uint16_t len;
    bool notifications_enabled;
} ble_gatt_char_evt_t;

typedef void (*ble_gatt_char_evt_handler_t)(const ble_gatt_char_evt_t *p_evt);

struct ble_gatt_characteristic_s
{
    uint16_t uuid;
    uint8_t properties;
    uint8_t *p_value;
    uint16_t *p_value_len;
    uint16_t max_len;
    ble_gatt_char_evt_handler_t evt_handler;
    /* Filled by ble_gatt_server_init() and used for notifications/CCCD tracking. */
    uint16_t value_handle;
    uint16_t cccd_handle;
};

typedef struct
{
    uint16_t uuid;
    ble_gatt_characteristic_t *p_characteristics;
    uint8_t characteristic_count;
    uint16_t service_handle;
} ble_gatt_service_t;

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
