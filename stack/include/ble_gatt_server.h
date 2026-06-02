/* SPDX-License-Identifier: MIT */

#ifndef BLE_GATT_SERVER_H__
#define BLE_GATT_SERVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_att.h"
#include "ble_uuid.h"

#define BLE_GATT_MAX_SERVICES 4U
#define BLE_GATT_MAX_CHARACTERISTICS 8U
#define BLE_GATT_CHAR_PROP_READ 0x02U
#define BLE_GATT_CHAR_PROP_WRITE_NO_RESP 0x04U
#define BLE_GATT_CHAR_PROP_WRITE 0x08U
#define BLE_GATT_CHAR_PROP_NOTIFY 0x10U
#define BLE_GATT_CHAR_PROP_INDICATE 0x20U

typedef struct ble_gatt_characteristic_s ble_gatt_characteristic_t;

typedef enum
{
    BLE_GATT_CHAR_EVT_WRITE = 0,
    BLE_GATT_CHAR_EVT_NOTIFY_ENABLED,
    BLE_GATT_CHAR_EVT_NOTIFY_DISABLED,
    BLE_GATT_CHAR_EVT_INDICATE_ENABLED,
    BLE_GATT_CHAR_EVT_INDICATE_DISABLED
} ble_gatt_char_evt_type_t;

typedef struct
{
    ble_gatt_char_evt_type_t evt_type;
    ble_gatt_characteristic_t *p_characteristic;
    bool notifications_enabled;
    bool indications_enabled;
} ble_gatt_char_evt_t;

typedef void (*ble_gatt_char_evt_handler_t)(const ble_gatt_char_evt_t *p_evt);

struct ble_gatt_characteristic_s
{
    ble_uuid_t uuid;
    uint8_t properties;
    uint8_t *p_value;
    uint16_t value_len;
    uint16_t max_len;
    ble_gatt_char_evt_handler_t evt_handler;
    /* Filled by ble_gatt_server_init() and used for notify/indicate CCCD tracking. */
    uint16_t value_handle;
    uint16_t cccd_handle;
};

typedef struct
{
    ble_uuid_t uuid;
    ble_gatt_characteristic_t *p_characteristics;
    uint8_t characteristic_count;
    uint16_t service_handle;
} ble_gatt_service_t;

typedef enum
{
    BLE_GATT_SERVER_EVT_MTU_EXCHANGE = 0,
} ble_gatt_server_evt_type_t;

typedef struct
{
    uint16_t requested_mtu;
    uint16_t response_mtu;
    uint16_t effective_mtu;
} ble_gatt_server_evt_params_t;

typedef struct
{
    ble_gatt_server_evt_type_t evt_type;
    ble_gatt_server_evt_params_t params;
} ble_gatt_server_evt_t;

typedef void (*ble_gatt_server_evt_handler_t)(const ble_gatt_server_evt_t *p_evt);

/**
 * @brief Register the stack-wide GATT server event callback.
 *
 * @param[in] handler Application callback for server events.
 */
void ble_gatt_server_register_evt_handler(ble_gatt_server_evt_handler_t handler);

/**
 * @brief Build the local ATT database from the supplied services and characteristics.
 *
 * @param[in,out] p_services Array of service definitions to register.
 * @param[in] service_count Number of services in @p p_services.
 *
 * @retval true The database was built successfully.
 * @retval false The database could not be built.
 */
bool ble_gatt_server_init(ble_gatt_service_t *p_services, uint8_t service_count);

/**
 * @brief Queue a notification for the selected characteristic value.
 *
 * @param[in] p_characteristic Characteristic whose current value should be notified.
 *
 * @retval true The notification was queued.
 * @retval false The notification could not be queued.
 */
bool ble_gatt_server_notify_characteristic(const ble_gatt_characteristic_t *p_characteristic);

/**
 * @brief Queue an indication for the selected characteristic value.
 *
 * @param[in] p_characteristic Characteristic whose current value should be indicated.
 *
 * @retval true The indication was queued.
 * @retval false The indication could not be queued.
 */
bool ble_gatt_server_indicate_characteristic(const ble_gatt_characteristic_t *p_characteristic);

#endif /* BLE_GATT_SERVER_H__ */
