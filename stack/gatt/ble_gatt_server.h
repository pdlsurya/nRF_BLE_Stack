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

#define BLE_GATT_MAX_VALUE_LEN 244U
#define BLE_GATT_MAX_SERVICES 4U
#define BLE_GATT_MAX_CHARACTERISTICS 8U
#define BLE_GATT_CHAR_PROP_READ 0x02U
#define BLE_GATT_CHAR_PROP_WRITE_NO_RESP 0x04U
#define BLE_GATT_CHAR_PROP_WRITE 0x08U
#define BLE_GATT_CHAR_PROP_NOTIFY 0x10U

#define BLE_ATT_GATT_MAX_MTU 247U
#define BLE_UUID16_LEN 2U
#define BLE_UUID128_LEN 16U

typedef enum
{
    BLE_UUID_TYPE_NONE = 0,
    BLE_UUID_TYPE_SIG_16,
    BLE_UUID_TYPE_VENDOR_16
} ble_uuid_type_t;

typedef struct
{
    ble_uuid_type_t type;
    uint16_t value;
} ble_uuid_t;

#define BLE_UUID_NONE_INIT        { .type = BLE_UUID_TYPE_NONE, .value = 0U }
#define BLE_UUID_SIG16_INIT(v)    { .type = BLE_UUID_TYPE_SIG_16, .value = (v) }
#define BLE_UUID_VENDOR16_INIT(v) { .type = BLE_UUID_TYPE_VENDOR_16, .value = (v) }

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
    bool notifications_enabled;
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
    /* Filled by ble_gatt_server_init() and used for notifications/CCCD tracking. */
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

bool ble_gatt_server_init(ble_gatt_service_t *p_services,
                          uint8_t service_count);
void ble_gatt_server_reset_connection_state(void);
uint16_t ble_gatt_server_build_notification(uint16_t value_handle, uint8_t *p_att, uint16_t max_len);

uint16_t ble_gatt_server_process_request(const uint8_t *p_att,
                                         uint16_t att_len,
                                         uint8_t *p_rsp,
                                         uint16_t rsp_max_len);

#endif
