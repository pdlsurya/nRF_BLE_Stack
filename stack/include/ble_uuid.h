/* SPDX-License-Identifier: MIT */

#ifndef BLE_UUID_H__
#define BLE_UUID_H__

#include <stdint.h>

#define BLE_UUID16_LEN 2U
#define BLE_UUID128_LEN 16U

typedef enum
{
    BLE_UUID_TYPE_NONE = 0,
    BLE_UUID_TYPE_SIG_16,
    BLE_UUID_TYPE_VENDOR_16,
    BLE_UUID_TYPE_RAW_128
} ble_uuid_type_t;

typedef union
{
    uint16_t uuid16;
    uint8_t uuid128[BLE_UUID128_LEN];
} ble_uuid_value_t;

typedef struct
{
    ble_uuid_type_t type;
    ble_uuid_value_t value;
} ble_uuid_t;

#define BLE_UUID_NONE_INIT         { .type = BLE_UUID_TYPE_NONE, .value.uuid16 = 0U }
#define BLE_UUID_SIG16_INIT(v)     { .type = BLE_UUID_TYPE_SIG_16, .value.uuid16 = (v) }
#define BLE_UUID_VENDOR16_INIT(v)  { .type = BLE_UUID_TYPE_VENDOR_16, .value.uuid16 = (v) }
#define BLE_UUID_RAW128_INIT(...)  { .type = BLE_UUID_TYPE_RAW_128, .value.uuid128 = { __VA_ARGS__ } }

/**
 * @brief Register the base 128-bit UUID used to expand vendor 16-bit UUIDs.
 *
 * @param[in] uuid128 Vendor base UUID in little-endian byte order.
 */
void ble_uuid_set_vendor_base(const uint8_t uuid128[BLE_UUID128_LEN]);

#endif /* BLE_UUID_H__ */
