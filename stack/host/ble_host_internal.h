/* SPDX-License-Identifier: MIT */

#ifndef BLE_HOST_INTERNAL_H__
#define BLE_HOST_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_gap.h"

#define BLE_HOST_LOCAL_DEVICE_NAME_MAX_LEN 20U

typedef struct
{
    char local_device_name[BLE_HOST_LOCAL_DEVICE_NAME_MAX_LEN + 1U];
    int8_t tx_power;
    ble_gap_conn_params_t preferred_conn_params;
    bool preferred_conn_params_valid;
    uint8_t local_vendor_uuid_base[BLE_UUID128_LEN];
    bool local_vendor_uuid_base_set;
    uint8_t next_l2cap_sig_identifier;
} ble_host_common_t;

typedef struct
{
    ble_gap_adv_service_uuid_list_type_t type;
    uint8_t uuid_count;
    ble_uuid_t uuids[BLE_GAP_ADV_SERVICE_UUID_PER_LIST_MAX_COUNT];
} ble_host_adv_service_uuid_list_t;

typedef struct
{
    bool name_present;
    ble_gap_adv_name_config_t name;
    bool tx_power_present;
    int8_t tx_power;
    uint8_t service_uuid_list_count;
    ble_host_adv_service_uuid_list_t service_uuid_lists[BLE_GAP_ADV_SERVICE_UUID_LIST_MAX_COUNT];
    bool service_data_present;
    ble_gap_adv_service_data_t service_data;
    bool manufacturer_data_present;
    ble_gap_adv_manufacturer_data_t manufacturer_data;
} ble_host_adv_data_t;

typedef struct
{
    uint8_t flags;
    uint16_t adv_interval_ms;
    ble_gap_adv_type_t adv_type;
    ble_host_adv_data_t adv_data;
    ble_host_adv_data_t scan_response_data;
} ble_host_peripheral_t;

typedef struct
{
    ble_scan_config_t scan_config;
} ble_host_central_t;

typedef struct
{
    ble_gap_role_t configured_role;
    ble_host_common_t common;
    ble_host_peripheral_t peripheral;
    ble_host_central_t central;
} ble_host_t;

extern ble_host_t m_host;

static inline bool ble_host_role_is_configured(ble_gap_role_t role)
{
    return m_host.configured_role == role;
}

#endif /* BLE_HOST_INTERNAL_H__ */
