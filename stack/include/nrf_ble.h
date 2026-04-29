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

#include "ble_uuid.h"
#include "ble_gatt_server.h"
#include "ble_gatt_client.h"

#define BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE 0x01U
#define BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE 0x02U
#define BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED 0x04U
#define BLE_GAP_ADV_FLAG_LE_BR_EDR_CONTROLLER 0x08U
#define BLE_GAP_ADV_FLAG_LE_BR_EDR_HOST 0x10U
#define BLE_GAP_PHY_1MBPS 0x01U
#define BLE_GAP_PHY_2MBPS 0x02U

#define BLE_ADV_INTERVAL_MS_DEFAULT 100U
#define BLE_SCAN_INTERVAL_MS_DEFAULT 100U
#define BLE_SCAN_WINDOW_MS_DEFAULT 50U
#define BLE_GAP_ADV_DATA_MAX_LEN 31U
#define BLE_GAP_SCAN_FILTER_NAME_MAX_LEN BLE_GAP_ADV_DATA_MAX_LEN
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

typedef enum
{
    BLE_GAP_ADV_NAME_FULL = 0,
    BLE_GAP_ADV_NAME_SHORT
} ble_gap_adv_name_type_t;

typedef enum
{
    BLE_GAP_ROLE_NONE = 0,
    BLE_GAP_ROLE_PERIPHERAL,
    BLE_GAP_ROLE_CENTRAL
} ble_gap_role_t;

typedef struct
{
    uint8_t addr[6];
    bool addr_is_random;
} ble_gap_addr_t;

typedef struct
{
    uint8_t flags;
    int8_t tx_power;
    uint16_t interval_ms;
    const ble_uuid_t *p_included_service_uuid;
    ble_gap_adv_name_type_t name_type;
    uint8_t short_name_length;
} ble_adv_config_t;

typedef struct
{
    uint16_t min_conn_interval_1p25ms;
    uint16_t max_conn_interval_1p25ms;
    uint16_t slave_latency;
    uint16_t supervision_timeout_10ms;
} ble_gap_conn_params_t;

typedef struct
{
    uint16_t interval_ms;
    uint16_t window_ms;
} ble_scan_config_t;

typedef struct
{
    bool match_addr;
    ble_gap_addr_t addr;
    bool match_name;
    char name[BLE_GAP_SCAN_FILTER_NAME_MAX_LEN + 1U];
    bool match_service_uuid16;
    uint16_t service_uuid16;
    bool match_service_uuid128;
    uint8_t service_uuid128[BLE_UUID128_LEN];
} ble_gap_scan_filter_t;

typedef struct
{
    ble_gap_addr_t addr;
    uint8_t adv_type;
    bool connectable;
    bool scannable;
    int8_t rssi;
    uint8_t data_len;
    uint8_t data[BLE_GAP_ADV_DATA_MAX_LEN];
} ble_gap_scan_report_t;

typedef enum
{
    BLE_GAP_EVT_CONNECTED = 0,
    BLE_GAP_EVT_DISCONNECTED,
    BLE_GAP_EVT_SUPERVISION_TIMEOUT,
    BLE_GAP_EVT_CONN_UPDATE_IND,
    BLE_GAP_EVT_PHY_UPDATE_IND,
    BLE_GAP_EVT_TERMINATE_IND,
} ble_gap_evt_type_t;

typedef struct
{
    uint16_t conn_interval_ms;
    uint16_t slave_latency;
    uint16_t supervision_timeout_ms;
    uint8_t tx_phy;
    uint8_t rx_phy;
    ble_gap_role_t role;
    ble_gap_addr_t peer_addr;
} ble_gap_evt_params_t;

typedef struct
{
    ble_gap_evt_type_t evt_type;
    ble_gap_evt_params_t params;
} ble_gap_evt_t;

typedef void (*ble_gap_evt_handler_t)(const ble_gap_evt_t *p_evt);
typedef void (*ble_gap_scan_report_handler_t)(const ble_gap_scan_report_t *p_report);

void ble_start_advertising(void);
void ble_stack_init(ble_gap_role_t role);
void ble_gap_register_evt_handler(ble_gap_evt_handler_t handler);
void ble_register_scan_report_handler(ble_gap_scan_report_handler_t handler);
void ble_adv_init(const ble_adv_config_t *p_config);
void ble_scan_init(const ble_scan_config_t *p_config);
bool ble_gap_set_scan_filter(const ble_gap_scan_filter_t *p_filter);
void ble_gap_clear_scan_filter(void);
void ble_start_scanning(void);
void ble_stop_scanning(void);
void ble_gap_set_device_name(const char *p_name);
void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params);
bool ble_gap_connect(const ble_gap_addr_t *p_peer_addr);
bool ble_gap_update_conn_params(void);
void ble_uuid_set_vendor_base(const uint8_t uuid128[BLE_UUID128_LEN]);
bool ble_is_connected(void);
bool ble_notify_characteristic(const ble_gatt_characteristic_t *p_characteristic);
bool ble_indicate_characteristic(const ble_gatt_characteristic_t *p_characteristic);
void ble_disconnect(void);

#endif // NRF_BLE_H__
