/* SPDX-License-Identifier: MIT */

#ifndef BLE_GAP_H__
#define BLE_GAP_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_uuid.h"

#define BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE 0x01U
#define BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE 0x02U
#define BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED 0x04U
#define BLE_GAP_ADV_FLAG_LE_BR_EDR_CONTROLLER 0x08U
#define BLE_GAP_ADV_FLAG_LE_BR_EDR_HOST 0x10U

#define BLE_AD_TYPE_FLAGS 0x01U
#define BLE_AD_TYPE_INCOMPLETE_UUID16_LIST 0x02U
#define BLE_AD_TYPE_COMPLETE_UUID16_LIST 0x03U
#define BLE_AD_TYPE_INCOMPLETE_UUID128_LIST 0x06U
#define BLE_AD_TYPE_COMPLETE_UUID128_LIST 0x07U
#define BLE_AD_TYPE_SHORT_LOCAL_NAME 0x08U
#define BLE_AD_TYPE_COMPLETE_LOCAL_NAME 0x09U
#define BLE_AD_TYPE_TX_POWER_LEVEL 0x0A
#define BLE_AD_TYPE_SERVICE_DATA_UUID16 0x16U
#define BLE_AD_TYPE_SERVICE_DATA_UUID128 0x21U
#define BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA 0xFFU

#define BLE_GAP_PHY_1MBPS 0x01U
#define BLE_GAP_PHY_2MBPS 0x02U

#define BLE_ADV_INTERVAL_MS_DEFAULT 100U
#define BLE_SCAN_INTERVAL_MS_DEFAULT 100U
#define BLE_SCAN_WINDOW_MS_DEFAULT 50U
#define BLE_GAP_ADV_DATA_MAX_LEN 31U
#define BLE_GAP_ADV_FIELD_DATA_MAX_LEN 27U
#define BLE_GAP_ADV_SERVICE_UUID_LIST_MAX_COUNT 4U
#define BLE_GAP_ADV_SERVICE_UUID_PER_LIST_MAX_COUNT 8U
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
    BLE_GAP_ADV_SERVICE_UUID_LIST_INCOMPLETE_16 = 0,
    BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_16,
    BLE_GAP_ADV_SERVICE_UUID_LIST_INCOMPLETE_128,
    BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_128,
} ble_gap_adv_service_uuid_list_type_t;

typedef enum
{
    BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED = 0x00U,
    BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED = 0x02U,
    BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED = 0x06U,
} ble_gap_adv_type_t;

typedef enum
{
    BLE_GAP_ROLE_NONE = 0,
    BLE_GAP_ROLE_PERIPHERAL,
    BLE_GAP_ROLE_CENTRAL
} ble_gap_role_t;

typedef struct
{
    uint8_t addr[6];
    bool is_random;
} ble_gap_addr_t;

typedef struct
{
    ble_uuid_t uuid;
    const uint8_t *p_data;
    uint8_t data_len;
} ble_gap_adv_service_data_t;

typedef struct
{
    uint16_t company_id;
    const uint8_t *p_data;
    uint8_t data_len;
} ble_gap_adv_manufacturer_data_t;

typedef struct
{
    ble_gap_adv_name_type_t name_type;
    uint8_t short_name_length;
} ble_gap_adv_name_config_t;

typedef struct
{
    ble_gap_adv_service_uuid_list_type_t type;
    const ble_uuid_t *p_uuids;
    uint8_t uuid_count;
} ble_gap_adv_service_uuid_list_config_t;

typedef struct
{
    const ble_gap_adv_name_config_t *p_name;
    const int8_t *p_tx_power;
    const ble_gap_adv_service_uuid_list_config_t *p_service_uuid_lists;
    uint8_t service_uuid_list_count;
    const ble_gap_adv_service_data_t *p_service_data;
    const ble_gap_adv_manufacturer_data_t *p_manufacturer_data;
} ble_gap_adv_data_config_t;

typedef struct
{
    uint8_t flags;
    uint16_t interval_ms;
    ble_gap_adv_type_t adv_type;
    ble_gap_adv_data_config_t adv_data;
    ble_gap_adv_data_config_t scan_response_data;
} ble_gap_adv_config_t;

typedef struct
{
    uint16_t min_conn_interval_1p25ms;
    uint16_t max_conn_interval_1p25ms;
    uint16_t slave_latency;
    uint16_t supervision_timeout_10ms;
} ble_gap_conn_params_t;

typedef enum
{
    BLE_GAP_CTRL_PROC_NONE = 0,
    BLE_GAP_CTRL_PROC_FEATURE_EXCHANGE,
    BLE_GAP_CTRL_PROC_DATA_LENGTH_UPDATE,
    BLE_GAP_CTRL_PROC_PHY_UPDATE,
    BLE_GAP_CTRL_PROC_CONN_UPDATE,
} ble_gap_ctrl_procedure_t;

typedef struct
{
    uint16_t interval_ms;
    uint16_t window_ms;
    bool active;
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
    bool scan_response;
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
    BLE_GAP_EVT_FEATURE_EXCHANGED,
    BLE_GAP_EVT_DATA_LENGTH_UPDATED,
    BLE_GAP_EVT_CONTROL_PROCEDURE_UNSUPPORTED,
} ble_gap_evt_type_t;

typedef struct
{
    uint16_t conn_interval_ms;
    uint16_t slave_latency;
    uint16_t supervision_timeout_ms;
    uint8_t tx_phy;
    uint8_t rx_phy;
    uint16_t max_tx_octets;
    uint16_t max_rx_octets;
    uint16_t max_tx_time_us;
    uint16_t max_rx_time_us;
    ble_gap_role_t role;
    ble_gap_addr_t peer_addr;
    uint8_t features[8];
    ble_gap_ctrl_procedure_t procedure;
    uint8_t unsupported_opcode;
} ble_gap_evt_params_t;

typedef struct
{
    ble_gap_evt_type_t evt_type;
    ble_gap_evt_params_t params;
} ble_gap_evt_t;

typedef void (*ble_gap_evt_handler_t)(const ble_gap_evt_t *p_evt);
typedef void (*ble_gap_scan_report_handler_t)(const ble_gap_scan_report_t *p_report);

/**
 * @brief Start advertising with the previously stored advertising configuration.
 */
void ble_gap_start_advertising(void);

/**
 * @brief Register the stack-wide GAP event callback.
 *
 * @param[in] handler Application callback for GAP events.
 */
void ble_gap_register_evt_handler(ble_gap_evt_handler_t handler);

/**
 * @brief Register the callback used for scan reports while scanning.
 *
 * @param[in] handler Application callback for scan reports.
 */
void ble_gap_register_scan_report_handler(ble_gap_scan_report_handler_t handler);

/**
 * @brief Store advertising parameters used by ble_gap_start_advertising().
 *
 * Advertising descriptors are copied during initialization. Service data and
 * manufacturer-specific data payload pointers are retained, allowing the
 * application to update those buffers between advertising events. The payload
 * buffers must remain valid while the advertising configuration is active.
 *
 * @param[in] p_config Advertising configuration, or NULL for defaults.
 *
 * @retval true The advertising configuration was accepted.
 * @retval false The configuration is invalid.
 */
bool ble_gap_adv_init(const ble_gap_adv_config_t *p_config);

/**
 * @brief Store scanning parameters used by ble_gap_start_scanning().
 *
 * @param[in] p_config Scan configuration, or NULL for defaults.
 */
void ble_gap_scan_init(const ble_scan_config_t *p_config);

/**
 * @brief Install the auto-connect and report filter used while scanning.
 *
 * @param[in] p_filter Filter configuration to apply.
 *
 * @retval true The filter was accepted.
 * @retval false The filter could not be applied.
 */
bool ble_gap_set_scan_filter(const ble_gap_scan_filter_t *p_filter);

/**
 * @brief Clear any previously configured scan filter.
 */
void ble_gap_clear_scan_filter(void);

/**
 * @brief Start scanning using the configuration from ble_gap_scan_init().
 */
void ble_gap_start_scanning(void);

/**
 * @brief Stop an active scan procedure.
 */
void ble_gap_stop_scanning(void);

/**
 * @brief Set the local GAP device name used in advertising and the GAP service.
 *
 * @param[in] p_name Null-terminated UTF-8 device name string.
 */
void ble_gap_set_device_name(const char *p_name);

/**
 * @brief Store preferred connection parameters for future procedures.
 *
 * @param[in] p_params Preferred connection parameters.
 */
void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params);

/**
 * @brief Start connection establishment to the selected peer in the central role.
 *
 * @param[in] p_peer_addr Peer address to connect to.
 *
 * @retval true The connect procedure was started.
 * @retval false The connect procedure could not be started.
 */
bool ble_gap_connect(const ble_gap_addr_t *p_peer_addr);

/**
 * @brief Send a peripheral-side L2CAP Connection Parameter Update Request.
 *
 * @retval true The request was queued.
 * @retval false The request could not be queued.
 */
bool ble_gap_request_conn_params_update(void);

/**
 * @brief Initiate a central-side LL connection update procedure.
 *
 * @param[in] p_params Connection parameters to indicate.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gap_initiate_conn_update(const ble_gap_conn_params_t *p_params);

/**
 * @brief Return whether a BLE link is currently established.
 *
 * @retval true A connection is active.
 * @retval false No connection is active.
 */
bool ble_gap_is_connected(void);

/**
 * @brief Terminate the current BLE connection if one is active.
 */
void ble_gap_disconnect(void);

#endif /* BLE_GAP_H__ */
