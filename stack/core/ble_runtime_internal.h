/**
 * @file ble_runtime_internal.h
 * @author Surya Poudel
 * @brief Internal runtime state and shared cross-layer definitions for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_RUNTIME_INTERNAL_H__
#define BLE_RUNTIME_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>

#include "app_timer.h"
#include "ble_ll_types.h"
#include "nrf_ble.h"
#include "radio_driver.h"

#define BLE_GAP_DEVICE_NAME_MAX_LEN 20U
#define BLE_IDENTITY_SALT 0x434D535456324C39ULL
#define BLE_L2CAP_CID_ATT 0x0004U
#define BLE_L2CAP_CID_SIGNALING 0x0005U
#define BLE_L2CAP_HDR_LEN 4U
#define BLE_L2CAP_SIG_HDR_LEN 4U
#define BLE_L2CAP_SIG_CONN_PARAM_UPDATE_REQ 0x12U
#define BLE_L2CAP_SIG_CONN_PARAM_UPDATE_RSP 0x13U
#define BLE_L2CAP_SIG_CONN_PARAM_ACCEPTED 0x0000U
#define BLE_L2CAP_SIG_CONN_PARAM_REJECTED 0x0001U
#define BLE_ADV_RX_WINDOW_US 1500U
#define BLE_CONN_EVENT_GUARD_US 5000U
#define BLE_CONN_TIMER_PRESCALER 4U
#define BLE_CONN_TIMER_IRQ_PRIORITY 0U
#define BLE_EVT_IRQ_PRIORITY 6U
#define BLE_EVT_QUEUE_SIZE 16U
#define BLE_CONN_PARAM_UPDATE_DELAY_MS 6000U
#define BLE_INITIATOR_WIN_OFFSET_UNITS 6U
#define BLE_INITIATOR_WIN_SIZE_UNITS 2U
#define BLE_INITIATOR_HOP_INCREMENT 5U
#define BLE_LL_DATA_LEN_DEFAULT_OCTETS 27U
#define BLE_LL_DATA_LEN_MAX_OCTETS 251U
#define BLE_LL_DATA_LEN_DEFAULT_TIME 328U
#define BLE_LL_DATA_LEN_MAX_TIME 2120U

#define BLE_LL_CTRL_CONN_UPDATE_IND 0x00U
#define BLE_LL_CTRL_CHANNEL_MAP_IND 0x01U
#define BLE_LL_CTRL_TERMINATE_IND 0x02U
#define BLE_LL_CTRL_UNKNOWN_RSP 0x07U
#define BLE_LL_CTRL_FEATURE_REQ 0x08U
#define BLE_LL_CTRL_FEATURE_RSP 0x09U
#define BLE_LL_CTRL_VERSION_IND 0x0CU
#define BLE_LL_CTRL_SLV_FEATURE_REQ 0x0EU
#define BLE_LL_CTRL_LENGTH_REQ 0x14U
#define BLE_LL_CTRL_LENGTH_RSP 0x15U
#define BLE_LL_CTRL_PHY_REQ 0x16U
#define BLE_LL_CTRL_PHY_RSP 0x17U
#define BLE_LL_CTRL_PHY_UPDATE_IND 0x18U
#define BLE_LL_VERSION_4_2 0x08U
#define BLE_LL_COMPANY_ID_NORDIC 0x0059U
#define BLE_LL_SUBVERSION 0x0000U
#define BLE_LL_FEATURE_DATA_LENGTH_EXTENSION 0x20U
#define BLE_LL_FEATURE_2M_PHY 0x01U
#define BLE_LL_PHY_1M 0x01U
#define BLE_LL_PHY_2M 0x02U

static inline uint32_t irq_lock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static inline void irq_unlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

typedef struct
{
    uint16_t conn_interval_ms;
    uint32_t conn_interval_us;
    uint16_t slave_latency;
    uint16_t supervision_timeout_ms;
} ble_link_conn_state_t;

typedef struct
{
    uint8_t hop_increment;
    uint8_t channel_count;
    uint64_t channel_map_bits;
    uint8_t last_unmapped_channel;
    uint8_t channels[37];
} ble_link_channel_state_t;

typedef struct
{
    uint8_t access_address[4];
    uint32_t crc_init;
    uint16_t max_tx_octets;
    uint16_t max_rx_octets;
    uint8_t next_expected_rx_sn;
    uint8_t tx_sn;
} ble_link_packet_state_t;

typedef struct
{
    uint8_t tx_phy;
    uint8_t rx_phy;
} ble_link_phy_state_t;

typedef struct
{
    bool started;
    bool rx_seen_this_interval;
    uint16_t missed_interval_count;
} ble_link_supervision_state_t;

typedef struct
{
    bool valid;
    uint16_t instant;
    uint8_t map[5];
} ble_link_pending_channel_map_t;

typedef struct
{
    bool valid;
    uint16_t instant;
    uint32_t window_offset_us;
    ble_link_conn_state_t conn;
} ble_link_pending_conn_update_t;

typedef struct
{
    bool valid;
    uint16_t instant;
    ble_link_phy_state_t phy;
} ble_link_pending_phy_update_t;

typedef struct
{
    ble_gap_role_t role;
    bool connected;
    ble_gap_addr_t peer_addr;
    ble_link_conn_state_t conn;
    ble_link_channel_state_t channel;
    ble_link_packet_state_t packet;
    ble_link_phy_state_t phy;
    ble_link_supervision_state_t supervision;
    uint16_t event_counter;
    ble_link_pending_channel_map_t pending_channel_map;
    ble_link_pending_conn_update_t pending_conn_update;
    ble_link_pending_phy_update_t pending_phy_update;
} ble_link_t;

typedef struct
{
    ble_gap_role_t configured_role;
    char gap_device_name[BLE_GAP_DEVICE_NAME_MAX_LEN + 1U];
    uint8_t flags;
    int8_t tx_power;
    uint16_t adv_interval_ms;
    ble_scan_config_t scan_config;
    ble_gap_adv_name_type_t adv_name_type;
    uint8_t adv_short_name_length;
    ble_uuid_t included_service_uuid;
    ble_gap_conn_params_t preferred_conn_params;
    bool preferred_conn_params_valid;
    uint8_t vendor_uuid_base[BLE_UUID128_LEN];
    bool vendor_uuid_base_set;
    uint8_t next_l2cap_sig_identifier;
} ble_host_t;

typedef enum
{
    BLE_ADV_RADIO_PHASE_IDLE = 0,
    BLE_ADV_RADIO_PHASE_WAIT_ADV_TX_DISABLED,
    BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED,
    BLE_ADV_RADIO_PHASE_WAIT_SCAN_RSP_TX_DISABLED,
} ble_adv_radio_phase_t;

typedef enum
{
    BLE_SCAN_RADIO_PHASE_IDLE = 0,
    BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED,
    BLE_SCAN_RADIO_PHASE_WAIT_CONNECT_TX_DISABLED,
} ble_scan_radio_phase_t;

typedef enum
{
    BLE_CONN_RADIO_PHASE_IDLE = 0,
    BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED,
    BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED,
} ble_conn_radio_phase_t;

typedef struct
{
    bool tx_acked;
    bool is_new_packet;
    bool consumes_pending;
} ble_conn_bcmatch_state_t;

typedef struct
{
    ble_ll_adv_pdu_t adv_tx_pdu;
    ble_scan_rsp_pdu_t scan_rsp_pdu;
    ble_adv_rx_pdu_t adv_rx_pdu;
    ble_adv_rx_pdu_t scan_rx_pdu;
    ble_connect_req_pdu_t connect_req_pdu;
    ble_ll_data_raw_pdu_t conn_rx_pdu;
    ble_ll_data_raw_pdu_t conn_tx_pdu;
    ble_ll_data_raw_pdu_t last_conn_tx_pdu;
    ble_ll_data_raw_pdu_t pending_conn_tx_pdu;
    uint8_t adv_address[6];
    uint8_t adv_txadd;
    uint8_t scan_channel_index;
    bool scanning;
    bool scan_connect_pending;
    bool connect_filter_enabled;
    ble_gap_scan_filter_t connect_filter;
    bool connect_target_valid;
    ble_gap_addr_t connect_target;
    bool adv_scan_rsp_pending;
    bool adv_connect_pending;
    bool tx_unacked;
    bool has_pending_conn_tx_pdu;
    bool conn_rx_process_pending;
    ble_adv_radio_phase_t adv_radio_phase;
    ble_scan_radio_phase_t scan_radio_phase;
    ble_conn_radio_phase_t conn_radio_phase;
    ble_conn_bcmatch_state_t conn_bcmatch;
    uint32_t conn_next_event_tick_us;
} ble_ctrl_runtime_t;

typedef enum
{
    BLE_DEFERRED_EVT_KIND_GAP = 0,
    BLE_DEFERRED_EVT_KIND_GATT_SERVER,
    BLE_DEFERRED_EVT_KIND_GATT_CHARACTERISTIC,
    BLE_DEFERRED_EVT_KIND_SCAN_REPORT,
    BLE_DEFERRED_EVT_KIND_GATT_CLIENT,
} ble_deferred_evt_kind_t;

typedef struct
{
    ble_deferred_evt_kind_t kind;
    union
    {
        ble_gap_evt_t gap_evt;
        ble_gatt_server_evt_t gatt_server_evt;
        ble_gap_scan_report_t scan_report;
        struct
        {
            ble_gatt_char_evt_type_t evt_type;
            ble_gatt_characteristic_t *p_characteristic;
        } gatt_characteristic;
        ble_gatt_client_evt_t gatt_client;
    } params;
} ble_deferred_evt_t;

typedef struct
{
    volatile ble_deferred_evt_t q[BLE_EVT_QUEUE_SIZE];
    volatile uint8_t widx;
    volatile uint8_t ridx;
} ble_evt_dispatch_state_t;

extern const uint8_t m_adv_channels[3];
extern const uint8_t m_adv_freq_mhz_offset[3];
extern const uint8_t m_adv_access_address[4];
extern const uint8_t m_data_channel_freq[37];
extern const uint32_t m_ble_crc_poly;
extern const uint32_t m_adv_crc_init;
extern ble_host_t m_host;
extern ble_link_t m_link;
extern ble_gap_evt_handler_t m_gap_evt_handler;
extern ble_gap_scan_report_handler_t m_scan_report_handler;
extern ble_gatt_server_evt_handler_t m_gatt_server_evt_handler;
extern ble_gatt_client_evt_handler_t m_gatt_client_evt_handler;
extern ble_ctrl_runtime_t m_ctrl_rt;

static inline bool ble_host_role_is_configured(ble_gap_role_t role)
{
    return m_host.configured_role == role;
}

uint16_t u16_decode(const uint8_t *p_src);
void u16_encode(uint16_t value, uint8_t *p_dst);
void ble_evt_dispatch_init(void);
bool ble_evt_notify_gap(ble_gap_evt_type_t evt_type);
bool ble_evt_notify_scan_report(const ble_gap_scan_report_t *p_report);
bool ble_evt_notify_gatt_characteristic(ble_gatt_char_evt_type_t evt_type, ble_gatt_characteristic_t *p_characteristic);
bool ble_evt_notify_gatt_server_mtu_exchange(uint16_t requested_mtu, uint16_t response_mtu, uint16_t effective_mtu);
bool ble_evt_notify_gatt_client(const ble_gatt_client_evt_t *p_evt);
bool ble_uuid_is_valid(const ble_uuid_t *p_uuid);
uint16_t ble_uuid_encoded_len(const ble_uuid_t *p_uuid);
bool ble_uuid_encode(const ble_uuid_t *p_uuid, uint8_t *p_dst);
bool ble_uuid_matches_bytes(const ble_uuid_t *p_uuid, const uint8_t *p_uuid_bytes, uint16_t uuid_len);
void ble_conn_param_update_timer_init(void);
void ble_conn_param_update_timer_start(void);
void ble_conn_param_update_timer_stop(void);
void controller_load_identity_address(void);

void controller_runtime_init(void);
bool controller_queue_l2cap_payload(uint16_t cid, const uint8_t *p_payload, uint16_t payload_len);
void controller_start_advertising_internal(void);
void controller_stop_scanning_internal(void);
void controller_start_scanning_internal(void);
bool controller_start_connecting(void);
void controller_disconnect_internal(void);
void ble_gatt_client_init(void);
void ble_gatt_client_reset_connection_state(void);
uint16_t ble_gatt_client_process_pdu(const uint8_t *p_att, uint16_t att_len, uint8_t *p_rsp, uint16_t rsp_max_len);

#endif /* BLE_RUNTIME_INTERNAL_H__ */
