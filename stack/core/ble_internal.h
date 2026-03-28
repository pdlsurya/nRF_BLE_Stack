/**
 * @file ble_internal.h
 * @author Surya Poudel
 * @brief Internal definitions and shared runtime state for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_INTERNAL_H__
#define BLE_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>

#include "app_timer.h"
#include "nrf_ble.h"
#include "radio_driver.h"

#define BLE_MAX_ADV_DATA_LEN 31U
#define BLE_ADV_PDU_OVERHEAD 6U
#define BLE_ADV_NAME_MAX_LEN 20U
#define BLE_IDENTITY_SALT 0x434D535456324C31ULL
#define BLE_L2CAP_CID_ATT 0x0004U
#define BLE_L2CAP_HDR_LEN 4U
#define BLE_ADV_RX_WINDOW_US 1500U
#define BLE_CONN_TXEN_DELAY_US 80U
#define BLE_CONN_EVENT_GUARD_US 500U
#define BLE_CONN_TIMER_PRESCALER 4U
#define BLE_CONN_TIMER_IRQ_PRIORITY 0U
#define BLE_EVT_IRQ_PRIORITY 6U
#define BLE_EVT_QUEUE_SIZE 16U

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
#define BLE_LL_VERSION_5_0 0x09U
#define BLE_LL_COMPANY_ID_NORDIC 0x0059U
#define BLE_LL_SUBVERSION 0x0000U

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

typedef enum
{
    LL_ADV_IND = 0x00,
    LL_ADV_DIRECT_IND,
    LL_ADV_NONCONN_IND,
    LL_SCAN_REQ,
    LL_SCAN_RSP,
    LL_CONNECT_REQ,
    LL_ADV_SCAN_IND,
    LL_AUX_ADV_IND,
    LL_AUX_SCAN_REQ,
    LL_AUX_SCAN_RSP,
    LL_AUX_CONNECT_REQ,
    LL_AUX_CHAIN_IND,
    LL_AUX_SYNC_IND,
    LL_AUX_SYNC_INFO,
    LL_AUX_CONNECT_RSP
} ble_adv_pdu_type_t;

typedef enum
{
    BLE_LLID_RESERVED = 0x00,
    BLE_LLID_CONTINUATION = 0x01,
    BLE_LLID_START_L2CAP = 0x02,
    BLE_LLID_CONTROL_PDU = 0x03
} ble_llid_t;

typedef struct
{
    uint8_t pdu_type : 4;
    uint8_t rfu : 2;
    uint8_t txadd : 1;
    uint8_t rxadd : 1;
} __attribute__((packed)) ble_ll_adv_header_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t mac_address[6];
    uint8_t payload[128];
} __attribute__((packed)) ble_ll_adv_pdu_t;

typedef struct
{
    uint8_t access_address[4];
    uint8_t crc_init[3];
    uint8_t win_size;
    uint16_t win_offset;
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
    uint8_t channel_map[5];
    uint8_t hop_increment : 5;
    uint8_t sca : 3;
} __attribute__((packed)) ble_ll_connect_ind_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t initiator_address[6];
    uint8_t advertiser_address[6];
    ble_ll_connect_ind_t ll_data;
} __attribute__((packed)) ble_connect_req_pdu_t;

typedef struct
{
    uint8_t llid : 2;
    uint8_t nesn : 1;
    uint8_t sn : 1;
    uint8_t md : 1;
    uint8_t rfu : 3;
} __attribute__((packed)) ble_ll_data_header_t;

typedef struct
{
    ble_ll_data_header_t header;
    uint8_t length;
    uint8_t payload[251];
} __attribute__((packed)) ble_ll_data_raw_pdu_t;

typedef struct
{
    bool advertising;
    bool adv_timer_created;
} ble_controller_t;

typedef struct
{
    bool connected;
    uint16_t conn_interval_ms;
    uint32_t conn_interval_us;
    uint16_t supervision_timeout_ms;
    uint8_t hop_increment;
    uint8_t channel_count;
    uint8_t channel_map[5];
    uint8_t last_unmapped_channel;
    uint8_t current_channel_index;
    uint8_t channels[37];
    uint8_t access_address[4];
    uint32_t crc_init;
    uint8_t rx_sn;
    uint8_t tx_sn;
    bool supervision_started;
    bool rx_seen_this_interval;
    bool event_anchor_captured;
    uint16_t missed_interval_count;
    uint16_t event_counter;
    bool pending_channel_map_valid;
    uint16_t pending_channel_map_instant;
    uint8_t pending_channel_map[5];
} ble_link_t;

typedef struct
{
    char adv_name[BLE_ADV_NAME_MAX_LEN + 1U];
    uint8_t adv_name_len;
    uint8_t flags;
    int8_t tx_power;
    uint16_t adv_interval_ms;
    uint16_t included_service_uuid;
    bool has_service_data;
    ble_service_data_t service_data;
    ble_gap_conn_params_t gap_conn_params;
} ble_host_t;

typedef struct
{
    ble_ll_adv_pdu_t air_pdu;
    ble_ll_data_raw_pdu_t conn_rx_pdu;
    ble_ll_data_raw_pdu_t conn_tx_pdu;
    ble_ll_data_raw_pdu_t last_conn_tx_pdu;
    ble_ll_data_raw_pdu_t pending_conn_tx_pdu;
    uint8_t adv_address[6];
    uint8_t adv_txadd;
    bool has_last_conn_tx_pdu;
    bool tx_unacked;
    bool has_pending_conn_tx_pdu;
    bool conn_timer_initialized;
    uint32_t conn_next_event_tick_us;
} ble_ctrl_runtime_t;

typedef enum
{
    BLE_DEFERRED_EVT_KIND_GAP = 0,
    BLE_DEFERRED_EVT_KIND_GATT,
    BLE_DEFERRED_EVT_KIND_GATT_CHARACTERISTIC,
} ble_deferred_evt_kind_t;

typedef struct
{
    ble_deferred_evt_kind_t kind;
    union
    {
        ble_evt_t stack_evt;
        struct
        {
            ble_gatt_evt_type_t evt_type;
            ble_gatt_characteristic_t *p_characteristic;
            uint8_t data[BLE_GATT_MAX_VALUE_LEN];
            uint16_t len;
            bool notifications_enabled;
        } gatt_characteristic;
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
extern ble_controller_t m_controller;
extern ble_link_t m_link;
extern ble_evt_handler_t m_evt_handler;
extern ble_ctrl_runtime_t m_ctrl_rt;

uint16_t u16_decode(const uint8_t *p_src);
uint8_t adv_pdu_type_get(const ble_ll_adv_pdu_t *p_pdu);
void u16_encode(uint16_t value, uint8_t *p_dst);
void ble_evt_dispatch_init(void);
bool ble_evt_notify_gap(ble_evt_type_t evt_type);
bool ble_evt_notify_gatt_characteristic(ble_gatt_evt_type_t evt_type,
                                        ble_gatt_characteristic_t *p_characteristic,
                                        const uint8_t *p_data,
                                        uint16_t len,
                                        bool notifications_enabled);
bool ble_evt_notify_gatt_mtu_exchange(uint16_t requested_mtu,
                                      uint16_t response_mtu,
                                      uint16_t effective_mtu);
void controller_load_identity_address(void);

void controller_runtime_init(void);
bool controller_queue_att_payload(const uint8_t *p_att_payload, uint16_t att_len);
void controller_disconnect_internal(void);

#endif
