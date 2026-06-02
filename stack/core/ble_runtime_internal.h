/* SPDX-License-Identifier: MIT */

#ifndef BLE_RUNTIME_INTERNAL_H__
#define BLE_RUNTIME_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>

#include "app_timer.h"
#include "ble_gap.h"
#include "ble_gatt_client.h"
#include "ble_gatt_server.h"
#include "ble_host_internal.h"
#include "ble_l2cap_internal.h"
#include "ble_controller_state_internal.h"
#include "ble_ll_internal.h"
#include "radio_driver.h"

#define BLE_IDENTITY_SALT 0x434D535456324C39ULL
#define BLE_EVT_IRQ_PRIORITY 6U
#define BLE_EVT_QUEUE_SIZE 16U

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
    uint16_t max_tx_time_us;
    uint16_t max_rx_time_us;
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
    uint8_t peer_features[8];
    ble_link_pending_channel_map_t pending_channel_map;
    ble_link_pending_conn_update_t pending_conn_update;
    ble_link_pending_phy_update_t pending_phy_update;
} ble_link_t;

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
extern ble_link_t m_link;
extern ble_gap_evt_handler_t m_gap_evt_handler;
extern ble_gap_scan_report_handler_t m_scan_report_handler;
extern ble_gatt_server_evt_handler_t m_gatt_server_evt_handler;
extern ble_gatt_client_evt_handler_t m_gatt_client_evt_handler;
extern ble_ctrl_runtime_t m_ctrl_rt;

uint16_t u16_decode(const uint8_t *p_src);
void u16_encode(uint16_t value, uint8_t *p_dst);
void ble_evt_dispatch_init(void);
bool ble_evt_notify_gap(ble_gap_evt_type_t evt_type);
bool ble_evt_notify_gap_ctrl_procedure_unsupported(ble_gap_ctrl_procedure_t procedure, uint8_t unsupported_opcode);
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
bool controller_central_initiate_conn_update(const ble_gap_conn_params_t *p_params);
void controller_peripheral_start_advertising_internal(void);
void controller_central_stop_scanning_internal(void);
void controller_central_start_scanning_internal(void);
bool controller_central_start_connecting(void);
void controller_disconnect_internal(void);

#endif /* BLE_RUNTIME_INTERNAL_H__ */
