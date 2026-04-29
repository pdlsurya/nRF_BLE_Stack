/**
 * @file ble_controller_central.c
 * @author Surya Poudel
 * @brief Central-role portions of the BLE controller implementation
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_controller_shared.h"

#include <string.h>

APP_TIMER_DEF(m_scan_timer_id);
APP_TIMER_DEF(m_scan_window_timer_id);

static bool controller_scan_addr_matches_filter(const ble_gap_addr_t *p_addr)
{
    if ((p_addr == NULL) || !m_ctrl_rt.connect_filter_enabled || !m_ctrl_rt.connect_filter.match_addr)
    {
        return false;
    }

    return (p_addr->addr_is_random == m_ctrl_rt.connect_filter.addr.addr_is_random) &&
           (memcmp(p_addr->addr, m_ctrl_rt.connect_filter.addr.addr, sizeof(p_addr->addr)) == 0);
}

static bool controller_adv_data_next(const uint8_t *p_adv_data,
                                     uint8_t adv_data_len,
                                     uint8_t *p_offset,
                                     uint8_t *p_ad_type,
                                     const uint8_t **pp_value,
                                     uint8_t *p_value_len)
{
    uint8_t field_len;

    if ((p_adv_data == NULL) ||
        (p_offset == NULL) ||
        (p_ad_type == NULL) ||
        (pp_value == NULL) ||
        (p_value_len == NULL) ||
        (*p_offset >= adv_data_len))
    {
        return false;
    }

    field_len = p_adv_data[*p_offset];
    if ((field_len == 0U) ||
        ((uint16_t)(*p_offset) + (uint16_t)field_len + 1U > (uint16_t)adv_data_len))
    {
        return false;
    }

    *p_ad_type = p_adv_data[*p_offset + 1U];
    *p_value_len = (uint8_t)(field_len - 1U);
    *pp_value = &p_adv_data[*p_offset + 2U];
    *p_offset = (uint8_t)(*p_offset + field_len + 1U);
    return true;
}

static bool controller_adv_data_matches_name(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    const uint8_t *p_value;
    uint8_t offset = 0U;
    uint8_t ad_type;
    uint8_t value_len;
    size_t filter_name_len = 0U;

    while ((filter_name_len < sizeof(m_ctrl_rt.connect_filter.name)) &&
           (m_ctrl_rt.connect_filter.name[filter_name_len] != '\0'))
    {
        filter_name_len++;
    }

    if (filter_name_len == 0U)
    {
        return false;
    }

    while (controller_adv_data_next(p_adv_data, adv_data_len, &offset, &ad_type, &p_value, &value_len))
    {
        if ((ad_type != BLE_AD_TYPE_SHORT_LOCAL_NAME) &&
            (ad_type != BLE_AD_TYPE_COMPLETE_LOCAL_NAME))
        {
            continue;
        }

        if ((value_len == filter_name_len) &&
            (memcmp(p_value, m_ctrl_rt.connect_filter.name, filter_name_len) == 0))
        {
            return true;
        }
    }

    return false;
}

static bool controller_adv_data_contains_uuid_bytes(const uint8_t *p_adv_data,
                                                    uint8_t adv_data_len,
                                                    const uint8_t *p_uuid_bytes,
                                                    uint16_t uuid_len,
                                                    uint8_t ad_type_incomplete,
                                                    uint8_t ad_type_complete)
{
    const uint8_t *p_value;
    uint8_t offset = 0U;
    uint8_t ad_type;
    uint8_t value_len;
    uint8_t i;

    if ((p_adv_data == NULL) || (p_uuid_bytes == NULL) || (uuid_len == 0U))
    {
        return false;
    }

    while (controller_adv_data_next(p_adv_data, adv_data_len, &offset, &ad_type, &p_value, &value_len))
    {
        if ((ad_type != ad_type_incomplete) && (ad_type != ad_type_complete))
        {
            continue;
        }

        if ((value_len == 0U) || ((value_len % uuid_len) != 0U))
        {
            continue;
        }

        for (i = 0U; (uint16_t)i + uuid_len <= value_len; i = (uint8_t)(i + uuid_len))
        {
            if (memcmp(&p_value[i], p_uuid_bytes, uuid_len) == 0)
            {
                return true;
            }
        }
    }

    return false;
}

static bool controller_adv_data_contains_service_uuid16(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    uint8_t uuid16[BLE_UUID16_LEN];

    u16_encode(m_ctrl_rt.connect_filter.service_uuid16, uuid16);
    return controller_adv_data_contains_uuid_bytes(p_adv_data,
                                                   adv_data_len,
                                                   uuid16,
                                                   BLE_UUID16_LEN,
                                                   BLE_AD_TYPE_INCOMPLETE_UUID16_LIST,
                                                   BLE_AD_TYPE_COMPLETE_UUID16_LIST);
}

static bool controller_adv_data_contains_service_uuid128(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    return controller_adv_data_contains_uuid_bytes(p_adv_data,
                                                   adv_data_len,
                                                   m_ctrl_rt.connect_filter.service_uuid128,
                                                   BLE_UUID128_LEN,
                                                   BLE_AD_TYPE_INCOMPLETE_UUID128_LIST,
                                                   BLE_AD_TYPE_COMPLETE_UUID128_LIST);
}

bool controller_scan_filter_matches(const ble_gap_addr_t *p_addr, const ble_adv_rx_pdu_t *p_rx)
{
    const uint8_t *p_adv_data;
    uint8_t adv_data_len;

    if ((p_addr == NULL) || (p_rx == NULL) || !m_ctrl_rt.connect_filter_enabled)
    {
        return false;
    }

    if (m_ctrl_rt.connect_filter.match_addr &&
        !controller_scan_addr_matches_filter(p_addr))
    {
        return false;
    }

    adv_data_len = (p_rx->adv.payload_length > BLE_ADV_ADVERTISER_ADDRESS_LEN)
                       ? (uint8_t)(p_rx->adv.payload_length - BLE_ADV_ADVERTISER_ADDRESS_LEN)
                       : 0U;
    p_adv_data = p_rx->adv.payload;

    if (m_ctrl_rt.connect_filter.match_name &&
        !controller_adv_data_matches_name(p_adv_data, adv_data_len))
    {
        return false;
    }

    if (m_ctrl_rt.connect_filter.match_service_uuid16 &&
        !controller_adv_data_contains_service_uuid16(p_adv_data, adv_data_len))
    {
        return false;
    }

    if (m_ctrl_rt.connect_filter.match_service_uuid128 &&
        !controller_adv_data_contains_service_uuid128(p_adv_data, adv_data_len))
    {
        return false;
    }

    return true;
}

static uint32_t controller_seed_next(uint32_t *p_state)
{
    *p_state = (*p_state * 1664525UL) + 1013904223UL;
    return *p_state;
}

static uint32_t controller_access_address_generate(const ble_gap_addr_t *p_peer_addr)
{
    uint32_t seed;
    uint8_t i;

    seed = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1] ^ 0xA55A3C5DU;
    if (p_peer_addr != NULL)
    {
        for (i = 0U; i < sizeof(p_peer_addr->addr); i++)
        {
            seed ^= ((uint32_t)p_peer_addr->addr[i]) << ((i % 4U) * 8U);
            seed = controller_seed_next(&seed);
        }
    }

    if ((seed == 0U) || (seed == 0x8E89BED6UL))
    {
        seed ^= 0xC3A55A3CU;
    }

    return seed;
}

void controller_build_connect_request(const ble_gap_addr_t *p_peer_addr)
{
    uint32_t access_address;
    uint32_t crc_init;
    uint16_t conn_interval_units;
    uint16_t supervision_timeout_units;
    uint8_t i;

    (void)memset(&m_ctrl_rt.connect_req_pdu, 0, sizeof(m_ctrl_rt.connect_req_pdu));
    m_ctrl_rt.connect_req_pdu.header.pdu_type = LL_CONNECT_REQ;
    m_ctrl_rt.connect_req_pdu.header.rfu = 0U;
    m_ctrl_rt.connect_req_pdu.header.txadd = (uint8_t)(m_ctrl_rt.adv_txadd & 0x01U);
    m_ctrl_rt.connect_req_pdu.header.rxadd = (uint8_t)(p_peer_addr->addr_is_random ? 1U : 0U);
    (void)memcpy(m_ctrl_rt.connect_req_pdu.initiator_address, m_ctrl_rt.adv_address, sizeof(m_ctrl_rt.connect_req_pdu.initiator_address));
    (void)memcpy(m_ctrl_rt.connect_req_pdu.advertiser_address, p_peer_addr->addr, sizeof(m_ctrl_rt.connect_req_pdu.advertiser_address));

    access_address = controller_access_address_generate(p_peer_addr);
    crc_init = controller_seed_next(&access_address) & 0x00FFFFFFUL;
    if (crc_init == 0U)
    {
        crc_init = 0x00ABCDE1UL;
    }

    conn_interval_units = m_host.preferred_conn_params_valid
                              ? m_host.preferred_conn_params.min_conn_interval_1p25ms
                              : BLE_INITIATOR_CONN_INTERVAL_UNITS_DEFAULT;
    supervision_timeout_units = m_host.preferred_conn_params_valid
                                    ? m_host.preferred_conn_params.supervision_timeout_10ms
                                    : BLE_INITIATOR_SUPERVISION_TIMEOUT_UNITS_DEFAULT;

    m_ctrl_rt.connect_req_pdu.payload_length = (uint8_t)(sizeof(m_ctrl_rt.connect_req_pdu.initiator_address) +
                                                         sizeof(m_ctrl_rt.connect_req_pdu.advertiser_address) +
                                                         sizeof(m_ctrl_rt.connect_req_pdu.ll_data));
    (void)memcpy(m_ctrl_rt.connect_req_pdu.ll_data.access_address, &access_address, sizeof(m_ctrl_rt.connect_req_pdu.ll_data.access_address));
    m_ctrl_rt.connect_req_pdu.ll_data.crc_init[0] = (uint8_t)(crc_init & 0xFFU);
    m_ctrl_rt.connect_req_pdu.ll_data.crc_init[1] = (uint8_t)((crc_init >> 8) & 0xFFU);
    m_ctrl_rt.connect_req_pdu.ll_data.crc_init[2] = (uint8_t)((crc_init >> 16) & 0xFFU);
    m_ctrl_rt.connect_req_pdu.ll_data.win_size = BLE_INITIATOR_WIN_SIZE_UNITS;
    m_ctrl_rt.connect_req_pdu.ll_data.win_offset = BLE_INITIATOR_WIN_OFFSET_UNITS;
    m_ctrl_rt.connect_req_pdu.ll_data.interval = conn_interval_units;
    m_ctrl_rt.connect_req_pdu.ll_data.latency = m_host.preferred_conn_params_valid ? m_host.preferred_conn_params.slave_latency : 0U;
    m_ctrl_rt.connect_req_pdu.ll_data.timeout = supervision_timeout_units;
    for (i = 0U; i < 5U; i++)
    {
        m_ctrl_rt.connect_req_pdu.ll_data.channel_map[i] = 0xFFU;
    }
    m_ctrl_rt.connect_req_pdu.ll_data.channel_map[4] = 0x1FU;
    m_ctrl_rt.connect_req_pdu.ll_data.hop_increment = BLE_INITIATOR_HOP_INCREMENT;
    m_ctrl_rt.connect_req_pdu.ll_data.sca = 0U;
}

void controller_publish_scan_report(const ble_adv_rx_pdu_t *p_rx)
{
    ble_gap_scan_report_t report;
    uint8_t adv_type;
    uint8_t payload_len;
    uint8_t data_len = 0U;

    if (p_rx == NULL)
    {
        return;
    }

    adv_type = p_rx->adv.header.pdu_type;
    if (!controller_adv_type_is_reportable(adv_type))
    {
        return;
    }

    (void)memset(&report, 0, sizeof(report));
    report.addr.addr_is_random = (p_rx->adv.header.txadd != 0U);
    (void)memcpy(report.addr.addr, p_rx->adv.mac_address, sizeof(report.addr.addr));
    report.adv_type = adv_type;
    report.connectable = controller_adv_type_is_connectable(adv_type);
    report.scannable = controller_adv_type_is_scannable(adv_type);
    report.rssi = 0;

    payload_len = p_rx->adv.payload_length;
    if (payload_len > BLE_ADV_ADVERTISER_ADDRESS_LEN)
    {
        data_len = (uint8_t)(payload_len - BLE_ADV_ADVERTISER_ADDRESS_LEN);
        if (data_len > BLE_GAP_ADV_DATA_MAX_LEN)
        {
            data_len = BLE_GAP_ADV_DATA_MAX_LEN;
        }
        (void)memcpy(report.data, p_rx->adv.payload, data_len);
        report.data_len = data_len;
    }

    (void)ble_evt_notify_scan_report(&report);
}

void controller_scan_timer_start(uint32_t interval_ms)
{
    APP_ERROR_CHECK(app_timer_start(m_scan_timer_id, APP_TIMER_TICKS(interval_ms), NULL));
}

void controller_scan_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_scan_timer_id));
}

void controller_scan_window_timer_start(uint32_t window_ms)
{
    APP_ERROR_CHECK(app_timer_start(m_scan_window_timer_id, APP_TIMER_TICKS(window_ms), NULL));
}

void controller_scan_window_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_scan_window_timer_id));
}

static void controller_scan_window_timeout_handler(void *p_context)
{
    uint32_t primask;

    (void)p_context;

    primask = irq_lock();
    if (m_ctrl_rt.scanning &&
        !m_link.connected &&
        (m_ctrl_rt.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED) &&
        controller_radio_is_rx_window_active() &&
        !controller_radio_has_pending_completion())
    {
        controller_reset_scan_radio_state();
        radio_disable();
    }
    irq_unlock(primask);
}

static void controller_scan_timer_handler(void *p_context)
{
    uint8_t ch;

    (void)p_context;

    if (m_link.connected || !m_ctrl_rt.scanning || (m_ctrl_rt.scan_radio_phase != BLE_SCAN_RADIO_PHASE_IDLE))
    {
        return;
    }

    ch = (uint8_t)(m_ctrl_rt.scan_channel_index % 3U);
    m_ctrl_rt.scan_channel_index = (uint8_t)((m_ctrl_rt.scan_channel_index + 1U) % 3U);
    (void)memset(&m_ctrl_rt.scan_rx_pdu, 0, sizeof(m_ctrl_rt.scan_rx_pdu));
    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_SCAN);
    radio_set_frequency(m_adv_freq_mhz_offset[ch]);
    radio_set_whiteiv(m_adv_channels[ch]);
    radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.scan_rx_pdu);
    m_ctrl_rt.scan_radio_phase = BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED;
    controller_set_mode_with_phy(RADIO_MODE_RX, BLE_LL_PHY_1M);

    controller_scan_window_timer_start(m_host.scan_config.window_ms);
}

void controller_stop_scanning_internal(void)
{
    controller_scan_timer_stop();
    controller_scan_window_timer_stop();
    m_ctrl_rt.scanning = false;
    m_ctrl_rt.connect_target_valid = false;
    controller_reset_scan_radio_state();
    if (!m_link.connected && (m_ctrl_rt.adv_radio_phase == BLE_ADV_RADIO_PHASE_IDLE))
    {
        radio_disable();
        radio_enable_interrupt_mask(0U);
    }
}

void controller_start_scanning_internal(void)
{
    if (m_link.connected || m_ctrl_rt.scanning)
    {
        return;
    }

    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.scan_rx_pdu.adv.payload),
                                    m_adv_access_address,
                                    m_adv_crc_init,
                                    (uint32_t)&m_ctrl_rt.scan_rx_pdu);
    m_ctrl_rt.scanning = true;
    m_ctrl_rt.scan_channel_index = 0U;

    controller_scan_timer_stop();
    controller_scan_timer_start(m_host.scan_config.interval_ms);
    controller_scan_timer_handler(NULL);
}

bool controller_start_connecting(void)
{
    if (!m_ctrl_rt.connect_filter_enabled || m_link.connected)
    {
        return false;
    }

    m_ctrl_rt.connect_target_valid = false;
    controller_start_scanning_internal();
    return true;
}

void controller_apply_central_connect_request(void)
{
    uint32_t first_event_delay_us;

    controller_prepare_connected_link(&m_ctrl_rt.connect_req_pdu, BLE_GAP_ROLE_CENTRAL, &m_ctrl_rt.connect_target);
    m_ctrl_rt.connect_target_valid = false;
    m_ctrl_rt.scanning = false;
    first_event_delay_us = (m_ctrl_rt.connect_req_pdu.ll_data.win_offset == 0U)
                               ? 2000U
                               : UNITS_1P25MS_TO_US(m_ctrl_rt.connect_req_pdu.ll_data.win_offset);
    controller_connected_timer_start(first_event_delay_us);
    (void)ble_evt_notify_gap(BLE_GAP_EVT_CONNECTED);
}

void controller_start_connection_event_central(void)
{
    bool new_tx_pdu;

    controller_hop_data_channel();
    m_ctrl_rt.conn_rx_pdu.header = (ble_ll_data_header_t){0};
    m_ctrl_rt.conn_rx_pdu.length = 0U;
    m_ctrl_rt.conn_rx_process_pending = false;
    controller_reset_conn_bcmatch_state();
    m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED;
    new_tx_pdu = !m_ctrl_rt.tx_unacked;

    if (new_tx_pdu)
    {
        if (m_ctrl_rt.has_pending_conn_tx_pdu)
        {
            m_ctrl_rt.conn_tx_pdu = m_ctrl_rt.pending_conn_tx_pdu;
            m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(m_ctrl_rt.conn_tx_pdu.header.llid);
            m_ctrl_rt.conn_bcmatch.consumes_pending = true;
        }
        else
        {
            m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTINUATION);
            m_ctrl_rt.conn_tx_pdu.length = 0U;
        }
    }

    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN);
    radio_set_bcc(BLE_LL_DATA_HEADER_BITS);
    radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
    controller_stage_conn_response(new_tx_pdu);
    controller_set_mode_with_phy(RADIO_MODE_TX, m_link.phy.tx_phy);
}

void radio_handle_connected_packet_central(void)
{
    bool tx_acked = m_ctrl_rt.tx_unacked && (m_ctrl_rt.conn_rx_pdu.header.nesn != m_link.packet.tx_sn);
    bool is_new_packet = (m_ctrl_rt.conn_rx_pdu.header.sn == m_link.packet.next_expected_rx_sn);
    bool consumes_pending = m_ctrl_rt.conn_bcmatch.consumes_pending;

    m_link.supervision.rx_seen_this_interval = true;
    if (tx_acked)
    {
        m_ctrl_rt.tx_unacked = false;
        m_link.packet.tx_sn ^= 1U;
        if (consumes_pending)
        {
            m_ctrl_rt.has_pending_conn_tx_pdu = false;
        }
    }

    if (is_new_packet)
    {
        m_link.packet.next_expected_rx_sn ^= 1U;
    }

    m_ctrl_rt.conn_rx_process_pending = is_new_packet;
    controller_reset_conn_bcmatch_state();
    m_link.supervision.started = true;
}

void radio_handle_connected_crc_error_central(void)
{
    m_ctrl_rt.conn_rx_process_pending = false;
    controller_reset_conn_bcmatch_state();
    m_link.supervision.started = true;
}

void controller_handle_connected_disabled_central(void)
{
    if (m_ctrl_rt.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.conn_radio_phase == BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED)
    {
        m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED;
        radio_set_packet_ptr((uint32_t)&m_ctrl_rt.conn_rx_pdu);
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
        radio_rx_enable();
        return;
    }

    m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
    if (m_ctrl_rt.conn_rx_process_pending)
    {
        m_ctrl_rt.conn_rx_process_pending = false;
        controller_process_received_conn_pdu();
    }
}

void controller_handle_scanning_disabled(void)
{
    if (m_ctrl_rt.scan_radio_phase == BLE_SCAN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        if (m_ctrl_rt.scan_connect_pending)
        {
            m_ctrl_rt.scan_connect_pending = false;
            controller_scan_window_timer_stop();
            m_ctrl_rt.scan_radio_phase = BLE_SCAN_RADIO_PHASE_WAIT_CONNECT_TX_DISABLED;
            radio_set_packet_ptr((uint32_t)&m_ctrl_rt.connect_req_pdu);
            radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
            radio_tx_enable();
            return;
        }

        controller_scan_window_timer_stop();
        controller_reset_scan_radio_state();
        return;
    }

    controller_scan_window_timer_stop();
    controller_reset_scan_radio_state();
    controller_apply_central_connect_request();
}

void controller_scan_timers_init(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_scan_timer_id, APP_TIMER_MODE_REPEATED, controller_scan_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_scan_window_timer_id, APP_TIMER_MODE_SINGLE_SHOT, controller_scan_window_timeout_handler));
}
