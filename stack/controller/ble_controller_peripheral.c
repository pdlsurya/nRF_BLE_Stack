/**
 * @file ble_controller_peripheral.c
 * @author Surya Poudel
 * @brief Peripheral-role portions of the BLE controller implementation
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_controller_shared.h"

#include <string.h>

APP_TIMER_DEF(m_adv_timer_id);

static bool host_add_ad_structure(uint8_t *p_adv_data_len, uint8_t type, const uint8_t *p_data, uint8_t size)
{
    uint8_t *p_adv_data;

    if ((uint16_t)(*p_adv_data_len) + (uint16_t)size + 2U > BLE_LL_ADV_DATA_MAX_LEN)
    {
        return false;
    }

    p_adv_data = &m_ctrl_rt.adv_tx_pdu.payload[*p_adv_data_len];
    p_adv_data[0] = (uint8_t)(size + 1U);
    p_adv_data[1] = type;
    (void)memcpy(&p_adv_data[2], p_data, size);
    *p_adv_data_len = (uint8_t)(*p_adv_data_len + size + 2U);

    return true;
}

static void host_add_device_name_ad_structure(uint8_t *p_adv_data_len)
{
    size_t full_name_len;
    uint8_t ad_type;
    uint8_t adv_name_len;

    if ((p_adv_data_len == NULL) || (m_host.gap_device_name[0] == '\0'))
    {
        return;
    }

    full_name_len = strlen(m_host.gap_device_name);
    if (m_host.adv_name_type == BLE_GAP_ADV_NAME_SHORT)
    {
        adv_name_len = (m_host.adv_short_name_length < full_name_len) ? m_host.adv_short_name_length : (uint8_t)full_name_len;
        ad_type = 0x08U;
    }
    else
    {
        adv_name_len = (uint8_t)full_name_len;
        ad_type = 0x09U;
    }

    if (adv_name_len == 0U)
    {
        return;
    }

    (void)host_add_ad_structure(p_adv_data_len, ad_type, (const uint8_t *)m_host.gap_device_name, adv_name_len);
}

static void host_build_adv_pdu(void)
{
    uint8_t adv_data_len = 0U;
    uint16_t uuid_len;
    uint8_t uuid_ad_type;
    uint8_t uuid_bytes[BLE_UUID128_LEN];

    (void)memset(&m_ctrl_rt.adv_tx_pdu, 0, sizeof(m_ctrl_rt.adv_tx_pdu));

    m_ctrl_rt.adv_tx_pdu.header.pdu_type = BLE_PRIMARY_ADV_PDU_TYPE;
    m_ctrl_rt.adv_tx_pdu.header.rfu = 0U;
    m_ctrl_rt.adv_tx_pdu.header.txadd = (uint8_t)(m_ctrl_rt.adv_txadd & 0x01U);
    m_ctrl_rt.adv_tx_pdu.header.rxadd = 0U;
    (void)memcpy(m_ctrl_rt.adv_tx_pdu.mac_address, m_ctrl_rt.adv_address, sizeof(m_ctrl_rt.adv_address));

    (void)host_add_ad_structure(&adv_data_len, 0x01U, &m_host.flags, 1U);
    host_add_device_name_ad_structure(&adv_data_len);

    uuid_len = ble_uuid_encoded_len(&m_host.included_service_uuid);
    if ((uuid_len != 0U) && ble_uuid_encode(&m_host.included_service_uuid, uuid_bytes))
    {
        uuid_ad_type = (uuid_len == BLE_UUID128_LEN) ? 0x07U : 0x03U;
        (void)host_add_ad_structure(&adv_data_len, uuid_ad_type, uuid_bytes, (uint8_t)uuid_len);
    }

    (void)host_add_ad_structure(&adv_data_len, 0x0AU, (const uint8_t *)&m_host.tx_power, 1U);

    m_ctrl_rt.adv_tx_pdu.payload_length = (uint8_t)(BLE_ADV_ADVERTISER_ADDRESS_LEN + adv_data_len);
}

static void host_build_scan_rsp_pdu(void)
{
    (void)memset(&m_ctrl_rt.scan_rsp_pdu, 0, sizeof(m_ctrl_rt.scan_rsp_pdu));

    m_ctrl_rt.scan_rsp_pdu.header.pdu_type = LL_SCAN_RSP;
    m_ctrl_rt.scan_rsp_pdu.header.rfu = 0U;
    m_ctrl_rt.scan_rsp_pdu.header.txadd = (uint8_t)(m_ctrl_rt.adv_txadd & 0x01U);
    m_ctrl_rt.scan_rsp_pdu.header.rxadd = 0U;
    (void)memcpy(m_ctrl_rt.scan_rsp_pdu.advertiser_address, m_ctrl_rt.adv_address, sizeof(m_ctrl_rt.scan_rsp_pdu.advertiser_address));

    /* Keep the scan response minimal to maximize RX->TX turnaround margin. */
    m_ctrl_rt.scan_rsp_pdu.payload_length = BLE_ADV_ADVERTISER_ADDRESS_LEN;
}

bool controller_scan_request_targets_us(const ble_scan_req_pdu_t *p_req)
{
    if (p_req == NULL)
    {
        return false;
    }

    if (p_req->header.pdu_type != LL_SCAN_REQ)
    {
        return false;
    }

    if (p_req->payload_length != BLE_SCAN_REQ_PAYLOAD_LEN)
    {
        return false;
    }

    if (p_req->header.rxadd != (m_ctrl_rt.adv_txadd & 0x01U))
    {
        return false;
    }

    return memcmp(p_req->advertiser_address, m_ctrl_rt.adv_address, sizeof(m_ctrl_rt.adv_address)) == 0;
}

bool controller_connect_request_targets_us(const ble_connect_req_pdu_t *p_req)
{
    const uint8_t expected_payload_len = (uint8_t)(sizeof(((ble_connect_req_pdu_t *)0)->initiator_address) + sizeof(((ble_connect_req_pdu_t *)0)->advertiser_address) + sizeof(((ble_connect_req_pdu_t *)0)->ll_data));

    if (p_req == NULL)
    {
        return false;
    }

    if (p_req->header.pdu_type != LL_CONNECT_REQ)
    {
        return false;
    }

    if (p_req->payload_length != expected_payload_len)
    {
        return false;
    }

    if (p_req->header.rxadd != (m_ctrl_rt.adv_txadd & 0x01U))
    {
        return false;
    }

    return memcmp(p_req->advertiser_address, m_ctrl_rt.adv_address, sizeof(m_ctrl_rt.adv_address)) == 0;
}

void controller_adv_timer_start(uint32_t interval_ms)
{
    APP_ERROR_CHECK(app_timer_stop(m_adv_timer_id));
    APP_ERROR_CHECK(app_timer_start(m_adv_timer_id, APP_TIMER_TICKS(interval_ms), NULL));
}

void controller_adv_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_adv_timer_id));
}

static void controller_adv_timer_handler(void *p_context)
{
    uint8_t ch;
    uint32_t primask;

    (void)p_context;

    if (m_link.connected)
    {
        return;
    }

    host_build_adv_pdu();
    host_build_scan_rsp_pdu();
    /* One advertising event must cover channels 37, 38, 39 for robust discovery. */
    for (ch = 0U; ch < 3U; ch++)
    {
        (void)memset(&m_ctrl_rt.adv_rx_pdu, 0, sizeof(m_ctrl_rt.adv_rx_pdu));
        controller_reset_adv_radio_state();
        radio_set_frequency(m_adv_freq_mhz_offset[ch]);
        radio_set_whiteiv(m_adv_channels[ch]);
        radio_tx_then_rx((uint32_t)&m_ctrl_rt.adv_tx_pdu, (uint32_t)&m_ctrl_rt.adv_rx_pdu);
        m_ctrl_rt.adv_radio_phase = BLE_ADV_RADIO_PHASE_WAIT_ADV_TX_DISABLED;

        /* Keep RX open long enough for SCAN_REQ/CONNECT_REQ + margin. */
        nrf_delay_us(BLE_ADV_RX_WINDOW_US);

         primask = irq_lock();
        if (!m_link.connected)
        {
            controller_reset_adv_radio_state();
            radio_disable();
        }
        irq_unlock(primask);
    }
}

void controller_apply_peripheral_connect_request(const ble_connect_req_pdu_t *p_req)
{
    ble_gap_addr_t peer_addr = {
        .addr_is_random = (p_req->header.txadd != 0U),
    };
    uint32_t first_event_delay_us;

    (void)memcpy(peer_addr.addr, p_req->initiator_address, sizeof(peer_addr.addr));
    controller_scan_timer_stop();
    controller_adv_timer_stop();
    controller_prepare_connected_link(p_req, BLE_GAP_ROLE_PERIPHERAL, &peer_addr);

    /* First peripheral listening window starts at transmitWindowOffset; for 0 offset,
       listen immediately (with a small guard) instead of waiting a full interval. */
    first_event_delay_us = (p_req->ll_data.win_offset == 0U) ? 2000U : UNITS_1P25MS_TO_US(p_req->ll_data.win_offset);
    controller_connected_timer_start(first_event_delay_us);
    (void)ble_evt_notify_gap(BLE_GAP_EVT_CONNECTED);
}

void controller_start_connection_event_peripheral(void)
{
    controller_hop_data_channel();
    m_ctrl_rt.conn_rx_pdu.header = (ble_ll_data_header_t){0};
    m_ctrl_rt.conn_rx_pdu.length = 0U;
    m_ctrl_rt.conn_rx_process_pending = false;
    controller_reset_conn_bcmatch_state();
    m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED;
    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN);
    radio_set_bcc(BLE_LL_DATA_HEADER_BITS);
    radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_BCSTART_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk);
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.conn_rx_pdu);
    controller_set_mode_with_phy(RADIO_MODE_RX, m_link.phy.rx_phy);
}

void radio_handle_connected_packet_peripheral(void)
{
    bool new_tx_pdu;
    uint16_t current_event_counter = (uint16_t)(m_link.event_counter - 1U);

    m_link.supervision.rx_seen_this_interval = true;
    m_ctrl_rt.conn_next_event_tick_us = controller_conn_next_event_tick_from_anchor(NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX], current_event_counter);
    controller_conn_timer_schedule_compare();
    if (m_ctrl_rt.conn_bcmatch.tx_acked)
    {
        m_ctrl_rt.tx_unacked = false;
        m_link.packet.tx_sn ^= 1U;
    }

    if (m_ctrl_rt.conn_bcmatch.is_new_packet)
    {
        m_link.packet.next_expected_rx_sn ^= 1U;
    }

    m_ctrl_rt.conn_rx_process_pending = m_ctrl_rt.conn_bcmatch.is_new_packet;
    new_tx_pdu = !m_ctrl_rt.tx_unacked;
    if (new_tx_pdu && m_ctrl_rt.conn_bcmatch.consumes_pending)
    {
        m_ctrl_rt.has_pending_conn_tx_pdu = false;
    }
    controller_reset_conn_bcmatch_state();
    controller_stage_conn_response(new_tx_pdu);

    m_link.supervision.started = true;
}

void radio_handle_connected_crc_error_peripheral(void)
{
    /*
     * A CRC error carries no valid SN/NESN, so keep RX/TX sequence state unchanged
     * and answer with the current NESN to request retransmission from the central.
     */
    bool new_tx_pdu;
    uint16_t current_event_counter = (uint16_t)(m_link.event_counter - 1U);

    m_ctrl_rt.conn_next_event_tick_us = controller_conn_next_event_tick_from_anchor(NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX], current_event_counter);
    controller_conn_timer_schedule_compare();
    controller_reset_conn_bcmatch_state();
    m_ctrl_rt.conn_rx_process_pending = false;
    if (m_ctrl_rt.tx_unacked)
    {
        new_tx_pdu = false;
    }
    else
    {
        m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTINUATION);
        m_ctrl_rt.conn_tx_pdu.length = 0U;
        new_tx_pdu = true;
    }
    controller_stage_conn_response(new_tx_pdu);
    m_link.supervision.started = true;
}

void radio_handle_connected_bcmatch_peripheral(void)
{
    if (m_ctrl_rt.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    controller_prestage_conn_response_from_header();
}

void controller_handle_connected_disabled_peripheral(void)
{
    if (m_ctrl_rt.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.conn_radio_phase == BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED;
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
        return;
    }

    m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
    controller_reset_conn_bcmatch_state();
    if (m_ctrl_rt.conn_rx_process_pending)
    {
        m_ctrl_rt.conn_rx_process_pending = false;
        controller_process_received_conn_pdu();
    }
}

void controller_handle_advertising_disabled(void)
{
    if (m_ctrl_rt.adv_radio_phase == BLE_ADV_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_ADV_TX_DISABLED)
    {
        m_ctrl_rt.adv_radio_phase = BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED;
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk);
        return;
    }

    if (m_ctrl_rt.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        if (m_ctrl_rt.adv_scan_rsp_pending)
        {
            m_ctrl_rt.adv_scan_rsp_pending = false;
            m_ctrl_rt.adv_radio_phase = BLE_ADV_RADIO_PHASE_WAIT_SCAN_RSP_TX_DISABLED;
            radio_set_packet_ptr((uint32_t)&m_ctrl_rt.scan_rsp_pdu);
            radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
            return;
        }

        if (m_ctrl_rt.adv_connect_pending)
        {
            m_ctrl_rt.adv_connect_pending = false;
            controller_reset_adv_radio_state();
            radio_disable();
            controller_apply_peripheral_connect_request(&m_ctrl_rt.adv_rx_pdu.connect_req);
            return;
        }

        controller_reset_adv_radio_state();
        return;
    }

    if (m_ctrl_rt.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_SCAN_RSP_TX_DISABLED)
    {
        controller_reset_adv_radio_state();
    }
}

void controller_start_advertising_internal(void)
{
    if (m_host.configured_role != BLE_GAP_ROLE_PERIPHERAL)
    {
        return;
    }

    if (m_link.connected)
    {
        return;
    }

    controller_stop_scanning_internal();
    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_ADV);

    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.adv_tx_pdu.payload), m_adv_access_address, m_adv_crc_init, (uint32_t)&m_ctrl_rt.adv_tx_pdu);
    controller_adv_timer_start(m_host.adv_interval_ms);
}

void controller_adv_timer_init(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_adv_timer_id, APP_TIMER_MODE_REPEATED, controller_adv_timer_handler));
}
