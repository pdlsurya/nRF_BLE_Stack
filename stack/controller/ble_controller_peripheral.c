/* SPDX-License-Identifier: MIT */

#include "ble_controller_shared.h"

#include <string.h>

APP_TIMER_DEF(m_adv_timer_id);

static bool controller_peripheral_adv_type_is_scannable(void)
{
    return (m_host.peripheral.adv_type == BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED) ||
           (m_host.peripheral.adv_type == BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
}

static bool controller_peripheral_adv_type_is_connectable(void)
{
    return m_host.peripheral.adv_type == BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
}

static bool controller_peripheral_add_ad_structure(uint8_t *p_payload,
                                                   uint8_t *p_data_len,
                                                   uint8_t type,
                                                   const uint8_t *p_data,
                                                   uint8_t size)
{
    uint8_t *p_ad_data;

    if ((p_payload == NULL) || (p_data_len == NULL) || (p_data == NULL) ||
        ((uint16_t)(*p_data_len) + (uint16_t)size + 2U > BLE_LL_ADV_DATA_MAX_LEN))
    {
        return false;
    }

    p_ad_data = &p_payload[*p_data_len];
    p_ad_data[0] = (uint8_t)(size + 1U);
    p_ad_data[1] = type;
    (void)memcpy(&p_ad_data[2], p_data, size);
    *p_data_len = (uint8_t)(*p_data_len + size + 2U);

    return true;
}

static void controller_peripheral_add_device_name_ad_structure(uint8_t *p_payload,
                                                               uint8_t *p_data_len,
                                                               const ble_gap_adv_name_config_t *p_name)
{
    size_t full_name_len;
    uint8_t ad_type;
    uint8_t adv_name_len;

    if ((p_payload == NULL) || (p_data_len == NULL) || (p_name == NULL) ||
        (m_host.common.local_device_name[0] == '\0'))
    {
        return;
    }

    full_name_len = strlen(m_host.common.local_device_name);
    if (p_name->name_type == BLE_GAP_ADV_NAME_SHORT)
    {
        adv_name_len = (p_name->short_name_length < full_name_len)
                           ? p_name->short_name_length
                           : (uint8_t)full_name_len;
        ad_type = BLE_AD_TYPE_SHORT_LOCAL_NAME;
    }
    else
    {
        adv_name_len = (uint8_t)full_name_len;
        ad_type = BLE_AD_TYPE_COMPLETE_LOCAL_NAME;
    }

    if (adv_name_len == 0U)
    {
        return;
    }

    (void)controller_peripheral_add_ad_structure(p_payload,
                                                p_data_len,
                                                ad_type,
                                                (const uint8_t *)m_host.common.local_device_name,
                                                adv_name_len);
}

static bool controller_peripheral_service_uuid_list_is_128_bit(ble_gap_adv_service_uuid_list_type_t type)
{
    return (type == BLE_GAP_ADV_SERVICE_UUID_LIST_INCOMPLETE_128) ||
           (type == BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_128);
}

static bool controller_peripheral_service_uuid_list_is_complete(ble_gap_adv_service_uuid_list_type_t type)
{
    return (type == BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_16) ||
           (type == BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_128);
}

static uint8_t controller_peripheral_service_uuid_list_ad_type(ble_gap_adv_service_uuid_list_type_t type,
                                                              bool complete)
{
    if (controller_peripheral_service_uuid_list_is_128_bit(type))
    {
        return complete ? BLE_AD_TYPE_COMPLETE_UUID128_LIST
                        : BLE_AD_TYPE_INCOMPLETE_UUID128_LIST;
    }

    return complete ? BLE_AD_TYPE_COMPLETE_UUID16_LIST
                    : BLE_AD_TYPE_INCOMPLETE_UUID16_LIST;
}

static void controller_peripheral_add_service_uuid_list_ad_structure(uint8_t *p_payload,
                                                                     uint8_t *p_data_len,
                                                                     const ble_host_adv_data_t *p_adv_data,
                                                                     uint8_t list_idx)
{
    bool complete;
    bool truncated = false;
    uint8_t ad_type;
    uint8_t uuid_data_len = 0U;
    uint8_t uuid_idx;
    uint8_t uuid_len;
    uint8_t uuid_list_data[BLE_GAP_ADV_SERVICE_UUID_PER_LIST_MAX_COUNT * BLE_UUID128_LEN];
    const ble_host_adv_service_uuid_list_t *p_uuid_list;

    if ((p_payload == NULL) || (p_data_len == NULL) || (p_adv_data == NULL) ||
        (list_idx >= p_adv_data->service_uuid_list_count))
    {
        return;
    }

    p_uuid_list = &p_adv_data->service_uuid_lists[list_idx];
    complete = controller_peripheral_service_uuid_list_is_complete(p_uuid_list->type);
    uuid_len = controller_peripheral_service_uuid_list_is_128_bit(p_uuid_list->type)
                   ? BLE_UUID128_LEN
                   : BLE_UUID16_LEN;

    for (uuid_idx = 0U; uuid_idx < p_uuid_list->uuid_count; uuid_idx++)
    {
        if ((ble_uuid_encoded_len(&p_uuid_list->uuids[uuid_idx]) != uuid_len) ||
            !ble_uuid_encode(&p_uuid_list->uuids[uuid_idx], &uuid_list_data[uuid_data_len]))
        {
            truncated = true;
            continue;
        }

        if ((uint16_t)(*p_data_len) + (uint16_t)uuid_data_len + (uint16_t)uuid_len + 2U >
            BLE_LL_ADV_DATA_MAX_LEN)
        {
            truncated = true;
            break;
        }

        uuid_data_len = (uint8_t)(uuid_data_len + uuid_len);
    }

    if (uuid_data_len == 0U)
    {
        return;
    }

    ad_type = controller_peripheral_service_uuid_list_ad_type(p_uuid_list->type,
                                                             complete && !truncated);
    (void)controller_peripheral_add_ad_structure(p_payload,
                                                p_data_len,
                                                ad_type,
                                                uuid_list_data,
                                                uuid_data_len);
}

static void controller_peripheral_add_service_uuid_list_ad_structures(uint8_t *p_payload,
                                                                      uint8_t *p_data_len,
                                                                      const ble_host_adv_data_t *p_adv_data)
{
    uint8_t list_idx;

    if ((p_payload == NULL) || (p_data_len == NULL) || (p_adv_data == NULL))
    {
        return;
    }

    for (list_idx = 0U; list_idx < p_adv_data->service_uuid_list_count; list_idx++)
    {
        controller_peripheral_add_service_uuid_list_ad_structure(p_payload,
                                                                 p_data_len,
                                                                 p_adv_data,
                                                                 list_idx);
    }
}

static void controller_peripheral_add_service_data_ad_structure(uint8_t *p_payload,
                                                               uint8_t *p_data_len,
                                                               const ble_gap_adv_service_data_t *p_service_data)
{
    uint8_t uuid_bytes[BLE_UUID128_LEN];
    uint8_t service_data[BLE_UUID128_LEN + BLE_GAP_ADV_FIELD_DATA_MAX_LEN];
    uint16_t uuid_len;
    uint8_t ad_type;

    if ((p_payload == NULL) || (p_data_len == NULL) || (p_service_data == NULL))
    {
        return;
    }

    uuid_len = ble_uuid_encoded_len(&p_service_data->uuid);
    if ((uuid_len == 0U) || !ble_uuid_encode(&p_service_data->uuid, uuid_bytes))
    {
        return;
    }

    ad_type = (uuid_len == BLE_UUID128_LEN)
                  ? BLE_AD_TYPE_SERVICE_DATA_UUID128
                  : BLE_AD_TYPE_SERVICE_DATA_UUID16;
    (void)memcpy(service_data, uuid_bytes, uuid_len);
    if (p_service_data->data_len != 0U)
    {
        (void)memcpy(&service_data[uuid_len],
                     p_service_data->p_data,
                     p_service_data->data_len);
    }

    (void)controller_peripheral_add_ad_structure(p_payload,
                                                p_data_len,
                                                ad_type,
                                                service_data,
                                                (uint8_t)(uuid_len + p_service_data->data_len));
}

static void controller_peripheral_add_manufacturer_data_ad_structure(uint8_t *p_payload,
                                                                     uint8_t *p_data_len,
                                                                     const ble_gap_adv_manufacturer_data_t *p_manufacturer_data)
{
    uint8_t manufacturer_data[sizeof(p_manufacturer_data->company_id) + BLE_GAP_ADV_FIELD_DATA_MAX_LEN];

    if ((p_payload == NULL) || (p_data_len == NULL) || (p_manufacturer_data == NULL))
    {
        return;
    }

    u16_encode(p_manufacturer_data->company_id, manufacturer_data);
    if (p_manufacturer_data->data_len != 0U)
    {
        (void)memcpy(&manufacturer_data[sizeof(p_manufacturer_data->company_id)],
                     p_manufacturer_data->p_data,
                     p_manufacturer_data->data_len);
    }

    (void)controller_peripheral_add_ad_structure(p_payload,
                                                p_data_len,
                                                BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                                                manufacturer_data,
                                                (uint8_t)(sizeof(p_manufacturer_data->company_id) +
                                                          p_manufacturer_data->data_len));
}

static void controller_peripheral_add_tx_power_ad_structure(uint8_t *p_payload,
                                                            uint8_t *p_data_len,
                                                            const int8_t *p_tx_power)
{
    if (p_tx_power == NULL)
    {
        return;
    }

    (void)controller_peripheral_add_ad_structure(p_payload,
                                                p_data_len,
                                                BLE_AD_TYPE_TX_POWER_LEVEL,
                                                (const uint8_t *)p_tx_power,
                                                1U);
}

static void controller_peripheral_add_adv_data_structures(uint8_t *p_payload,
                                                          uint8_t *p_data_len,
                                                          const ble_host_adv_data_t *p_adv_data)
{
    if ((p_payload == NULL) || (p_data_len == NULL) || (p_adv_data == NULL))
    {
        return;
    }

    controller_peripheral_add_device_name_ad_structure(p_payload,
                                                       p_data_len,
                                                       p_adv_data->name_present ? &p_adv_data->name : NULL);
    controller_peripheral_add_service_uuid_list_ad_structures(p_payload,
                                                             p_data_len,
                                                             p_adv_data);
    controller_peripheral_add_service_data_ad_structure(p_payload,
                                                       p_data_len,
                                                       p_adv_data->service_data_present ? &p_adv_data->service_data : NULL);
    controller_peripheral_add_manufacturer_data_ad_structure(p_payload,
                                                            p_data_len,
                                                            p_adv_data->manufacturer_data_present ? &p_adv_data->manufacturer_data : NULL);
    controller_peripheral_add_tx_power_ad_structure(p_payload,
                                                   p_data_len,
                                                   p_adv_data->tx_power_present ? &p_adv_data->tx_power : NULL);
}

static void controller_peripheral_build_adv_pdu(void)
{
    uint8_t adv_data_len = 0U;

    (void)memset(&m_ctrl_rt.peripheral.adv_tx_pdu, 0, sizeof(m_ctrl_rt.peripheral.adv_tx_pdu));

    m_ctrl_rt.peripheral.adv_tx_pdu.header.pdu_type = (uint8_t)m_host.peripheral.adv_type;
    m_ctrl_rt.peripheral.adv_tx_pdu.header.rfu = 0U;
    m_ctrl_rt.peripheral.adv_tx_pdu.header.txadd = (uint8_t)(m_ctrl_rt.local_addr.txadd & 0x01U);
    m_ctrl_rt.peripheral.adv_tx_pdu.header.rxadd = 0U;
    (void)memcpy(m_ctrl_rt.peripheral.adv_tx_pdu.advertiser_address,
                 m_ctrl_rt.local_addr.addr,
                 sizeof(m_ctrl_rt.local_addr.addr));

    (void)controller_peripheral_add_ad_structure(m_ctrl_rt.peripheral.adv_tx_pdu.payload,
                                                &adv_data_len,
                                                BLE_AD_TYPE_FLAGS,
                                                &m_host.peripheral.flags,
                                                1U);
    controller_peripheral_add_adv_data_structures(m_ctrl_rt.peripheral.adv_tx_pdu.payload,
                                                  &adv_data_len,
                                                  &m_host.peripheral.adv_data);

    m_ctrl_rt.peripheral.adv_tx_pdu.payload_length = (uint8_t)(BLE_ADV_ADVERTISER_ADDRESS_LEN + adv_data_len);
}

static void controller_peripheral_build_scan_rsp_pdu(void)
{
    uint8_t scan_rsp_data_len = 0U;

    (void)memset(&m_ctrl_rt.peripheral.scan_rsp_pdu, 0, sizeof(m_ctrl_rt.peripheral.scan_rsp_pdu));

    m_ctrl_rt.peripheral.scan_rsp_pdu.header.pdu_type = LL_SCAN_RSP;
    m_ctrl_rt.peripheral.scan_rsp_pdu.header.rfu = 0U;
    m_ctrl_rt.peripheral.scan_rsp_pdu.header.txadd = (uint8_t)(m_ctrl_rt.local_addr.txadd & 0x01U);
    m_ctrl_rt.peripheral.scan_rsp_pdu.header.rxadd = 0U;
    (void)memcpy(m_ctrl_rt.peripheral.scan_rsp_pdu.advertiser_address,
                 m_ctrl_rt.local_addr.addr,
                 sizeof(m_ctrl_rt.peripheral.scan_rsp_pdu.advertiser_address));

    controller_peripheral_add_adv_data_structures(m_ctrl_rt.peripheral.scan_rsp_pdu.payload,
                                                  &scan_rsp_data_len,
                                                  &m_host.peripheral.scan_response_data);

    m_ctrl_rt.peripheral.scan_rsp_pdu.payload_length = (uint8_t)(BLE_ADV_ADVERTISER_ADDRESS_LEN + scan_rsp_data_len);
}

static bool controller_peripheral_scan_request_targets_us(const ble_ll_scan_req_pdu_t *p_req)
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

    if (p_req->header.rxadd != (m_ctrl_rt.local_addr.txadd & 0x01U))
    {
        return false;
    }

    return memcmp(p_req->advertiser_address,
                  m_ctrl_rt.local_addr.addr,
                  sizeof(m_ctrl_rt.local_addr.addr)) == 0;
}

static bool controller_peripheral_connect_request_targets_us(const ble_ll_connect_req_pdu_t *p_req)
{
    const uint8_t expected_payload_len = (uint8_t)(sizeof(((ble_ll_connect_req_pdu_t *)0)->initiator_address) + sizeof(((ble_ll_connect_req_pdu_t *)0)->advertiser_address) + sizeof(((ble_ll_connect_req_pdu_t *)0)->ll_data));

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

    if (p_req->header.rxadd != (m_ctrl_rt.local_addr.txadd & 0x01U))
    {
        return false;
    }

    return memcmp(p_req->advertiser_address,
                  m_ctrl_rt.local_addr.addr,
                  sizeof(m_ctrl_rt.local_addr.addr)) == 0;
}

static void controller_peripheral_adv_timer_start(uint32_t interval_ms)
{
    APP_ERROR_CHECK(app_timer_stop(m_adv_timer_id));
    APP_ERROR_CHECK(app_timer_start(m_adv_timer_id, APP_TIMER_TICKS(interval_ms), NULL));
}

static void controller_peripheral_adv_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_adv_timer_id));
}

static void controller_peripheral_reset_adv_state(void)
{
    m_ctrl_rt.peripheral.scan_rsp_tx_pending = false;
    m_ctrl_rt.peripheral.connect_req_pending = false;
    m_ctrl_rt.peripheral.adv_radio_phase = BLE_ADV_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
}

void controller_peripheral_state_reset(void)
{
    controller_peripheral_reset_adv_state();
}

static void controller_peripheral_handle_adv_crc_ok(const ble_ll_adv_req_pdu_t *p_adv_rx)
{
    if (p_adv_rx == NULL)
    {
        return;
    }

    if (controller_peripheral_adv_type_is_scannable() &&
        (p_adv_rx->scan_req.header.pdu_type == LL_SCAN_REQ) &&
        controller_peripheral_scan_request_targets_us(&p_adv_rx->scan_req))
    {
        m_ctrl_rt.peripheral.scan_rsp_pdu.header.rxadd = (uint8_t)(p_adv_rx->scan_req.header.txadd & 0x01U);
        m_ctrl_rt.peripheral.scan_rsp_tx_pending = true;
        return;
    }

    if (controller_peripheral_adv_type_is_connectable() &&
        (p_adv_rx->connect_req.header.pdu_type == LL_CONNECT_REQ) &&
        controller_peripheral_connect_request_targets_us(&p_adv_rx->connect_req))
    {
        m_ctrl_rt.peripheral.connect_req_pending = true;
    }
}

static void controller_peripheral_adv_timer_handler(void *p_context)
{
    uint8_t ch;
    uint32_t primask;

    (void)p_context;

    if (m_link.connected)
    {
        return;
    }

    controller_peripheral_build_adv_pdu();
    controller_peripheral_build_scan_rsp_pdu();
    /* One advertising event must cover channels 37, 38, 39 for robust discovery. */
    for (ch = 0U; ch < 3U; ch++)
    {
        (void)memset(&m_ctrl_rt.peripheral.adv_rx_pdu, 0, sizeof(m_ctrl_rt.peripheral.adv_rx_pdu));
        controller_peripheral_reset_adv_state();
        radio_set_frequency(m_adv_freq_mhz_offset[ch]);
        radio_set_whiteiv(m_adv_channels[ch]);
        radio_tx_then_rx((uint32_t)&m_ctrl_rt.peripheral.adv_tx_pdu, (uint32_t)&m_ctrl_rt.peripheral.adv_rx_pdu);
        m_ctrl_rt.peripheral.adv_radio_phase = BLE_ADV_RADIO_PHASE_WAIT_ADV_TX_DISABLED;

        /* Keep RX open long enough for SCAN_REQ/CONNECT_REQ + margin. */
        nrf_delay_us(BLE_ADV_RX_WINDOW_US);

        primask = irq_lock();
        if (!m_link.connected)
        {
            controller_peripheral_reset_adv_state();
            radio_disable();
        }
        irq_unlock(primask);
    }
}

static void controller_peripheral_reset_conn_bcmatch_state(void)
{
    m_ctrl_rt.conn.conn_bcmatch.tx_acked = false;
    m_ctrl_rt.conn.conn_bcmatch.is_new_packet = false;
    controller_reset_conn_tx_selection_state();
}

static void controller_peripheral_prestage_conn_response_from_header(void)
{
    const ble_ll_data_header_t *p_rx_header = &m_ctrl_rt.conn.conn_rx_pdu.header;
    bool tx_acked = m_ctrl_rt.conn.tx_unacked && (p_rx_header->nesn != m_link.packet.tx_sn);
    bool is_new_packet = (p_rx_header->sn == m_link.packet.next_expected_rx_sn);
    uint8_t next_expected_rx_sn;
    uint8_t tx_sn;

    controller_peripheral_reset_conn_bcmatch_state();
    m_ctrl_rt.conn.conn_bcmatch.tx_acked = tx_acked;
    m_ctrl_rt.conn.conn_bcmatch.is_new_packet = is_new_packet;

    if (m_ctrl_rt.conn.tx_unacked && !tx_acked)
    {
        return;
    }

    next_expected_rx_sn = m_link.packet.next_expected_rx_sn;
    tx_sn = m_link.packet.tx_sn;

    if (tx_acked)
    {
        tx_sn ^= 1U;
    }

    if (is_new_packet)
    {
        next_expected_rx_sn ^= 1U;
    }

    if (!controller_load_pending_conn_tx_pdu_for_state(next_expected_rx_sn, tx_sn))
    {
        m_ctrl_rt.conn.conn_tx_pdu.header = controller_conn_header_for_state(BLE_LLID_CONTINUATION, next_expected_rx_sn, tx_sn);
        m_ctrl_rt.conn.conn_tx_pdu.length = 0U;
    }
}

static void controller_peripheral_apply_connect_request(const ble_ll_connect_req_pdu_t *p_req)
{
    ble_gap_addr_t peer_addr = {
        .is_random = (p_req->header.txadd != 0U),
    };
    uint32_t first_event_delay_us;

    (void)memcpy(peer_addr.addr, p_req->initiator_address, sizeof(peer_addr.addr));
    controller_peripheral_adv_timer_stop();
    controller_prepare_connected_link(p_req, BLE_GAP_ROLE_PERIPHERAL, &peer_addr);

    /* First peripheral listening window starts at transmitWindowOffset; for 0 offset,
       listen immediately (with a small guard) instead of waiting a full interval. */
    first_event_delay_us = (p_req->ll_data.win_offset == 0U) ? 2000U : UNITS_1P25MS_TO_US(p_req->ll_data.win_offset);
    controller_connected_timer_start(first_event_delay_us);
    (void)ble_evt_notify_gap(BLE_GAP_EVT_CONNECTED);
}

void controller_peripheral_start_connection_event(void)
{
    controller_hop_data_channel();
    m_ctrl_rt.conn.conn_rx_pdu.header = (ble_ll_data_header_t){0};
    m_ctrl_rt.conn.conn_rx_pdu.length = 0U;
    m_ctrl_rt.conn.conn_rx_process_pending = false;
    controller_peripheral_reset_conn_bcmatch_state();
    m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED;
    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN_PERIPHERAL);
    radio_set_bcc(BLE_LL_DATA_HEADER_BITS);
    radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_BCSTART_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk);
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.conn.conn_rx_pdu);
    controller_set_mode_with_phy(RADIO_MODE_RX, m_link.phy.rx_phy);
}

static void controller_peripheral_handle_connected_packet(void)
{
    bool new_tx_pdu;
    uint16_t current_event_counter = (uint16_t)(m_link.event_counter - 1U);

    m_link.supervision.rx_seen_this_interval = true;
    m_ctrl_rt.conn.conn_next_event_tick_us = controller_conn_next_event_tick_from_anchor(NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX], current_event_counter);
    controller_conn_timer_schedule_compare();
    if (m_ctrl_rt.conn.conn_bcmatch.tx_acked)
    {
        m_ctrl_rt.conn.tx_unacked = false;
        m_link.packet.tx_sn ^= 1U;
    }

    if (m_ctrl_rt.conn.conn_bcmatch.is_new_packet)
    {
        m_link.packet.next_expected_rx_sn ^= 1U;
    }

    m_ctrl_rt.conn.conn_rx_process_pending = m_ctrl_rt.conn.conn_bcmatch.is_new_packet;
    new_tx_pdu = !m_ctrl_rt.conn.tx_unacked;
    controller_stage_conn_response(new_tx_pdu);
    controller_peripheral_reset_conn_bcmatch_state();

    m_link.supervision.started = true;
}

static void controller_peripheral_handle_connected_crc_error(void)
{
    /*
     * A CRC error carries no valid SN/NESN, so keep RX/TX sequence state unchanged
     * and answer with the current NESN to request retransmission from the central.
     */
    bool new_tx_pdu;
    uint16_t current_event_counter = (uint16_t)(m_link.event_counter - 1U);

    m_ctrl_rt.conn.conn_next_event_tick_us = controller_conn_next_event_tick_from_anchor(NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX], current_event_counter);
    controller_conn_timer_schedule_compare();
    controller_peripheral_reset_conn_bcmatch_state();
    m_ctrl_rt.conn.conn_rx_process_pending = false;
    if (m_ctrl_rt.conn.tx_unacked)
    {
        new_tx_pdu = false;
    }
    else
    {
        m_ctrl_rt.conn.conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTINUATION);
        m_ctrl_rt.conn.conn_tx_pdu.length = 0U;
        new_tx_pdu = true;
    }
    controller_stage_conn_response(new_tx_pdu);
    m_link.supervision.started = true;
}

static void controller_peripheral_handle_connected_bcmatch(void)
{
    if (m_ctrl_rt.conn.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    controller_peripheral_prestage_conn_response_from_header();
}

static void controller_peripheral_handle_connected_disabled(void)
{
    if (m_ctrl_rt.conn.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.conn.conn_radio_phase == BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED;
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
        return;
    }

    m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
    controller_peripheral_reset_conn_bcmatch_state();
    if (m_ctrl_rt.conn.conn_rx_process_pending)
    {
        m_ctrl_rt.conn.conn_rx_process_pending = false;
        controller_process_received_conn_pdu();
    }
}

static void controller_peripheral_handle_adv_disabled(void)
{
    if (m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_ADV_TX_DISABLED)
    {
        m_ctrl_rt.peripheral.adv_radio_phase = BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED;
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk);
        return;
    }

    if (m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        if (m_ctrl_rt.peripheral.scan_rsp_tx_pending)
        {
            m_ctrl_rt.peripheral.scan_rsp_tx_pending = false;
            m_ctrl_rt.peripheral.adv_radio_phase = BLE_ADV_RADIO_PHASE_WAIT_SCAN_RSP_TX_DISABLED;
            radio_set_packet_ptr((uint32_t)&m_ctrl_rt.peripheral.scan_rsp_pdu);
            radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
            return;
        }

        if (m_ctrl_rt.peripheral.connect_req_pending)
        {
            m_ctrl_rt.peripheral.connect_req_pending = false;
            controller_peripheral_reset_adv_state();
            radio_disable();
            controller_peripheral_apply_connect_request(&m_ctrl_rt.peripheral.adv_rx_pdu.connect_req);
            return;
        }

        controller_peripheral_reset_adv_state();
        return;
    }

    if (m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_SCAN_RSP_TX_DISABLED)
    {
        controller_peripheral_reset_adv_state();
    }
}

void controller_peripheral_handle_radio_event(radio_event_t evt, const ble_ll_adv_req_pdu_t *p_adv_rx)
{
    if (m_link.connected)
    {
        if (evt == RADIO_EVENT_BCMATCH)
        {
            controller_peripheral_handle_connected_bcmatch();
        }
        else if (evt == RADIO_EVENT_CRC_OK)
        {
            controller_peripheral_handle_connected_packet();
        }
        else if (evt == RADIO_EVENT_CRC_ERROR)
        {
            controller_peripheral_handle_connected_crc_error();
        }
        else if (evt == RADIO_EVENT_DISABLED)
        {
            controller_peripheral_handle_connected_disabled();
        }
        return;
    }

    if (m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (evt == RADIO_EVENT_DISABLED)
    {
        controller_peripheral_handle_adv_disabled();
        return;
    }

    if ((m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED) &&
        (evt == RADIO_EVENT_CRC_OK))
    {
        controller_peripheral_handle_adv_crc_ok(p_adv_rx);
    }
}

void controller_peripheral_start_advertising_internal(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL))
    {
        return;
    }

    if (m_link.connected)
    {
        return;
    }

    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_ADV);

    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.peripheral.adv_tx_pdu.payload),
                                    m_adv_access_address,
                                    m_adv_crc_init,
                                    (uint32_t)&m_ctrl_rt.peripheral.adv_tx_pdu);
    controller_peripheral_adv_timer_start(m_host.peripheral.adv_interval_ms);
}

void controller_peripheral_adv_timer_init(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_adv_timer_id, APP_TIMER_MODE_REPEATED, controller_peripheral_adv_timer_handler));
}
