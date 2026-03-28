/**
 * @file ble_controller.c
 * @author Surya Poudel
 * @brief BLE controller and link-layer implementation for nRF
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_internal.h"

#include <string.h>

#include "app_error.h"
#include "ble_gatt_server.h"
#include "nrf_delay.h"

APP_TIMER_DEF(m_adv_timer_id);

static void ble_advertise(void);
static void controller_start_connection_event(void);
static void controller_apply_pending_link_updates(void);

static void controller_conn_timer_schedule_compare(void)
{
    uint32_t compare_tick = m_ctrl_rt.conn_next_event_tick_us;

    if (compare_tick > BLE_CONN_EVENT_GUARD_US)
    {
        compare_tick -= BLE_CONN_EVENT_GUARD_US;
    }
    else
    {
        compare_tick = 1U;
    }

    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;
    NRF_TIMER2->CC[0] = compare_tick;
}

static void controller_conn_timer_init(void)
{
    if (m_ctrl_rt.conn_timer_initialized)
    {
        return;
    }

    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER2->PRESCALER = BLE_CONN_TIMER_PRESCALER;
    NRF_TIMER2->SHORTS = 0U;
    NRF_TIMER2->INTENCLR = 0xFFFFFFFFUL;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;

    NVIC_ClearPendingIRQ(TIMER2_IRQn);
    NVIC_SetPriority(TIMER2_IRQn, BLE_CONN_TIMER_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIMER2_IRQn);
    m_ctrl_rt.conn_timer_initialized = true;
}

static void controller_conn_timer_stop(void)
{
    NRF_TIMER2->TASKS_STOP = 1U;
    NRF_TIMER2->TASKS_CLEAR = 1U;
    NRF_TIMER2->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;
    m_ctrl_rt.conn_next_event_tick_us = 0U;
}

static void controller_conn_timer_start(uint32_t first_event_delay_us)
{
    controller_conn_timer_init();
    controller_conn_timer_stop();
    m_ctrl_rt.conn_next_event_tick_us = first_event_delay_us;
    controller_conn_timer_schedule_compare();
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER2->TASKS_START = 1U;
}

static void controller_prepare_radio_common(uint8_t max_payload_size,
                                            const uint8_t *p_address,
                                            uint32_t crc_init,
                                            uint32_t packet_ptr)
{
    radio_reset_packet_config();
    radio_set_len_field_size(8U);
    radio_set_s0_field_size(1U);
    radio_set_max_payload_size(max_payload_size);
    radio_set_address_width(4U);
    radio_set_payload_endian(RADIO_LITTLE_ENDIAN);
    radio_enable_whitening(true);
    radio_set_address(p_address, 4U, 0U);
    radio_set_tx_logical_address(0U);
    radio_set_rx_logical_address(0U);
    radio_set_data_rate(RADIO_1MBPS);
    radio_set_tx_power((uint32_t)(uint8_t)m_host.tx_power);
    radio_configure_crc(3U, 1U, m_ble_crc_poly, crc_init);
    radio_set_packet_ptr(packet_ptr);
}

void TIMER2_IRQHandler(void)
{
    if (NRF_TIMER2->EVENTS_COMPARE[0] == 0U)
    {
        return;
    }

    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;

    if (!m_link.connected)
    {
        return;
    }

    if (m_link.conn_interval_us != 0U)
    {
        m_ctrl_rt.conn_next_event_tick_us += m_link.conn_interval_us;
        controller_conn_timer_schedule_compare();
    }

    if (m_link.supervision_started)
    {
        if (m_link.rx_seen_this_interval)
        {
            m_link.missed_interval_count = 0U;
        }
        else if (m_link.missed_interval_count < 0xFFFFU)
        {
            m_link.missed_interval_count++;
            if (((uint32_t)m_link.missed_interval_count * (uint32_t)m_link.conn_interval_ms) >=
                (uint32_t)m_link.supervision_timeout_ms)
            {
                (void)ble_evt_notify_gap(BLE_GAP_EVT_SUPERVISION_TIMEOUT);
                controller_disconnect_internal();
                return;
            }
        }
    }

    m_link.rx_seen_this_interval = false;
    m_link.event_anchor_captured = false;
    controller_apply_pending_link_updates();
    controller_start_connection_event();
    if (m_link.event_counter < 0xFFFFU)
    {
        m_link.event_counter++;
    }
}
static void controller_set_adv_channel(uint8_t channel_idx)
{
    uint8_t adv_channel_index = (uint8_t)(channel_idx % 3U);

    radio_set_frequency(m_adv_freq_mhz_offset[adv_channel_index]);
    radio_set_whiteiv(m_adv_channels[adv_channel_index]);
}

static void controller_set_data_channel(uint8_t channel)
{
    if (channel >= 37U)
    {
        channel = 0U;
    }

    radio_set_frequency(m_data_channel_freq[channel]);
    radio_set_whiteiv(channel);
}

static bool controller_channel_used(uint8_t channel)
{
    uint8_t b = (uint8_t)(channel >> 3);
    uint8_t m = (uint8_t)(1U << (channel & 0x07U));
    return ((m_link.channel_map[b] & m) != 0U);
}

static uint8_t controller_build_used_channels(const uint8_t *p_channel_map, uint8_t *p_channels)
{
    uint8_t i;
    uint8_t count = 0U;

    if ((p_channel_map == NULL) || (p_channels == NULL))
    {
        return 0U;
    }

    for (i = 0U; i < 37U; i++)
    {
        if ((p_channel_map[i >> 3] & (uint8_t)(1U << (i & 0x07U))) != 0U)
        {
            p_channels[count++] = i;
        }
    }

    if (count == 0U)
    {
        for (i = 0U; i < 37U; i++)
        {
            p_channels[i] = i;
        }
        count = 37U;
    }

    return count;
}

static void controller_apply_channel_map(const uint8_t *p_channel_map)
{
    if (p_channel_map == NULL)
    {
        return;
    }

    (void)memcpy(m_link.channel_map, p_channel_map, sizeof(m_link.channel_map));
    m_link.channel_count = controller_build_used_channels(m_link.channel_map, m_link.channels);
}

static void controller_apply_pending_link_updates(void)
{
    if (m_link.pending_channel_map_valid &&
        (m_link.event_counter == m_link.pending_channel_map_instant))
    {
        controller_apply_channel_map(m_link.pending_channel_map);
        m_link.pending_channel_map_valid = false;
    }
}

static void controller_hop_data_channel(void)
{
    uint8_t unmapped;
    uint8_t mapped;

    if ((m_link.channel_count == 0U) || (m_link.hop_increment == 0U))
    {
        controller_set_data_channel(0U);
        return;
    }

    unmapped = (uint8_t)((m_link.last_unmapped_channel + m_link.hop_increment) % 37U);
    m_link.last_unmapped_channel = unmapped;

    if (controller_channel_used(unmapped))
    {
        mapped = unmapped;
    }
    else
    {
        mapped = m_link.channels[unmapped % m_link.channel_count];
    }

    m_link.current_channel_index = mapped;
    controller_set_data_channel(mapped);
}

static void controller_reset_tx_runtime(void)
{
    m_ctrl_rt.has_last_conn_tx_pdu = false;
    m_ctrl_rt.tx_unacked = false;
    m_ctrl_rt.has_pending_conn_tx_pdu = false;
}

static ble_ll_data_header_t controller_conn_header(uint8_t llid)
{
    ble_ll_data_header_t header = {
        .llid = llid,
        .nesn = (uint8_t)(m_link.rx_sn & 0x01U),
        .sn = (uint8_t)(m_link.tx_sn & 0x01U),
        .md = 0U,
        .rfu = 0U,
    };

    return header;
}

static void controller_queue_current_tx_pdu(void)
{
    uint32_t primask = irq_lock();

    m_ctrl_rt.pending_conn_tx_pdu = m_ctrl_rt.conn_tx_pdu;
    m_ctrl_rt.has_pending_conn_tx_pdu = true;
    irq_unlock(primask);
}

static bool host_add_ad_structure(uint8_t *p_adv_data_len,
                                  uint8_t type,
                                  const uint8_t *p_data,
                                  uint8_t size)
{
    uint8_t *p_adv_data;

    if ((p_adv_data_len == NULL) || (p_data == NULL) || (size == 0U))
    {
        return false;
    }
    if ((uint16_t)(*p_adv_data_len) + (uint16_t)size + 2U > BLE_MAX_ADV_DATA_LEN)
    {
        return false;
    }

    p_adv_data = &m_ctrl_rt.air_pdu.payload[*p_adv_data_len];
    p_adv_data[0] = (uint8_t)(size + 1U);
    p_adv_data[1] = type;
    (void)memcpy(&p_adv_data[2], p_data, size);
    *p_adv_data_len = (uint8_t)(*p_adv_data_len + size + 2U);

    return true;
}

static void host_build_adv_pdu(void)
{
    uint8_t adv_data_len = 0U;
    uint16_t uuid_le;
    uint8_t service_data_raw[3];
    uint8_t flags;

    (void)memset(&m_ctrl_rt.air_pdu, 0, sizeof(m_ctrl_rt.air_pdu));

    m_ctrl_rt.air_pdu.header.pdu_type = LL_ADV_IND;
    m_ctrl_rt.air_pdu.header.rfu = 0U;
    m_ctrl_rt.air_pdu.header.txadd = (uint8_t)(m_ctrl_rt.adv_txadd & 0x01U);
    m_ctrl_rt.air_pdu.header.rxadd = 0U;
    (void)memcpy(m_ctrl_rt.air_pdu.mac_address,
                 m_ctrl_rt.adv_address,
                 sizeof(m_ctrl_rt.adv_address));

    flags = m_host.flags;
    (void)host_add_ad_structure(&adv_data_len, 0x01U, &flags, 1U);

    if (m_host.adv_name_len != 0U)
    {
        (void)host_add_ad_structure(&adv_data_len, 0x09U, (const uint8_t *)m_host.adv_name, m_host.adv_name_len);
    }

    (void)host_add_ad_structure(&adv_data_len, 0x0AU, (const uint8_t *)&m_host.tx_power, 1U);

    if (m_host.included_service_uuid != 0U)
    {
        uuid_le = m_host.included_service_uuid;
        (void)host_add_ad_structure(&adv_data_len, 0x03U, (const uint8_t *)&uuid_le, sizeof(uuid_le));
    }

    if (m_host.has_service_data)
    {
        uuid_le = m_host.service_data.uuid;
        service_data_raw[0] = (uint8_t)(uuid_le & 0xFFU);
        service_data_raw[1] = (uint8_t)((uuid_le >> 8) & 0xFFU);
        service_data_raw[2] = m_host.service_data.data;
        (void)host_add_ad_structure(&adv_data_len, 0x16U, service_data_raw, sizeof(service_data_raw));
    }

    m_ctrl_rt.air_pdu.payload_length = (uint8_t)(BLE_ADV_PDU_OVERHEAD + adv_data_len);
}

static void controller_prepare_radio_for_adv(void)
{
    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.air_pdu.payload),
                                    m_adv_access_address,
                                    m_adv_crc_init,
                                    (uint32_t)&m_ctrl_rt.air_pdu);
    controller_set_adv_channel(0U);
}

static void controller_prepare_radio_for_connection(void)
{
    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.conn_rx_pdu.payload),
                                    m_link.access_address,
                                    m_link.crc_init,
                                    (uint32_t)&m_ctrl_rt.conn_rx_pdu);
    controller_set_data_channel(m_link.channels[m_link.current_channel_index]);
}

void controller_disconnect_internal(void)
{
    bool was_connected = m_link.connected;

    m_link.connected = false;
    m_link.conn_interval_us = 0U;
    controller_reset_tx_runtime();
    m_link.event_anchor_captured = false;
    m_link.pending_channel_map_valid = false;
    m_link.pending_channel_map_instant = 0U;
    controller_conn_timer_stop();
    ble_gatt_server_reset_connection_state();
    radio_disable();
    radio_set_shorts(0U);

    if (was_connected)
    {
        (void)ble_evt_notify_gap(BLE_GAP_EVT_DISCONNECTED);
    }
}

static uint16_t ll_control_process(const uint8_t *p_payload, uint8_t len, uint8_t *p_rsp)
{
    uint8_t opcode;

    if ((p_payload == NULL) || (len == 0U))
    {
        return 0U;
    }

    opcode = p_payload[0];
    switch (opcode)
    {
    case BLE_LL_CTRL_CONN_UPDATE_IND:
        (void)ble_evt_notify_gap(BLE_GAP_EVT_CONN_UPDATE_IND);
        return 0U;

    case BLE_LL_CTRL_CHANNEL_MAP_IND:
        if (len >= 8U)
        {
            (void)memcpy(m_link.pending_channel_map, &p_payload[1], sizeof(m_link.pending_channel_map));
            m_link.pending_channel_map_instant = u16_decode(&p_payload[6]);
            m_link.pending_channel_map_valid = true;
        }
        return 0U;

    case BLE_LL_CTRL_FEATURE_REQ:
    case BLE_LL_CTRL_SLV_FEATURE_REQ:
        p_rsp[0] = BLE_LL_CTRL_FEATURE_RSP;
        (void)memset(&p_rsp[1], 0, 8U);
        return 9U;

    case BLE_LL_CTRL_VERSION_IND:
        p_rsp[0] = BLE_LL_CTRL_VERSION_IND;
        p_rsp[1] = BLE_LL_VERSION_5_0;
        u16_encode(BLE_LL_COMPANY_ID_NORDIC, &p_rsp[2]);
        u16_encode(BLE_LL_SUBVERSION, &p_rsp[4]);
        return 6U;

    case BLE_LL_CTRL_LENGTH_REQ:
        p_rsp[0] = BLE_LL_CTRL_LENGTH_RSP;
        u16_encode(27U, &p_rsp[1]);
        u16_encode(328U, &p_rsp[3]);
        u16_encode(27U, &p_rsp[5]);
        u16_encode(328U, &p_rsp[7]);
        return 9U;

    case BLE_LL_CTRL_UNKNOWN_RSP:
        return 0U;

    case BLE_LL_CTRL_PHY_REQ:
        p_rsp[0] = BLE_LL_CTRL_PHY_RSP;
        p_rsp[1] = 0x01U;
        p_rsp[2] = 0x01U;
        return 3U;

    case BLE_LL_CTRL_TERMINATE_IND:
        (void)ble_evt_notify_gap(BLE_GAP_EVT_TERMINATE_IND);
        controller_disconnect_internal();
        return 0U;

    default:
        p_rsp[0] = BLE_LL_CTRL_UNKNOWN_RSP;
        p_rsp[1] = opcode;
        return 2U;
    }
}

static void controller_build_empty_pdu(void)
{
    m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTINUATION);
    m_ctrl_rt.conn_tx_pdu.length = 0U;
}

static void controller_build_control_pdu(const uint8_t *p_payload, uint8_t payload_len)
{
    m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTROL_PDU);
    m_ctrl_rt.conn_tx_pdu.length = payload_len;
    (void)memcpy(m_ctrl_rt.conn_tx_pdu.payload, p_payload, payload_len);
}

static void controller_build_att_pdu(const uint8_t *p_att_payload, uint16_t att_len)
{
    m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(BLE_LLID_START_L2CAP);
    m_ctrl_rt.conn_tx_pdu.length = (uint8_t)(att_len + BLE_L2CAP_HDR_LEN);
    u16_encode(att_len, &m_ctrl_rt.conn_tx_pdu.payload[0]);
    u16_encode(BLE_L2CAP_CID_ATT, &m_ctrl_rt.conn_tx_pdu.payload[2]);
    (void)memcpy(&m_ctrl_rt.conn_tx_pdu.payload[BLE_L2CAP_HDR_LEN], p_att_payload, att_len);
}

bool controller_queue_att_payload(const uint8_t *p_att_payload, uint16_t att_len)
{
    uint32_t primask;

    if ((p_att_payload == NULL) || (att_len == 0U) || !m_link.connected)
    {
        return false;
    }

    if ((uint16_t)(att_len + BLE_L2CAP_HDR_LEN) > sizeof(m_ctrl_rt.conn_tx_pdu.payload))
    {
        return false;
    }

    primask = irq_lock();

    if (m_ctrl_rt.has_pending_conn_tx_pdu)
    {
        irq_unlock(primask);
        return false;
    }

    controller_build_att_pdu(p_att_payload, att_len);
    controller_queue_current_tx_pdu();
    irq_unlock(primask);

    return true;
}

static void controller_prepare_pending_pdu_for_tx(void)
{
    m_ctrl_rt.conn_tx_pdu.header = controller_conn_header(m_ctrl_rt.pending_conn_tx_pdu.header.llid);
    m_ctrl_rt.conn_tx_pdu.length = m_ctrl_rt.pending_conn_tx_pdu.length;
    if (m_ctrl_rt.pending_conn_tx_pdu.length > 0U)
    {
        (void)memcpy(m_ctrl_rt.conn_tx_pdu.payload,
                     m_ctrl_rt.pending_conn_tx_pdu.payload,
                     m_ctrl_rt.pending_conn_tx_pdu.length);
    }
}

static void controller_send_conn_response(bool new_tx_pdu)
{
    uint32_t tx_ptr;

    if (!m_link.connected)
    {
        return;
    }

    if ((!new_tx_pdu) && m_ctrl_rt.has_last_conn_tx_pdu)
    {
        tx_ptr = (uint32_t)&m_ctrl_rt.last_conn_tx_pdu;
    }
    else
    {
        tx_ptr = (uint32_t)&m_ctrl_rt.conn_tx_pdu;
    }

    /* RX phase uses END->DISABLE. Wait until radio reaches DISABLED first. */
    radio_wait_disabled();
    radio_clear_disabled_event();

    /*
     * The peer listens for the slave response after the 150 us BLE inter-frame
     * spacing. With fast ramp-up enabled, hold TXEN briefly so START lands in
     * that listening window instead of too early.
     */
    nrf_delay_us(BLE_CONN_TXEN_DELAY_US);

    /* TX with READY->START and END->DISABLE for deterministic turnaround. */
    radio_set_shorts(RADIO_SHORTS_READY_START_Msk |
                     RADIO_SHORTS_END_DISABLE_Msk);
    radio_clear_ready_event();
    radio_clear_end_event();
    radio_set_packet_ptr(tx_ptr);
    radio_tx_enable();
    radio_wait_end();
    radio_clear_end_event();
    radio_wait_disabled();
    radio_clear_disabled_event();

    /* Back to RX; keep END->DISABLE for the next RX packet turn-around. */
    radio_set_shorts(RADIO_SHORTS_END_DISABLE_Msk);
    m_ctrl_rt.conn_rx_pdu.header = (ble_ll_data_header_t){0};
    m_ctrl_rt.conn_rx_pdu.length = 0U;
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.conn_rx_pdu);
    radio_rx_enable();
    radio_wait_ready();
    radio_clear_ready_event();
    radio_start();

    if (new_tx_pdu)
    {
        m_ctrl_rt.last_conn_tx_pdu = m_ctrl_rt.conn_tx_pdu;
        m_ctrl_rt.has_last_conn_tx_pdu = true;
        m_ctrl_rt.tx_unacked = true;
    }
}

static void controller_start_connection_event(void)
{
    if (!m_link.connected)
    {
        return;
    }

    controller_hop_data_channel();
    radio_disable();
    radio_set_shorts(RADIO_SHORTS_END_DISABLE_Msk);
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.conn_rx_pdu);
    radio_set_mode(MODE_RX);
    radio_rx();
}

static void adv_timer_handler(void *p_context)
{
    (void)p_context;
    ble_advertise();
}

static void controller_apply_connect_request(const ble_connect_req_pdu_t *p_req)
{
    uint32_t first_event_delay_us;

    if (p_req == NULL)
    {
        return;
    }

    (void)memset(&m_link, 0, sizeof(m_link));
    m_link.connected = true;
    m_link.conn_interval_ms = (uint16_t)BLE_1P25MS_UNITS_TO_MS(p_req->ll_data.interval);
    m_link.conn_interval_us = (uint32_t)p_req->ll_data.interval * 1250U;
    m_link.supervision_timeout_ms = (uint16_t)BLE_10MS_UNITS_TO_MS(p_req->ll_data.timeout);
    m_link.hop_increment = p_req->ll_data.hop_increment;
    if (m_link.hop_increment == 0U)
    {
        m_link.hop_increment = 5U;
    }
    m_link.crc_init = (uint32_t)p_req->ll_data.crc_init[0] |
                      ((uint32_t)p_req->ll_data.crc_init[1] << 8) |
                      ((uint32_t)p_req->ll_data.crc_init[2] << 16);
    controller_reset_tx_runtime();
    ble_gatt_server_reset_connection_state();

    (void)memcpy(m_link.access_address, p_req->ll_data.access_address, sizeof(m_link.access_address));
    controller_apply_channel_map(p_req->ll_data.channel_map);

    (void)app_timer_stop(m_adv_timer_id);
    m_controller.advertising = false;
    controller_prepare_radio_for_connection();

    /* First slave listening window starts at transmitWindowOffset; for 0 offset,
       listen immediately (with a small guard) instead of waiting a full interval. */
    first_event_delay_us = (p_req->ll_data.win_offset == 0U) ? 2000U : ((uint32_t)p_req->ll_data.win_offset * 1250U);
    controller_conn_timer_start(first_event_delay_us);

    (void)ble_evt_notify_gap(BLE_GAP_EVT_CONNECTED);
}

static void radio_handle_connected_packet(void)
{
    ble_ll_data_raw_pdu_t rx_pdu = m_ctrl_rt.conn_rx_pdu;
    bool is_new_packet = (rx_pdu.header.sn == m_link.rx_sn);
    bool tx_needs_retransmit = false;
    uint8_t ctrl_rsp[12];
    uint16_t ctrl_rsp_len = 0U;
    bool has_att_rsp = false;
    uint16_t att_rsp_len = 0U;
    uint8_t att_rsp[BLE_ATT_GATT_MAX_MTU];

    if (is_new_packet)
    {
        m_link.rx_sn ^= 1U;
    }

    if (!m_link.event_anchor_captured && (m_link.conn_interval_us != 0U))
    {
        NRF_TIMER2->TASKS_CAPTURE[1] = 1U;
        m_ctrl_rt.conn_next_event_tick_us = NRF_TIMER2->CC[1] + m_link.conn_interval_us;
        controller_conn_timer_schedule_compare();
        m_link.event_anchor_captured = true;
    }

    m_link.rx_seen_this_interval = true;

    if (!is_new_packet)
    {
        controller_send_conn_response(false);
        return;
    }

    /*
     * transmitSeqNum advances only after the peer acknowledges the last PDU.
     * Per the spec, a received NESN different from transmitSeqNum means ACK.
     */
    if (m_ctrl_rt.tx_unacked && (rx_pdu.header.nesn != m_link.tx_sn))
    {
        m_ctrl_rt.tx_unacked = false;
        m_link.tx_sn ^= 1U;
    }

    tx_needs_retransmit = m_ctrl_rt.tx_unacked && m_ctrl_rt.has_last_conn_tx_pdu;

    /*
     * Respond first to stay within the RX->TX turn-around budget. Any control/ATT
     * response derived from this packet is prepared for the next connection event.
     */
    if (tx_needs_retransmit)
    {
        controller_send_conn_response(false);
    }
    else
    {
        if (m_ctrl_rt.has_pending_conn_tx_pdu)
        {
            controller_prepare_pending_pdu_for_tx();
            m_ctrl_rt.has_pending_conn_tx_pdu = false;
        }
        else
        {
            controller_build_empty_pdu();
        }
        controller_send_conn_response(true);
    }

    if (!m_link.connected)
    {
        return;
    }

    if (!m_link.supervision_started)
    {
        m_link.supervision_started = true;
    }

    if (rx_pdu.header.llid == BLE_LLID_CONTROL_PDU)
    {
        if (rx_pdu.length == 0U)
        {
            return;
        }

        ctrl_rsp_len = ll_control_process(rx_pdu.payload, rx_pdu.length, ctrl_rsp);
        if ((ctrl_rsp_len > 0U) && m_link.connected)
        {
            controller_build_control_pdu(ctrl_rsp, (uint8_t)ctrl_rsp_len);
            controller_queue_current_tx_pdu();
        }
        return;
    }

    if (rx_pdu.length == 0U)
    {
        return;
    }

    if ((rx_pdu.header.llid == BLE_LLID_START_L2CAP) ||
        (rx_pdu.header.llid == BLE_LLID_CONTINUATION))
    {
        if (rx_pdu.length >= BLE_L2CAP_HDR_LEN)
        {
            uint16_t l2cap_len = u16_decode(&rx_pdu.payload[0]);
            uint16_t l2cap_cid = u16_decode(&rx_pdu.payload[2]);

            if ((l2cap_cid == BLE_L2CAP_CID_ATT) &&
                ((uint16_t)rx_pdu.length >= (uint16_t)(l2cap_len + BLE_L2CAP_HDR_LEN)))
            {
                att_rsp_len = ble_gatt_server_process_request(&rx_pdu.payload[4],
                                                              l2cap_len,
                                                              att_rsp,
                                                              sizeof(att_rsp),
                                                              &has_att_rsp);
                if (has_att_rsp && (att_rsp_len > 0U))
                {
                    (void)controller_queue_att_payload(att_rsp, att_rsp_len);
                }
            }
        }
    }
}

static void radio_evt_handler(void)
{
    if (m_link.connected)
    {
        radio_handle_connected_packet();
        return;
    }

    if (adv_pdu_type_get(&m_ctrl_rt.air_pdu) == LL_CONNECT_REQ)
    {
        controller_apply_connect_request((const ble_connect_req_pdu_t *)&m_ctrl_rt.air_pdu);
    }
}
static void ble_advertise(void)
{
    uint8_t ch;

    if (!m_controller.advertising || m_link.connected)
    {
        return;
    }

    host_build_adv_pdu();
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.air_pdu);

    /* One advertising event must cover channels 37, 38, 39 for robust discovery. */
    for (ch = 0U; ch < 3U; ch++)
    {
        if (m_link.connected)
        {
            break;
        }

        controller_set_adv_channel(ch);
        radio_set_mode(MODE_TX);
        radio_tx_rx();

        /* Keep RX open long enough for SCAN_REQ/CONNECT_REQ + margin. */
        nrf_delay_us(BLE_ADV_RX_WINDOW_US);

        if (m_link.connected)
        {
            return;
        }

        radio_disable();
        radio_set_shorts(0U);
    }

    controller_set_adv_channel(0U);
}

void ble_start_advertising(void)
{
    ret_code_t err;
    uint32_t adv_ticks;

    if (m_link.connected)
    {
        return;
    }

    m_controller.advertising = true;
    controller_prepare_radio_for_adv();

    adv_ticks = APP_TIMER_TICKS(m_host.adv_interval_ms);
    if (adv_ticks < APP_TIMER_MIN_TIMEOUT_TICKS)
    {
        adv_ticks = APP_TIMER_MIN_TIMEOUT_TICKS;
    }

    err = app_timer_start(m_adv_timer_id, adv_ticks, NULL);
    APP_ERROR_CHECK(err);
}

void ble_stop_advertising(void)
{
    ret_code_t err;

    m_controller.advertising = false;
    err = app_timer_stop(m_adv_timer_id);
    if ((err != NRF_SUCCESS) && (err != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err);
    }
    radio_disable();
    radio_set_shorts(0U);
}

void controller_runtime_init(void)
{
    ret_code_t err;

    err = app_timer_init();
    if ((err != NRF_SUCCESS) && (err != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err);
    }

    if (!m_controller.adv_timer_created)
    {
        err = app_timer_create(&m_adv_timer_id, APP_TIMER_MODE_REPEATED, adv_timer_handler);
        APP_ERROR_CHECK(err);
        m_controller.adv_timer_created = true;
    }

    radio_power_on();
    radio_configure_modecnf0(RADIO_RAMP_UP_FAST, RADIO_DEFAULT_TX_B1);
    radio_set_tifs(150U);
    controller_conn_timer_init();
    controller_prepare_radio_for_adv();
    radio_set_event_handler(radio_evt_handler);
    radio_enable_interrupts();
}
