/**
 * @file ble_controller_common.c
 * @author Surya Poudel
 * @brief Shared BLE controller and link-layer implementation for nRF
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_controller_shared.h"
#include "ble_l2cap_internal.h"
#include "ble_gatt_client_internal.h"
#include "ble_gatt_server_internal.h"

#include <string.h>

static radio_data_rate_t controller_radio_data_rate_for_phy(uint8_t phy)
{
    return (phy == BLE_LL_PHY_2M) ? BLE_2MBPS : BLE_1MBPS;
}

static radio_preamble_length_t controller_radio_preamble_for_phy(uint8_t phy)
{
    return (phy == BLE_LL_PHY_2M) ? RADIO_PREAMBLE_16BIT : RADIO_PREAMBLE_8BIT;
}

static bool controller_phy_is_supported(uint8_t phy)
{
    return (phy == BLE_LL_PHY_1M) || (phy == BLE_LL_PHY_2M);
}

static uint8_t controller_symmetric_phy_mask_from_req(const uint8_t *p_payload, uint8_t len)
{
    uint8_t requested_phys;

    if ((p_payload == NULL) || (len < 3U))
    {
        return (BLE_LL_PHY_1M | BLE_LL_PHY_2M);
    }

    requested_phys = (uint8_t)(p_payload[1] & p_payload[2] & (BLE_LL_PHY_1M | BLE_LL_PHY_2M));

    return (requested_phys != 0U) ? requested_phys : BLE_LL_PHY_1M;
}

static void controller_local_features_fill(uint8_t features[8])
{
    (void)memset(features, 0, 8U);
    features[0] = BLE_LL_FEATURE_DATA_LENGTH_EXTENSION;
    features[1] = BLE_LL_FEATURE_2M_PHY;
}

static void controller_store_peer_features(const uint8_t *p_payload, uint8_t len)
{
    uint8_t local_features[8];
    uint8_t i;

    (void)memset(m_link.peer_features, 0, sizeof(m_link.peer_features));
    if ((p_payload == NULL) || (len < 9U))
    {
        return;
    }

    controller_local_features_fill(local_features);
    for (i = 0U; i < sizeof(m_link.peer_features); i++)
    {
        m_link.peer_features[i] = (uint8_t)(p_payload[1U + i] & local_features[i]);
    }
}

void controller_set_mode_with_phy(radio_mode_t mode, uint8_t phy)
{
    radio_cfg_drate_plen_and_enable_mode(mode, controller_radio_data_rate_for_phy(phy), controller_radio_preamble_for_phy(phy));
}

void controller_conn_timer_schedule_compare(void)
{
    uint32_t compare_tick = m_ctrl_rt.conn.conn_next_event_tick_us;

    /* Peripheral events should open RX slightly ahead of the anchor, but a
     * central must not transmit early or the peer will miss the packet. */
    if ((m_link.role == BLE_GAP_ROLE_PERIPHERAL) && (compare_tick > BLE_CONN_EVENT_GUARD_US))
    {
        compare_tick -= BLE_CONN_EVENT_GUARD_US;
    }
    else if (compare_tick == 0U)
    {
        compare_tick = 1U;
    }

    NRF_TIMER0->EVENTS_COMPARE[BLE_CONN_TIMER_COMPARE_CC_INDEX] = 0U;
    NRF_TIMER0->CC[BLE_CONN_TIMER_COMPARE_CC_INDEX] = compare_tick;
}

static void controller_conn_timer_init(void)
{
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER0->PRESCALER = BLE_CONN_TIMER_PRESCALER;
    NRF_TIMER0->SHORTS = 0U;
    NRF_TIMER0->INTENCLR = 0xFFFFFFFFUL;
    NRF_TIMER0->EVENTS_COMPARE[BLE_CONN_TIMER_COMPARE_CC_INDEX] = 0U;
    NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX] = 0U;
    NRF_PPI->CHENCLR = (1UL << BLE_CONN_TIMER_PPI_CH_RADIO_ADDRESS_CAPTURE);

    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, BLE_CONN_TIMER_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIMER0_IRQn);
}

static void controller_conn_timer_stop(void)
{
    NRF_TIMER0->TASKS_STOP = 1U;
    NRF_TIMER0->TASKS_CLEAR = 1U;
    NRF_TIMER0->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
    NRF_TIMER0->EVENTS_COMPARE[BLE_CONN_TIMER_COMPARE_CC_INDEX] = 0U;
    NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX] = 0U;
    NRF_PPI->CHENCLR = (1UL << BLE_CONN_TIMER_PPI_CH_RADIO_ADDRESS_CAPTURE);
    m_ctrl_rt.conn.conn_next_event_tick_us = 0U;
}

uint32_t controller_conn_next_event_tick_from_anchor(uint32_t current_event_tick_us, uint16_t current_event_counter)
{
    uint32_t next_event_tick_us = current_event_tick_us + m_link.conn.conn_interval_us;

    if (m_link.pending_conn_update.valid && ((uint16_t)(current_event_counter + 1U) == m_link.pending_conn_update.instant))
    {
        next_event_tick_us += m_link.pending_conn_update.window_offset_us;
    }
    else if (m_link.pending_conn_update.valid && (current_event_counter == m_link.pending_conn_update.instant))
    {
        m_link.conn = m_link.pending_conn_update.conn;
        next_event_tick_us = current_event_tick_us + m_link.conn.conn_interval_us;
        m_link.pending_conn_update.valid = false;
        m_link.supervision.missed_interval_count = 0U;
        if (m_link.role == BLE_GAP_ROLE_CENTRAL)
        {
            controller_central_handle_conn_update_instant_complete();
        }
        (void)ble_evt_notify_gap(BLE_GAP_EVT_CONN_UPDATE_IND);
    }

    return next_event_tick_us;
}

void controller_prepare_radio_common(uint8_t max_payload_size, const uint8_t *p_address, uint32_t crc_init, uint32_t packet_ptr)
{
    radio_reset_packet_config();
    radio_set_len_field_size(8U);
    radio_set_s0_field_size(1U);
    radio_set_preamble_length(RADIO_PREAMBLE_8BIT);
    radio_set_data_rate(BLE_1MBPS);
    radio_set_max_payload_size(max_payload_size);
    radio_set_address_width(4U);
    radio_set_payload_endian(RADIO_LITTLE_ENDIAN);
    radio_enable_whitening(true);
    radio_set_address(p_address, 4U, 0U);
    radio_set_tx_logical_address(0U);
    radio_set_rx_logical_address(0U);
    radio_set_tx_power((uint32_t)(uint8_t)m_host.common.tx_power);
    radio_configure_crc(3U, 1U, m_ble_crc_poly, crc_init);
    radio_set_packet_ptr(packet_ptr);
}

static void controller_apply_channel_map(const uint8_t *p_channel_map)
{
    uint8_t i;

    m_link.channel.channel_map_bits = 0U;
    (void)memcpy(&m_link.channel.channel_map_bits, p_channel_map, 5U);
    m_link.channel.channel_map_bits &= 0x1FFFFFFFFFULL;
    m_link.channel.channel_count = 0U;
    for (i = 0U; i < 37U; i++)
    {
        if ((m_link.channel.channel_map_bits & (1ULL << i)) != 0ULL)
        {
            m_link.channel.channels[m_link.channel.channel_count++] = i;
        }
    }
    if (m_link.channel.channel_count == 0U)
    {
        for (i = 0U; i < 37U; i++)
        {
            m_link.channel.channels[i] = i;
        }
        m_link.channel.channel_count = 37U;
    }
}

static uint16_t controller_data_length_octets_clamp(uint16_t octets)
{
    if (octets < BLE_LL_DATA_LEN_DEFAULT_OCTETS)
    {
        return BLE_LL_DATA_LEN_DEFAULT_OCTETS;
    }

    return (octets > BLE_LL_DATA_LEN_MAX_OCTETS) ? BLE_LL_DATA_LEN_MAX_OCTETS : octets;
}

static uint16_t controller_data_length_time_clamp(uint16_t time_us)
{
    if (time_us < BLE_LL_DATA_LEN_DEFAULT_TIME)
    {
        return BLE_LL_DATA_LEN_DEFAULT_TIME;
    }

    return (time_us > BLE_LL_DATA_LEN_MAX_TIME) ? BLE_LL_DATA_LEN_MAX_TIME : time_us;
}

static void controller_apply_data_length(uint16_t peer_max_rx_octets,
                                         uint16_t peer_max_rx_time_us,
                                         uint16_t peer_max_tx_octets,
                                         uint16_t peer_max_tx_time_us)
{
    m_link.packet.max_tx_octets = controller_data_length_octets_clamp(peer_max_rx_octets);
    m_link.packet.max_rx_octets = controller_data_length_octets_clamp(peer_max_tx_octets);
    m_link.packet.max_tx_time_us = controller_data_length_time_clamp(peer_max_rx_time_us);
    m_link.packet.max_rx_time_us = controller_data_length_time_clamp(peer_max_tx_time_us);
    (void)ble_evt_notify_gap(BLE_GAP_EVT_DATA_LENGTH_UPDATED);
}

static void controller_schedule_phy_update(const uint8_t *p_payload, uint8_t len)
{
    uint8_t next_rx_phy;
    uint8_t next_tx_phy;

    if ((p_payload == NULL) || (len < 5U))
    {
        return;
    }

    next_rx_phy = p_payload[1];
    next_tx_phy = p_payload[2];
    if (next_rx_phy == 0U)
    {
        next_rx_phy = m_link.phy.rx_phy;
    }
    if (next_tx_phy == 0U)
    {
        next_tx_phy = m_link.phy.tx_phy;
    }
    if (!controller_phy_is_supported(next_rx_phy) ||
        !controller_phy_is_supported(next_tx_phy))
    {
        return;
    }
    if (next_rx_phy != next_tx_phy)
    {
        controller_disconnect_internal();
        return;
    }

    m_link.pending_phy_update.phy.rx_phy = next_rx_phy;
    m_link.pending_phy_update.phy.tx_phy = next_tx_phy;
    m_link.pending_phy_update.instant = u16_decode(&p_payload[3]);
    m_link.pending_phy_update.valid = true;
}

void controller_schedule_conn_update(const uint8_t *p_payload, uint8_t len)
{
    if ((p_payload == NULL) || (len < 12U))
    {
        return;
    }

    m_link.pending_conn_update.window_offset_us = UNITS_1P25MS_TO_US(u16_decode(&p_payload[2]));
    m_link.pending_conn_update.conn.conn_interval_us = UNITS_1P25MS_TO_US(u16_decode(&p_payload[4]));
    m_link.pending_conn_update.conn.conn_interval_ms = UNITS_1P25MS_TO_MS(u16_decode(&p_payload[4]));
    m_link.pending_conn_update.conn.slave_latency = u16_decode(&p_payload[6]);
    m_link.pending_conn_update.conn.supervision_timeout_ms = UNITS_10MS_TO_MS(u16_decode(&p_payload[8]));
    m_link.pending_conn_update.instant = u16_decode(&p_payload[10]);
    m_link.pending_conn_update.valid = (m_link.pending_conn_update.conn.conn_interval_us != 0U) && (m_link.pending_conn_update.conn.supervision_timeout_ms != 0U);
}

void controller_hop_data_channel(void)
{
    uint8_t unmapped;
    uint8_t mapped;

    unmapped = (uint8_t)((m_link.channel.last_unmapped_channel + m_link.channel.hop_increment) % 37U);
    m_link.channel.last_unmapped_channel = unmapped;

    if ((m_link.channel.channel_map_bits & (1ULL << unmapped)) != 0ULL)
    {
        mapped = unmapped;
    }
    else
    {
        mapped = m_link.channel.channels[unmapped % m_link.channel.channel_count];
    }

    radio_set_frequency(m_data_channel_freq[mapped]);
    radio_set_whiteiv(mapped);
}

ble_ll_data_header_t controller_conn_header(uint8_t llid)
{
    return (ble_ll_data_header_t){
        .llid = llid,
        .nesn = (uint8_t)(m_link.packet.next_expected_rx_sn & 0x01U),
        .sn = (uint8_t)(m_link.packet.tx_sn & 0x01U),
        .md = 0U,
        .rfu = 0U,
    };
}

ble_ll_data_header_t controller_conn_header_for_state(uint8_t llid, uint8_t next_expected_rx_sn, uint8_t tx_sn)
{
    return (ble_ll_data_header_t){
        .llid = llid,
        .nesn = (uint8_t)(next_expected_rx_sn & 0x01U),
        .sn = (uint8_t)(tx_sn & 0x01U),
        .md = 0U,
        .rfu = 0U,
    };
}

static bool controller_queue_control_payload(ble_ll_data_raw_pdu_t *p_pdu,
                                             bool *p_has_pending,
                                             const uint8_t *p_payload,
                                             uint8_t payload_len)
{
    uint32_t primask;

    if ((p_pdu == NULL) || (p_has_pending == NULL) || (p_payload == NULL) ||
        (payload_len == 0U) || !m_link.connected ||
        (payload_len > sizeof(p_pdu->payload)))
    {
        return false;
    }

    primask = irq_lock();
    if (*p_has_pending)
    {
        irq_unlock(primask);
        return false;
    }

    p_pdu->header = controller_conn_header(BLE_LLID_CONTROL_PDU);
    p_pdu->length = payload_len;
    (void)memcpy(p_pdu->payload, p_payload, payload_len);
    *p_has_pending = true;
    irq_unlock(primask);
    return true;
}

static bool controller_queue_ll_control_response_payload(const uint8_t *p_payload, uint8_t payload_len)
{
    return controller_queue_control_payload(&m_ctrl_rt.conn.pending_conn_ctrl_rsp_pdu,
                                            &m_ctrl_rt.conn.has_pending_conn_ctrl_rsp_pdu,
                                            p_payload,
                                            payload_len);
}

bool controller_queue_ll_control_payload(const uint8_t *p_payload, uint8_t payload_len)
{
    return controller_queue_control_payload(&m_ctrl_rt.conn.pending_conn_ctrl_pdu,
                                            &m_ctrl_rt.conn.has_pending_conn_ctrl_pdu,
                                            p_payload,
                                            payload_len);
}

void controller_reset_conn_tx_selection_state(void)
{
    m_ctrl_rt.conn.selected_conn_tx_source = BLE_CONN_PENDING_SOURCE_NONE;
}

static void controller_l2cap_tx_queue_reset(void)
{
    m_ctrl_rt.conn.l2cap_tx_queue.ridx = 0U;
    m_ctrl_rt.conn.l2cap_tx_queue.widx = 0U;
    m_ctrl_rt.conn.l2cap_tx_queue.count = 0U;
}

static ble_ll_data_raw_pdu_t *controller_l2cap_tx_queue_peek(void)
{
    if (m_ctrl_rt.conn.l2cap_tx_queue.count == 0U)
    {
        return NULL;
    }

    return &m_ctrl_rt.conn.l2cap_tx_queue.q[m_ctrl_rt.conn.l2cap_tx_queue.ridx];
}

static void controller_l2cap_tx_queue_pop(void)
{
    if (m_ctrl_rt.conn.l2cap_tx_queue.count == 0U)
    {
        return;
    }

    m_ctrl_rt.conn.l2cap_tx_queue.ridx =
        (uint8_t)((m_ctrl_rt.conn.l2cap_tx_queue.ridx + 1U) % BLE_CONN_TX_L2CAP_QUEUE_DEPTH);
    m_ctrl_rt.conn.l2cap_tx_queue.count--;
}

static void controller_clear_consumed_pending_source(uint8_t pending_source)
{
    if (pending_source == BLE_CONN_PENDING_SOURCE_CONTROL_RESPONSE)
    {
        m_ctrl_rt.conn.has_pending_conn_ctrl_rsp_pdu = false;
    }
    else if (pending_source == BLE_CONN_PENDING_SOURCE_CONTROL)
    {
        m_ctrl_rt.conn.has_pending_conn_ctrl_pdu = false;
    }
    else if (pending_source == BLE_CONN_PENDING_SOURCE_L2CAP)
    {
        controller_l2cap_tx_queue_pop();
    }
}

bool controller_load_pending_conn_tx_pdu_for_state(uint8_t next_expected_rx_sn, uint8_t tx_sn)
{
    if (m_ctrl_rt.conn.has_pending_conn_ctrl_rsp_pdu)
    {
        m_ctrl_rt.conn.conn_tx_pdu = m_ctrl_rt.conn.pending_conn_ctrl_rsp_pdu;
        m_ctrl_rt.conn.conn_tx_pdu.header = controller_conn_header_for_state(m_ctrl_rt.conn.conn_tx_pdu.header.llid,
                                                                             next_expected_rx_sn,
                                                                             tx_sn);
        m_ctrl_rt.conn.selected_conn_tx_source = BLE_CONN_PENDING_SOURCE_CONTROL_RESPONSE;
        return true;
    }

    if (m_ctrl_rt.conn.has_pending_conn_ctrl_pdu)
    {
        m_ctrl_rt.conn.conn_tx_pdu = m_ctrl_rt.conn.pending_conn_ctrl_pdu;
        m_ctrl_rt.conn.conn_tx_pdu.header = controller_conn_header_for_state(m_ctrl_rt.conn.conn_tx_pdu.header.llid,
                                                                             next_expected_rx_sn,
                                                                             tx_sn);
        m_ctrl_rt.conn.selected_conn_tx_source = BLE_CONN_PENDING_SOURCE_CONTROL;
        return true;
    }

    if (m_ctrl_rt.conn.l2cap_tx_queue.count != 0U)
    {
        ble_ll_data_raw_pdu_t *p_pending = controller_l2cap_tx_queue_peek();

        if (p_pending == NULL)
        {
            return false;
        }

        m_ctrl_rt.conn.conn_tx_pdu = *p_pending;
        m_ctrl_rt.conn.conn_tx_pdu.header = controller_conn_header_for_state(m_ctrl_rt.conn.conn_tx_pdu.header.llid,
                                                                             next_expected_rx_sn,
                                                                             tx_sn);
        m_ctrl_rt.conn.selected_conn_tx_source = BLE_CONN_PENDING_SOURCE_L2CAP;
        return true;
    }

    return false;
}

void controller_disconnect_internal(void)
{
    bool was_connected = m_link.connected;
    ble_gap_role_t old_role = m_link.role;

    if (old_role == BLE_GAP_ROLE_CENTRAL)
    {
        controller_central_scan_timer_stop();
        controller_central_scan_window_timer_stop();
    }
    m_link.connected = false;
    m_link.role = BLE_GAP_ROLE_NONE;
    m_link.peer_addr = (ble_gap_addr_t){0};
    m_link.conn.conn_interval_us = 0U;
    m_ctrl_rt.conn.tx_unacked = false;
    m_ctrl_rt.conn.has_pending_conn_ctrl_rsp_pdu = false;
    m_ctrl_rt.conn.has_pending_conn_ctrl_pdu = false;
    controller_l2cap_tx_queue_reset();
    m_ctrl_rt.conn.conn_rx_process_pending = false;
    m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    controller_reset_conn_tx_selection_state();
    m_link.pending_channel_map.valid = false;
    m_link.pending_conn_update.valid = false;
    m_link.pending_phy_update.valid = false;
    m_link.pending_channel_map.instant = 0U;
    controller_conn_timer_stop();
    ble_gatt_server_reset_connection_state();
    ble_gatt_client_reset_connection_state();
    radio_disable();
    if (old_role == BLE_GAP_ROLE_CENTRAL)
    {
        controller_central_state_reset();
    }
    else if (old_role == BLE_GAP_ROLE_PERIPHERAL)
    {
        controller_peripheral_state_reset();
    }
    radio_enable_interrupt_mask(0U);

    if (was_connected)
    {
        (void)ble_evt_notify_gap(BLE_GAP_EVT_DISCONNECTED);
    }
}

static uint16_t ll_control_process(const uint8_t *p_payload, uint8_t len, uint8_t *p_rsp)
{
    uint8_t opcode;

    if (len == 0U)
    {
        return 0U;
    }

    opcode = p_payload[0];
    switch (opcode)
    {
    case BLE_LL_CTRL_CONN_UPDATE_IND:
        controller_schedule_conn_update(p_payload, len);
        return 0U;

    case BLE_LL_CTRL_CHANNEL_MAP_IND:
        if (len >= 8U)
        {
            (void)memcpy(m_link.pending_channel_map.map, &p_payload[1], sizeof(m_link.pending_channel_map.map));
            m_link.pending_channel_map.instant = u16_decode(&p_payload[6]);
            m_link.pending_channel_map.valid = true;
        }
        return 0U;

    case BLE_LL_CTRL_FEATURE_REQ:
    case BLE_LL_CTRL_SLV_FEATURE_REQ:
        controller_store_peer_features(p_payload, len);
        (void)ble_evt_notify_gap(BLE_GAP_EVT_FEATURE_EXCHANGED);
        p_rsp[0] = BLE_LL_CTRL_FEATURE_RSP;
        controller_local_features_fill(&p_rsp[1]);
        return 9U;

    case BLE_LL_CTRL_FEATURE_RSP:
        controller_store_peer_features(p_payload, len);
        if (m_link.role == BLE_GAP_ROLE_CENTRAL)
        {
            controller_central_handle_feature_exchange_complete();
        }
        (void)ble_evt_notify_gap(BLE_GAP_EVT_FEATURE_EXCHANGED);
        return 0U;

    case BLE_LL_CTRL_VERSION_IND:
        p_rsp[0] = BLE_LL_CTRL_VERSION_IND;
        p_rsp[1] = BLE_LL_VERSION_4_2;
        u16_encode(BLE_LL_COMPANY_ID_NORDIC, &p_rsp[2]);
        u16_encode(BLE_LL_SUBVERSION, &p_rsp[4]);
        return 6U;

    case BLE_LL_CTRL_LENGTH_REQ:
        if (len >= 9U)
        {
            controller_apply_data_length(u16_decode(&p_payload[1]),
                                         u16_decode(&p_payload[3]),
                                         u16_decode(&p_payload[5]),
                                         u16_decode(&p_payload[7]));
        }
        p_rsp[0] = BLE_LL_CTRL_LENGTH_RSP;
        u16_encode(BLE_LL_DATA_LEN_MAX_OCTETS, &p_rsp[1]);
        u16_encode(BLE_LL_DATA_LEN_MAX_TIME, &p_rsp[3]);
        u16_encode(BLE_LL_DATA_LEN_MAX_OCTETS, &p_rsp[5]);
        u16_encode(BLE_LL_DATA_LEN_MAX_TIME, &p_rsp[7]);
        return 9U;

    case BLE_LL_CTRL_LENGTH_RSP:
        if (len >= 9U)
        {
            controller_apply_data_length(u16_decode(&p_payload[1]),
                                         u16_decode(&p_payload[3]),
                                         u16_decode(&p_payload[5]),
                                         u16_decode(&p_payload[7]));
            if (m_link.role == BLE_GAP_ROLE_CENTRAL)
            {
                controller_central_handle_data_length_update_complete();
            }
        }
        return 0U;

    case BLE_LL_CTRL_PHY_REQ:
    {
        uint8_t symmetric_phys = controller_symmetric_phy_mask_from_req(p_payload, len);

        p_rsp[0] = BLE_LL_CTRL_PHY_RSP;
        p_rsp[1] = symmetric_phys;
        p_rsp[2] = symmetric_phys;
        return 3U;
    }

    case BLE_LL_CTRL_PHY_RSP:
        if (m_link.role == BLE_GAP_ROLE_CENTRAL)
        {
            return controller_central_process_phy_rsp(p_payload, len, p_rsp);
        }
        return 0U;

    case BLE_LL_CTRL_PHY_UPDATE_IND:
        controller_schedule_phy_update(p_payload, len);
        return 0U;

    case BLE_LL_CTRL_UNKNOWN_RSP:
        if ((m_link.role == BLE_GAP_ROLE_CENTRAL) && (len >= 2U))
        {
            controller_central_handle_unknown_rsp(p_payload[1]);
        }
        return 0U;

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

bool controller_queue_l2cap_payload(uint16_t cid, const uint8_t *p_payload, uint16_t payload_len)
{
    uint32_t primask;

    if ((p_payload == NULL) || (payload_len == 0U) || !m_link.connected)
    {
        return false;
    }

    if ((uint16_t)(payload_len + BLE_L2CAP_HDR_LEN) > sizeof(m_ctrl_rt.conn.l2cap_tx_queue.q[0].payload))
    {
        return false;
    }

    primask = irq_lock();

    if (m_ctrl_rt.conn.l2cap_tx_queue.count >= BLE_CONN_TX_L2CAP_QUEUE_DEPTH)
    {
        irq_unlock(primask);
        return false;
    }

    {
        ble_ll_data_raw_pdu_t *p_pending =
            &m_ctrl_rt.conn.l2cap_tx_queue.q[m_ctrl_rt.conn.l2cap_tx_queue.widx];

        p_pending->header = controller_conn_header(BLE_LLID_START_L2CAP);
        p_pending->length = (uint8_t)(payload_len + BLE_L2CAP_HDR_LEN);
        u16_encode(payload_len, &p_pending->payload[0]);
        u16_encode(cid, &p_pending->payload[2]);
        (void)memcpy(&p_pending->payload[BLE_L2CAP_HDR_LEN], p_payload, payload_len);
        m_ctrl_rt.conn.l2cap_tx_queue.widx =
            (uint8_t)((m_ctrl_rt.conn.l2cap_tx_queue.widx + 1U) % BLE_CONN_TX_L2CAP_QUEUE_DEPTH);
        m_ctrl_rt.conn.l2cap_tx_queue.count++;
    }
    irq_unlock(primask);

    return true;
}

void controller_stage_conn_response(bool new_tx_pdu)
{
    radio_set_packet_ptr((uint32_t)(new_tx_pdu ? &m_ctrl_rt.conn.conn_tx_pdu : &m_ctrl_rt.conn.last_conn_tx_pdu));
    if (new_tx_pdu)
    {
        m_ctrl_rt.conn.last_conn_tx_pdu = m_ctrl_rt.conn.conn_tx_pdu;
        m_ctrl_rt.conn.tx_unacked = true;
        if (m_ctrl_rt.conn.selected_conn_tx_source != BLE_CONN_PENDING_SOURCE_NONE)
        {
            controller_clear_consumed_pending_source(m_ctrl_rt.conn.selected_conn_tx_source);
        }
    }
}

void controller_process_received_conn_pdu(void)
{
    ble_ll_data_raw_pdu_t rx_pdu = m_ctrl_rt.conn.conn_rx_pdu;
    uint16_t ctrl_rsp_len = 0U;
    uint8_t ctrl_rsp[sizeof(m_ctrl_rt.conn.pending_conn_ctrl_rsp_pdu.payload)];

    if (rx_pdu.header.llid == BLE_LLID_CONTROL_PDU)
    {
        if (rx_pdu.length == 0U)
        {
            return;
        }

        ctrl_rsp_len = ll_control_process(rx_pdu.payload, rx_pdu.length, ctrl_rsp);
        if ((ctrl_rsp_len > 0U) && m_link.connected)
        {
            (void)controller_queue_ll_control_response_payload(ctrl_rsp, (uint8_t)ctrl_rsp_len);
        }
        return;
    }

    if ((rx_pdu.header.llid == BLE_LLID_START_L2CAP) || (rx_pdu.header.llid == BLE_LLID_CONTINUATION))
    {
        ble_l2cap_process_conn_data_pdu(rx_pdu.payload, rx_pdu.length);
    }
}

void TIMER0_IRQHandler(void)
{
    uint32_t current_event_tick_us;
    uint32_t next_event_tick_us;

    if (NRF_TIMER0->EVENTS_COMPARE[BLE_CONN_TIMER_COMPARE_CC_INDEX] == 0U)
    {
        return;
    }

    NRF_TIMER0->EVENTS_COMPARE[BLE_CONN_TIMER_COMPARE_CC_INDEX] = 0U;

    if (!m_link.connected)
    {
        return;
    }

    current_event_tick_us = m_ctrl_rt.conn.conn_next_event_tick_us;
    next_event_tick_us = controller_conn_next_event_tick_from_anchor(current_event_tick_us, m_link.event_counter);

    if (m_link.supervision.started)
    {
        if (m_link.supervision.rx_seen_this_interval)
        {
            m_link.supervision.missed_interval_count = 0U;
        }
        else
        {
            m_link.supervision.missed_interval_count++;
            if (((uint32_t)m_link.supervision.missed_interval_count * (uint32_t)m_link.conn.conn_interval_ms) >= (uint32_t)m_link.conn.supervision_timeout_ms)
            {
                (void)ble_evt_notify_gap(BLE_GAP_EVT_SUPERVISION_TIMEOUT);
                controller_disconnect_internal();
                return;
            }
        }
    }

    m_link.supervision.rx_seen_this_interval = false;

    m_ctrl_rt.conn.conn_next_event_tick_us = next_event_tick_us;
    controller_conn_timer_schedule_compare();

    if (m_link.pending_channel_map.valid && (m_link.event_counter == m_link.pending_channel_map.instant))
    {
        controller_apply_channel_map(m_link.pending_channel_map.map);
        m_link.pending_channel_map.valid = false;
    }

    if (m_link.pending_phy_update.valid && (m_link.event_counter == m_link.pending_phy_update.instant))
    {
        m_link.phy = m_link.pending_phy_update.phy;
        m_link.pending_phy_update.valid = false;
        if (m_link.role == BLE_GAP_ROLE_CENTRAL)
        {
            controller_central_handle_phy_update_instant_complete();
        }
        (void)ble_evt_notify_gap(BLE_GAP_EVT_PHY_UPDATE_IND);
    }

    if (m_link.role == BLE_GAP_ROLE_CENTRAL)
    {
        controller_central_start_connection_event();
    }
    else
    {
        controller_peripheral_start_connection_event();
    }
    m_link.event_counter++;
}

void controller_connected_timer_start(uint32_t first_event_delay_us)
{
    controller_conn_timer_stop();
    m_ctrl_rt.conn.conn_next_event_tick_us = first_event_delay_us;
    controller_conn_timer_schedule_compare();
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER0->TASKS_CLEAR = 1U;
    NRF_TIMER0->CC[BLE_CONN_TIMER_CAPTURE_CC_INDEX] = 0U;
    NRF_PPI->CHENSET = (1UL << BLE_CONN_TIMER_PPI_CH_RADIO_ADDRESS_CAPTURE);
    NRF_TIMER0->TASKS_START = 1U;
}

void controller_prepare_connected_link(const ble_connect_req_pdu_t *p_req,
                                       ble_gap_role_t role,
                                       const ble_gap_addr_t *p_peer_addr)
{
    (void)memset(&m_link, 0, sizeof(m_link));
    m_link.role = role;
    m_link.connected = true;
    if (p_peer_addr != NULL)
    {
        m_link.peer_addr = *p_peer_addr;
    }
    m_link.conn.conn_interval_ms = UNITS_1P25MS_TO_MS(p_req->ll_data.interval);
    m_link.conn.conn_interval_us = UNITS_1P25MS_TO_US(p_req->ll_data.interval);
    m_link.conn.slave_latency = p_req->ll_data.latency;
    m_link.conn.supervision_timeout_ms = UNITS_10MS_TO_MS(p_req->ll_data.timeout);
    m_link.channel.hop_increment = p_req->ll_data.hop_increment;
    if (m_link.channel.hop_increment == 0U)
    {
        m_link.channel.hop_increment = BLE_INITIATOR_HOP_INCREMENT;
    }
    m_link.packet.crc_init = (uint32_t)p_req->ll_data.crc_init[0] |
                             ((uint32_t)p_req->ll_data.crc_init[1] << 8) |
                             ((uint32_t)p_req->ll_data.crc_init[2] << 16);
    m_link.packet.max_tx_octets = BLE_LL_DATA_LEN_DEFAULT_OCTETS;
    m_link.packet.max_rx_octets = BLE_LL_DATA_LEN_DEFAULT_OCTETS;
    m_link.packet.max_tx_time_us = BLE_LL_DATA_LEN_DEFAULT_TIME;
    m_link.packet.max_rx_time_us = BLE_LL_DATA_LEN_DEFAULT_TIME;
    m_link.phy.tx_phy = BLE_LL_PHY_1M;
    m_link.phy.rx_phy = BLE_LL_PHY_1M;
    m_ctrl_rt.conn.tx_unacked = false;
    m_ctrl_rt.conn.has_pending_conn_ctrl_rsp_pdu = false;
    m_ctrl_rt.conn.has_pending_conn_ctrl_pdu = false;
    controller_l2cap_tx_queue_reset();
    m_ctrl_rt.conn.conn_rx_process_pending = false;
    m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        controller_central_state_reset();
    }
    else
    {
        controller_peripheral_state_reset();
    }
    controller_reset_conn_tx_selection_state();
    ble_gatt_server_reset_connection_state();
    ble_gatt_client_reset_connection_state();

    (void)memcpy(m_link.packet.access_address,
                 p_req->ll_data.access_address,
                 sizeof(m_link.packet.access_address));
    controller_apply_channel_map(p_req->ll_data.channel_map);

    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN_CENTRAL);
    }
    else
    {
        radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN_PERIPHERAL);
    }
    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.conn.conn_rx_pdu.payload),
                                    m_link.packet.access_address,
                                    m_link.packet.crc_init,
                                    (uint32_t)&m_ctrl_rt.conn.conn_rx_pdu);
    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        controller_central_auto_ctrl_start();
    }
}

static void radio_evt_handler(radio_event_t evt)
{
    const ble_adv_rx_pdu_t *p_adv_rx = &m_ctrl_rt.peripheral.adv_rx_pdu;

    if (m_link.connected)
    {
        if (m_link.role == BLE_GAP_ROLE_CENTRAL)
        {
            const ble_adv_rx_pdu_t *p_scan_rx = &m_ctrl_rt.central.scan_rx_pdu;

            controller_central_handle_radio_event(evt, p_scan_rx);
        }
        else
        {
            controller_peripheral_handle_radio_event(evt, p_adv_rx);
        }
        return;
    }

    if (m_ctrl_rt.peripheral.adv_radio_phase != BLE_ADV_RADIO_PHASE_IDLE)
    {
        controller_peripheral_handle_radio_event(evt, p_adv_rx);
        return;
    }

    if ((m_host.configured_role == BLE_GAP_ROLE_CENTRAL) &&
        (m_ctrl_rt.central.scan_radio_phase != BLE_SCAN_RADIO_PHASE_IDLE))
    {
        const ble_adv_rx_pdu_t *p_scan_rx = &m_ctrl_rt.central.scan_rx_pdu;

        controller_central_handle_radio_event(evt, p_scan_rx);
    }
}

void controller_runtime_init(void)
{
    APP_ERROR_CHECK(app_timer_init());

    controller_peripheral_adv_timer_init();
    controller_central_scan_timers_init();

    radio_power_on();
    radio_configure_modecnf0(RADIO_RAMP_UP_DEFAULT, RADIO_DEFAULT_TX_B1);
    radio_set_tifs(150U);
    controller_conn_timer_init();
    radio_set_event_handler(radio_evt_handler);
    radio_enable_interrupt_mask(0U);
}
