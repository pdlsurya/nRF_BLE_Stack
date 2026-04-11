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

#define BLE_LL_DATA_HEADER_BITS 16U
#define BLE_SCAN_REQ_PAYLOAD_LEN 12U
#define BLE_PRIMARY_ADV_PDU_TYPE LL_ADV_IND
#define BLE_RADIO_IRQ_MASK_ADV (RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_RADIO_IRQ_MASK_CONN (RADIO_INTENSET_BCMATCH_Msk | RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)

static void controller_reset_conn_bcmatch_state(void);
static void controller_reset_adv_radio_state(void);
static void controller_handle_advertising_disabled(void);

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

static void controller_set_mode_with_phy(radio_mode_t mode, uint8_t phy)
{
    radio_cfg_drate_plen_and_enable_mode(mode, controller_radio_data_rate_for_phy(phy), controller_radio_preamble_for_phy(phy));
}

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
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER2->PRESCALER = BLE_CONN_TIMER_PRESCALER;
    NRF_TIMER2->SHORTS = 0U;
    NRF_TIMER2->INTENCLR = 0xFFFFFFFFUL;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;

    NVIC_ClearPendingIRQ(TIMER2_IRQn);
    NVIC_SetPriority(TIMER2_IRQn, BLE_CONN_TIMER_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIMER2_IRQn);
}

static void controller_conn_timer_stop(void)
{
    NRF_TIMER2->TASKS_STOP = 1U;
    NRF_TIMER2->TASKS_CLEAR = 1U;
    NRF_TIMER2->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;
    m_ctrl_rt.conn_next_event_tick_us = 0U;
}

static void controller_prepare_radio_common(uint8_t max_payload_size, const uint8_t *p_address, uint32_t crc_init, uint32_t packet_ptr)
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
    radio_set_tx_power((uint32_t)(uint8_t)m_host.tx_power);
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

static void controller_apply_data_length(uint16_t peer_max_rx_octets, uint16_t peer_max_tx_octets)
{
    m_link.packet.max_tx_octets = controller_data_length_octets_clamp(peer_max_rx_octets);
    m_link.packet.max_rx_octets = controller_data_length_octets_clamp(peer_max_tx_octets);
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

static void controller_schedule_conn_update(const uint8_t *p_payload, uint8_t len)
{
    if ((p_payload == NULL) || (len < 11U))
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

static void controller_hop_data_channel(void)
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

static ble_ll_data_header_t controller_conn_header(uint8_t llid)
{
    return (ble_ll_data_header_t){
        .llid = llid,
        .nesn = (uint8_t)(m_link.packet.next_expected_rx_sn & 0x01U),
        .sn = (uint8_t)(m_link.packet.tx_sn & 0x01U),
        .md = 0U,
        .rfu = 0U,
    };
}

static ble_ll_data_header_t controller_conn_header_for_state(uint8_t llid, uint8_t next_expected_rx_sn, uint8_t tx_sn)
{
    return (ble_ll_data_header_t){
        .llid = llid,
        .nesn = (uint8_t)(next_expected_rx_sn & 0x01U),
        .sn = (uint8_t)(tx_sn & 0x01U),
        .md = 0U,
        .rfu = 0U,
    };
}

static bool host_add_ad_structure(uint8_t *p_adv_data_len, uint8_t type, const uint8_t *p_data, uint8_t size)
{
    uint8_t *p_adv_data;

    if ((uint16_t)(*p_adv_data_len) + (uint16_t)size + 2U > BLE_MAX_ADV_DATA_LEN)
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

    if (m_host.adv_name[0] != '\0')
    {
        (void)host_add_ad_structure(&adv_data_len, 0x09U, (const uint8_t *)m_host.adv_name, (uint8_t)strlen(m_host.adv_name));
    }

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

static bool controller_scan_request_targets_us(const ble_scan_req_pdu_t *p_req)
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

void controller_disconnect_internal(void)
{
    bool was_connected = m_link.connected;

    m_link.connected = false;
    m_link.conn.conn_interval_us = 0U;
    m_ctrl_rt.tx_unacked = false;
    m_ctrl_rt.has_pending_conn_tx_pdu = false;
    m_ctrl_rt.conn_rx_process_pending = false;
    m_ctrl_rt.adv_radio_phase = BLE_ADV_RADIO_PHASE_IDLE;
    m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    controller_reset_conn_bcmatch_state();
    m_link.pending_channel_map.valid = false;
    m_link.pending_conn_update.valid = false;
    m_link.pending_phy_update.valid = false;
    m_link.pending_channel_map.instant = 0U;
    controller_conn_timer_stop();
    ble_gatt_server_reset_connection_state();
    radio_disable();
    controller_reset_adv_radio_state();
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
        p_rsp[0] = BLE_LL_CTRL_FEATURE_RSP;
        (void)memset(&p_rsp[1], 0, 8U);
        p_rsp[1] = BLE_LL_FEATURE_DATA_LENGTH_EXTENSION;
        p_rsp[2] = BLE_LL_FEATURE_2M_PHY;
        return 9U;

    case BLE_LL_CTRL_VERSION_IND:
        p_rsp[0] = BLE_LL_CTRL_VERSION_IND;
        p_rsp[1] = BLE_LL_VERSION_4_2;
        u16_encode(BLE_LL_COMPANY_ID_NORDIC, &p_rsp[2]);
        u16_encode(BLE_LL_SUBVERSION, &p_rsp[4]);
        return 6U;

    case BLE_LL_CTRL_LENGTH_REQ:
        if (len >= 9U)
        {
            controller_apply_data_length(u16_decode(&p_payload[1]), u16_decode(&p_payload[5]));
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
            controller_apply_data_length(u16_decode(&p_payload[1]), u16_decode(&p_payload[5]));
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
        return 0U;

    case BLE_LL_CTRL_PHY_UPDATE_IND:
        controller_schedule_phy_update(p_payload, len);
        return 0U;

    case BLE_LL_CTRL_UNKNOWN_RSP:
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

    if ((uint16_t)(payload_len + BLE_L2CAP_HDR_LEN) > sizeof(m_ctrl_rt.pending_conn_tx_pdu.payload))
    {
        return false;
    }

    primask = irq_lock();

    if (m_ctrl_rt.has_pending_conn_tx_pdu)
    {
        irq_unlock(primask);
        return false;
    }

    m_ctrl_rt.pending_conn_tx_pdu.header = controller_conn_header(BLE_LLID_START_L2CAP);
    m_ctrl_rt.pending_conn_tx_pdu.length = (uint8_t)(payload_len + BLE_L2CAP_HDR_LEN);
    u16_encode(payload_len, &m_ctrl_rt.pending_conn_tx_pdu.payload[0]);
    u16_encode(cid, &m_ctrl_rt.pending_conn_tx_pdu.payload[2]);
    (void)memcpy(&m_ctrl_rt.pending_conn_tx_pdu.payload[BLE_L2CAP_HDR_LEN], p_payload, payload_len);
    m_ctrl_rt.has_pending_conn_tx_pdu = true;
    irq_unlock(primask);

    return true;
}

static void controller_reset_conn_bcmatch_state(void)
{
    m_ctrl_rt.conn_bcmatch.tx_acked = false;
    m_ctrl_rt.conn_bcmatch.is_new_packet = false;
    m_ctrl_rt.conn_bcmatch.consumes_pending = false;
}

static void controller_reset_adv_radio_state(void)
{
    m_ctrl_rt.adv_scan_rsp_pending = false;
    m_ctrl_rt.adv_connect_pending = false;
    m_ctrl_rt.adv_radio_phase = BLE_ADV_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
}

static void controller_stage_conn_response(bool new_tx_pdu)
{
    radio_set_packet_ptr((uint32_t)(new_tx_pdu ? &m_ctrl_rt.conn_tx_pdu : &m_ctrl_rt.last_conn_tx_pdu));
    if (new_tx_pdu)
    {
        m_ctrl_rt.last_conn_tx_pdu = m_ctrl_rt.conn_tx_pdu;
        m_ctrl_rt.tx_unacked = true;
    }
}

static void controller_process_received_conn_pdu(void)
{
    ble_ll_data_raw_pdu_t rx_pdu = m_ctrl_rt.conn_rx_pdu;
    uint16_t ctrl_rsp_len = 0U;
    uint16_t att_rsp_len = 0U;
    uint8_t att_rsp[BLE_ATT_GATT_MAX_MTU];

    if (rx_pdu.header.llid == BLE_LLID_CONTROL_PDU)
    {
        if (rx_pdu.length == 0U)
        {
            return;
        }

        ctrl_rsp_len = ll_control_process(rx_pdu.payload, rx_pdu.length, m_ctrl_rt.pending_conn_tx_pdu.payload);
        if ((ctrl_rsp_len > 0U) && m_link.connected)
        {
            m_ctrl_rt.pending_conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTROL_PDU);
            m_ctrl_rt.pending_conn_tx_pdu.length = (uint8_t)ctrl_rsp_len;
            m_ctrl_rt.has_pending_conn_tx_pdu = true;
        }
        return;
    }

    if ((rx_pdu.header.llid == BLE_LLID_START_L2CAP) || (rx_pdu.header.llid == BLE_LLID_CONTINUATION))
    {
        if (rx_pdu.length >= BLE_L2CAP_HDR_LEN)
        {
            uint16_t l2cap_len = u16_decode(&rx_pdu.payload[0]);
            uint16_t l2cap_cid = u16_decode(&rx_pdu.payload[2]);

            if ((l2cap_cid == BLE_L2CAP_CID_ATT) && ((uint16_t)rx_pdu.length >= (uint16_t)(l2cap_len + BLE_L2CAP_HDR_LEN)))
            {
                att_rsp_len = ble_gatt_server_process_request(&rx_pdu.payload[4], l2cap_len, att_rsp, sizeof(att_rsp));
                if (att_rsp_len > 0U)
                {
                    (void)controller_queue_l2cap_payload(BLE_L2CAP_CID_ATT, att_rsp, att_rsp_len);
                }
            }
        }
    }
}

static void controller_prestage_conn_response_from_header(void)
{
    const ble_ll_data_header_t *p_rx_header = &m_ctrl_rt.conn_rx_pdu.header;
    bool tx_acked = m_ctrl_rt.tx_unacked && (p_rx_header->nesn != m_link.packet.tx_sn);
    bool is_new_packet = (p_rx_header->sn == m_link.packet.next_expected_rx_sn);
    uint8_t next_expected_rx_sn;
    uint8_t tx_sn;

    controller_reset_conn_bcmatch_state();
    m_ctrl_rt.conn_bcmatch.tx_acked = tx_acked;
    m_ctrl_rt.conn_bcmatch.is_new_packet = is_new_packet;

    if (m_ctrl_rt.tx_unacked && !tx_acked)
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

    if (m_ctrl_rt.has_pending_conn_tx_pdu)
    {
        m_ctrl_rt.conn_tx_pdu = m_ctrl_rt.pending_conn_tx_pdu;
        m_ctrl_rt.conn_tx_pdu.header = controller_conn_header_for_state(m_ctrl_rt.conn_tx_pdu.header.llid, next_expected_rx_sn, tx_sn);
        m_ctrl_rt.conn_bcmatch.consumes_pending = true;
    }
    else
    {
        m_ctrl_rt.conn_tx_pdu.header = controller_conn_header_for_state(BLE_LLID_CONTINUATION, next_expected_rx_sn, tx_sn);
        m_ctrl_rt.conn_tx_pdu.length = 0U;
    }
}

static void controller_start_connection_event(void)
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

void TIMER2_IRQHandler(void)
{
    uint32_t current_event_tick_us;
    uint32_t next_event_tick_us;

    if (NRF_TIMER2->EVENTS_COMPARE[0] == 0U)
    {
        return;
    }

    NRF_TIMER2->EVENTS_COMPARE[0] = 0U;

    if (!m_link.connected)
    {
        return;
    }

    current_event_tick_us = m_ctrl_rt.conn_next_event_tick_us;
    next_event_tick_us = current_event_tick_us + m_link.conn.conn_interval_us;

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
    if (m_link.pending_conn_update.valid && ((m_link.event_counter + 1) == m_link.pending_conn_update.instant))
    {
        next_event_tick_us += m_link.pending_conn_update.window_offset_us;
    }
    else if (m_link.pending_conn_update.valid && (m_link.event_counter == m_link.pending_conn_update.instant))
    {
        m_link.conn = m_link.pending_conn_update.conn;
        next_event_tick_us = current_event_tick_us + m_link.conn.conn_interval_us;

        m_link.pending_conn_update.valid = false;
        m_link.supervision.missed_interval_count = 0U;
        (void)ble_evt_notify_gap(BLE_GAP_EVT_CONN_UPDATE_IND);
    }

    m_ctrl_rt.conn_next_event_tick_us = next_event_tick_us;
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
        (void)ble_evt_notify_gap(BLE_GAP_EVT_PHY_UPDATE_IND);
    }

    controller_start_connection_event();
    m_link.event_counter++;
}

static void adv_timer_handler(void *p_context)
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

static bool controller_connect_request_targets_us(const ble_connect_req_pdu_t *p_req)
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

static void controller_apply_connect_request(const ble_connect_req_pdu_t *p_req)
{
    uint32_t first_event_delay_us;

    (void)memset(&m_link, 0, sizeof(m_link));
    m_link.connected = true;
    m_link.conn.conn_interval_ms = UNITS_1P25MS_TO_MS(p_req->ll_data.interval);
    m_link.conn.conn_interval_us = UNITS_1P25MS_TO_US(p_req->ll_data.interval);
    m_link.conn.slave_latency = p_req->ll_data.latency;
    m_link.conn.supervision_timeout_ms = UNITS_10MS_TO_MS(p_req->ll_data.timeout);
    m_link.channel.hop_increment = p_req->ll_data.hop_increment;
    if (m_link.channel.hop_increment == 0U)
    {
        m_link.channel.hop_increment = 5U;
    }
    m_link.packet.crc_init = (uint32_t)p_req->ll_data.crc_init[0] | ((uint32_t)p_req->ll_data.crc_init[1] << 8) | ((uint32_t)p_req->ll_data.crc_init[2] << 16);
    m_link.packet.max_tx_octets = BLE_LL_DATA_LEN_DEFAULT_OCTETS;
    m_link.packet.max_rx_octets = BLE_LL_DATA_LEN_DEFAULT_OCTETS;
    m_link.phy.tx_phy = BLE_LL_PHY_1M;
    m_link.phy.rx_phy = BLE_LL_PHY_1M;
    m_ctrl_rt.tx_unacked = false;
    m_ctrl_rt.has_pending_conn_tx_pdu = false;
    m_ctrl_rt.conn_rx_process_pending = false;
    m_ctrl_rt.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    controller_reset_conn_bcmatch_state();
    ble_gatt_server_reset_connection_state();

    (void)memcpy(m_link.packet.access_address, p_req->ll_data.access_address, sizeof(m_link.packet.access_address));
    controller_apply_channel_map(p_req->ll_data.channel_map);

    (void)app_timer_stop(m_adv_timer_id);
    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN);
    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.conn_rx_pdu.payload), m_link.packet.access_address, m_link.packet.crc_init, (uint32_t)&m_ctrl_rt.conn_rx_pdu);

    /* First slave listening window starts at transmitWindowOffset; for 0 offset,
       listen immediately (with a small guard) instead of waiting a full interval. */
    first_event_delay_us = (p_req->ll_data.win_offset == 0U) ? 2000U : UNITS_1P25MS_TO_US(p_req->ll_data.win_offset);
    controller_conn_timer_stop();
    m_ctrl_rt.conn_next_event_tick_us = first_event_delay_us;
    controller_conn_timer_schedule_compare();
    NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER2->TASKS_START = 1U;

    (void)ble_evt_notify_gap(BLE_GAP_EVT_CONNECTED);
}

static void radio_handle_connected_packet(void)
{
    bool new_tx_pdu;

    m_link.supervision.rx_seen_this_interval = true;
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

static void radio_handle_connected_crc_error(void)
{
    /*
     * A CRC error carries no valid SN/NESN, so keep RX/TX sequence state unchanged
     * and answer with the current NESN to request retransmission from the central.
     */
    bool new_tx_pdu;

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

static void radio_handle_connected_bcmatch(void)
{
    if (m_ctrl_rt.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    controller_prestage_conn_response_from_header();
}

static void controller_handle_connected_disabled(void)
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

static void controller_handle_advertising_disabled(void)
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
            controller_apply_connect_request(&m_ctrl_rt.adv_rx_pdu.connect_req);
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

static void radio_evt_handler(radio_event_t evt)
{
    const ble_adv_rx_pdu_t *p_rx = &m_ctrl_rt.adv_rx_pdu;

    if (m_link.connected)
    {
        if (evt == RADIO_EVENT_BCMATCH)
        {
            radio_handle_connected_bcmatch();
        }
        else if (evt == RADIO_EVENT_CRC_OK)
        {
            radio_handle_connected_packet();
        }
        else if (evt == RADIO_EVENT_CRC_ERROR)
        {
            radio_handle_connected_crc_error();
        }
        else
        {
            controller_handle_connected_disabled();
        }
        return;
    }

    if (evt == RADIO_EVENT_DISABLED)
    {
        controller_handle_advertising_disabled();
        return;
    }

    if (m_ctrl_rt.adv_radio_phase != BLE_ADV_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        return;
    }
    if (evt == RADIO_EVENT_CRC_OK)
    {
        if ((p_rx->adv.header.pdu_type == LL_SCAN_REQ) && controller_scan_request_targets_us(&p_rx->scan_req))
        {
            m_ctrl_rt.scan_rsp_pdu.header.rxadd = (uint8_t)(p_rx->scan_req.header.txadd & 0x01U);
            m_ctrl_rt.adv_scan_rsp_pending = true;
            return;
        }

        if ((p_rx->adv.header.pdu_type == LL_CONNECT_REQ) && controller_connect_request_targets_us(&p_rx->connect_req))
        {
            m_ctrl_rt.adv_connect_pending = true;
        }
    }
}
void ble_start_advertising(void)
{

    if (m_link.connected)
    {
        return;
    }

    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_ADV);

    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.adv_tx_pdu.payload), m_adv_access_address, m_adv_crc_init, (uint32_t)&m_ctrl_rt.adv_tx_pdu);

    APP_ERROR_CHECK(app_timer_stop(m_adv_timer_id));

    APP_ERROR_CHECK(app_timer_start(m_adv_timer_id, APP_TIMER_TICKS(m_host.adv_interval_ms), NULL));
}

void controller_runtime_init(void)
{

    APP_ERROR_CHECK(app_timer_init());

    APP_ERROR_CHECK(app_timer_create(&m_adv_timer_id, APP_TIMER_MODE_REPEATED, adv_timer_handler));

    radio_power_on();
    radio_configure_modecnf0(RADIO_RAMP_UP_DEFAULT, RADIO_DEFAULT_TX_B1);
    radio_set_tifs(150U);
    controller_conn_timer_init();
    radio_set_event_handler(radio_evt_handler);
    radio_enable_interrupt_mask(0U);
}
