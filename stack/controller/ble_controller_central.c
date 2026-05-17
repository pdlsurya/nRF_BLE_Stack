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

#define BLE_INITIATOR_CONN_INTERVAL_UNITS_DEFAULT 24U
#define BLE_INITIATOR_SUPERVISION_TIMEOUT_UNITS_DEFAULT 400U

APP_TIMER_DEF(m_scan_timer_id);
APP_TIMER_DEF(m_scan_window_timer_id);

static void controller_central_reset_scan_state(void);
static void controller_central_auto_ctrl_complete(ble_gap_ctrl_procedure_t procedure);
static uint8_t controller_central_ctrl_proc_request_opcode(ble_gap_ctrl_procedure_t procedure);
static void controller_central_build_scan_request(const ble_gap_addr_t *p_peer_addr);

static void controller_central_ctrl_proc_reset(void)
{
    m_ctrl_rt.central.central_ctrl_proc.procedure = BLE_GAP_CTRL_PROC_NONE;
    m_ctrl_rt.central.central_ctrl_proc.state = BLE_CENTRAL_CTRL_PROC_STATE_IDLE;
}

void controller_central_state_reset(void)
{
    m_ctrl_rt.central.scanning = false;
    m_ctrl_rt.central.connect_target_valid = false;
    m_ctrl_rt.central.scan_rsp_connect_candidate_valid = false;
    m_ctrl_rt.central.scan_channel_index = 0U;
    controller_central_reset_scan_state();
    controller_central_ctrl_proc_reset();
    m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
}

void controller_central_handle_feature_exchange_complete(void)
{
    if (m_link.role != BLE_GAP_ROLE_CENTRAL)
    {
        return;
    }

    if ((m_ctrl_rt.central.central_ctrl_proc.procedure == BLE_GAP_CTRL_PROC_FEATURE_EXCHANGE) &&
        (m_ctrl_rt.central.central_ctrl_proc.state == BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP))
    {
        controller_central_ctrl_proc_reset();
    }

    controller_central_auto_ctrl_complete(BLE_GAP_CTRL_PROC_FEATURE_EXCHANGE);
}

void controller_central_handle_data_length_update_complete(void)
{
    if (m_link.role != BLE_GAP_ROLE_CENTRAL)
    {
        return;
    }

    if ((m_ctrl_rt.central.central_ctrl_proc.procedure == BLE_GAP_CTRL_PROC_DATA_LENGTH_UPDATE) &&
        (m_ctrl_rt.central.central_ctrl_proc.state == BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP))
    {
        controller_central_ctrl_proc_reset();
    }

    controller_central_auto_ctrl_complete(BLE_GAP_CTRL_PROC_DATA_LENGTH_UPDATE);
}

void controller_central_handle_unknown_rsp(uint8_t unsupported_opcode)
{
    ble_gap_ctrl_procedure_t procedure;

    if (m_link.role != BLE_GAP_ROLE_CENTRAL)
    {
        return;
    }

    if ((m_ctrl_rt.central.central_ctrl_proc.state != BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP) ||
        (unsupported_opcode != controller_central_ctrl_proc_request_opcode(m_ctrl_rt.central.central_ctrl_proc.procedure)))
    {
        return;
    }

    procedure = m_ctrl_rt.central.central_ctrl_proc.procedure;
    controller_central_ctrl_proc_reset();
    (void)ble_evt_notify_gap_ctrl_procedure_unsupported(procedure, unsupported_opcode);
    controller_central_auto_ctrl_complete(procedure);
}

void controller_central_handle_conn_update_instant_complete(void)
{
    if (m_link.role != BLE_GAP_ROLE_CENTRAL)
    {
        return;
    }

    if ((m_ctrl_rt.central.central_ctrl_proc.procedure == BLE_GAP_CTRL_PROC_CONN_UPDATE) &&
        (m_ctrl_rt.central.central_ctrl_proc.state == BLE_CENTRAL_CTRL_PROC_STATE_WAIT_INSTANT))
    {
        controller_central_ctrl_proc_reset();
    }
}

void controller_central_handle_phy_update_instant_complete(void)
{
    if (m_link.role != BLE_GAP_ROLE_CENTRAL)
    {
        return;
    }

    if ((m_ctrl_rt.central.central_ctrl_proc.procedure == BLE_GAP_CTRL_PROC_PHY_UPDATE) &&
        (m_ctrl_rt.central.central_ctrl_proc.state == BLE_CENTRAL_CTRL_PROC_STATE_WAIT_INSTANT))
    {
        controller_central_ctrl_proc_reset();
    }

    controller_central_auto_ctrl_complete(BLE_GAP_CTRL_PROC_PHY_UPDATE);
}

static uint8_t controller_central_ctrl_proc_request_opcode(ble_gap_ctrl_procedure_t procedure)
{
    switch (procedure)
    {
    case BLE_GAP_CTRL_PROC_FEATURE_EXCHANGE:
        return BLE_LL_CTRL_FEATURE_REQ;

    case BLE_GAP_CTRL_PROC_DATA_LENGTH_UPDATE:
        return BLE_LL_CTRL_LENGTH_REQ;

    case BLE_GAP_CTRL_PROC_PHY_UPDATE:
        return BLE_LL_CTRL_PHY_REQ;

    default:
        return 0U;
    }
}

static bool controller_central_request_feature_exchange(void)
{
    uint8_t pdu[9];

    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) ||
        !m_link.connected ||
        (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE))
    {
        return false;
    }

    pdu[0] = BLE_LL_CTRL_FEATURE_REQ;
    (void)memset(&pdu[1], 0, sizeof(pdu) - 1U);
    pdu[1] = BLE_LL_FEATURE_DATA_LENGTH_EXTENSION;
    pdu[2] = BLE_LL_FEATURE_2M_PHY;
    if (!controller_queue_ll_control_payload(pdu, sizeof(pdu)))
    {
        return false;
    }

    m_ctrl_rt.central.central_ctrl_proc.procedure = BLE_GAP_CTRL_PROC_FEATURE_EXCHANGE;
    m_ctrl_rt.central.central_ctrl_proc.state = BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP;
    return true;
}

static bool controller_central_request_data_length_update(void)
{
    uint8_t pdu[9];

    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) ||
        !m_link.connected ||
        (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE))
    {
        return false;
    }

    pdu[0] = BLE_LL_CTRL_LENGTH_REQ;
    u16_encode(BLE_LL_DATA_LEN_MAX_OCTETS, &pdu[1]);
    u16_encode(BLE_LL_DATA_LEN_MAX_TIME, &pdu[3]);
    u16_encode(BLE_LL_DATA_LEN_MAX_OCTETS, &pdu[5]);
    u16_encode(BLE_LL_DATA_LEN_MAX_TIME, &pdu[7]);
    if (!controller_queue_ll_control_payload(pdu, sizeof(pdu)))
    {
        return false;
    }

    m_ctrl_rt.central.central_ctrl_proc.procedure = BLE_GAP_CTRL_PROC_DATA_LENGTH_UPDATE;
    m_ctrl_rt.central.central_ctrl_proc.state = BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP;
    return true;
}

static bool controller_central_request_phy_update(uint8_t tx_phys, uint8_t rx_phys)
{
    uint8_t pdu[3];
    uint8_t requested_tx_phys = (uint8_t)(tx_phys & (BLE_LL_PHY_1M | BLE_LL_PHY_2M));
    uint8_t requested_rx_phys = (uint8_t)(rx_phys & (BLE_LL_PHY_1M | BLE_LL_PHY_2M));

    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) ||
        !m_link.connected ||
        (requested_tx_phys == 0U) ||
        (requested_rx_phys == 0U) ||
        (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE) ||
        m_link.pending_phy_update.valid)
    {
        return false;
    }

    pdu[0] = BLE_LL_CTRL_PHY_REQ;
    pdu[1] = requested_tx_phys;
    pdu[2] = requested_rx_phys;
    if (!controller_queue_ll_control_payload(pdu, sizeof(pdu)))
    {
        return false;
    }

    m_ctrl_rt.central.central_ctrl_proc.procedure = BLE_GAP_CTRL_PROC_PHY_UPDATE;
    m_ctrl_rt.central.central_ctrl_proc.state = BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP;
    return true;
}

static void controller_central_auto_ctrl_advance(void)
{
    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) || !m_link.connected)
    {
        m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
        return;
    }

    if (m_ctrl_rt.central.auto_ctrl_stage == BLE_AUTO_CTRL_STAGE_WAIT_FEATURES)
    {
        if (!controller_central_request_feature_exchange())
        {
            if (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE)
            {
                return;
            }

            m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_WAIT_DATA_LENGTH;
            controller_central_auto_ctrl_advance();
        }
        return;
    }

    if (m_ctrl_rt.central.auto_ctrl_stage == BLE_AUTO_CTRL_STAGE_WAIT_DATA_LENGTH)
    {
        if (!controller_central_request_data_length_update())
        {
            if (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE)
            {
                return;
            }

            m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_WAIT_PHY;
            controller_central_auto_ctrl_advance();
        }
        return;
    }

    if (m_ctrl_rt.central.auto_ctrl_stage == BLE_AUTO_CTRL_STAGE_WAIT_PHY)
    {
        if (!controller_central_request_phy_update((uint8_t)(BLE_LL_PHY_1M | BLE_LL_PHY_2M),
                                                   (uint8_t)(BLE_LL_PHY_1M | BLE_LL_PHY_2M)))
        {
            if ((m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE) ||
                m_link.pending_phy_update.valid)
            {
                return;
            }

            m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
        }
        return;
    }

    m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
}

void controller_central_auto_ctrl_start(void)
{
    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) || !m_link.connected)
    {
        m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
        return;
    }

    m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_WAIT_FEATURES;
    controller_central_auto_ctrl_advance();
}

static void controller_central_auto_ctrl_complete(ble_gap_ctrl_procedure_t procedure)
{
    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) || !m_link.connected)
    {
        m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
        return;
    }

    if ((procedure == BLE_GAP_CTRL_PROC_FEATURE_EXCHANGE) &&
        (m_ctrl_rt.central.auto_ctrl_stage == BLE_AUTO_CTRL_STAGE_WAIT_FEATURES))
    {
        m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_WAIT_DATA_LENGTH;
        controller_central_auto_ctrl_advance();
        return;
    }

    if ((procedure == BLE_GAP_CTRL_PROC_DATA_LENGTH_UPDATE) &&
        (m_ctrl_rt.central.auto_ctrl_stage == BLE_AUTO_CTRL_STAGE_WAIT_DATA_LENGTH))
    {
        m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_WAIT_PHY;
        controller_central_auto_ctrl_advance();
        return;
    }

    if ((procedure == BLE_GAP_CTRL_PROC_PHY_UPDATE) &&
        (m_ctrl_rt.central.auto_ctrl_stage == BLE_AUTO_CTRL_STAGE_WAIT_PHY))
    {
        m_ctrl_rt.central.auto_ctrl_stage = BLE_AUTO_CTRL_STAGE_IDLE;
    }
}

bool controller_central_initiate_conn_update(const ble_gap_conn_params_t *p_params)
{
    uint8_t pdu[12];
    uint16_t instant;

    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) ||
        !m_link.connected ||
        (p_params == NULL) ||
        (p_params->min_conn_interval_1p25ms == 0U) ||
        (p_params->max_conn_interval_1p25ms == 0U) ||
        (p_params->min_conn_interval_1p25ms > p_params->max_conn_interval_1p25ms) ||
        (p_params->supervision_timeout_10ms == 0U) ||
        (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_NONE) ||
        m_link.pending_conn_update.valid)
    {
        return false;
    }

    instant = (uint16_t)(m_link.event_counter + BLE_LL_CTRL_INSTANT_OFFSET_EVENTS);
    pdu[0] = BLE_LL_CTRL_CONN_UPDATE_IND;
    pdu[1] = BLE_LL_CTRL_CONN_UPDATE_WIN_SIZE_UNITS;
    u16_encode(0U, &pdu[2]);
    u16_encode(p_params->min_conn_interval_1p25ms, &pdu[4]);
    u16_encode(p_params->slave_latency, &pdu[6]);
    u16_encode(p_params->supervision_timeout_10ms, &pdu[8]);
    u16_encode(instant, &pdu[10]);
    if (!controller_queue_ll_control_payload(pdu, sizeof(pdu)))
    {
        return false;
    }

    controller_schedule_conn_update(pdu, sizeof(pdu));
    m_ctrl_rt.central.central_ctrl_proc.procedure = BLE_GAP_CTRL_PROC_CONN_UPDATE;
    m_ctrl_rt.central.central_ctrl_proc.state = BLE_CENTRAL_CTRL_PROC_STATE_WAIT_INSTANT;
    return true;
}

uint16_t controller_central_process_phy_rsp(const uint8_t *p_payload, uint8_t len, uint8_t *p_rsp)
{
    uint8_t symmetric_phys;
    uint8_t next_phy;
    uint16_t instant;

    if ((m_link.role != BLE_GAP_ROLE_CENTRAL) ||
        (m_ctrl_rt.central.central_ctrl_proc.procedure != BLE_GAP_CTRL_PROC_PHY_UPDATE) ||
        (m_ctrl_rt.central.central_ctrl_proc.state != BLE_CENTRAL_CTRL_PROC_STATE_WAIT_RSP) ||
        (p_rsp == NULL) ||
        (len < 3U))
    {
        return 0U;
    }

    symmetric_phys = (uint8_t)(p_payload[1] & p_payload[2] & (BLE_LL_PHY_1M | BLE_LL_PHY_2M));
    next_phy = ((symmetric_phys & BLE_LL_PHY_2M) != 0U) ? BLE_LL_PHY_2M : BLE_LL_PHY_1M;
    instant = (uint16_t)(m_link.event_counter + BLE_LL_CTRL_INSTANT_OFFSET_EVENTS);

    p_rsp[0] = BLE_LL_CTRL_PHY_UPDATE_IND;
    p_rsp[1] = next_phy;
    p_rsp[2] = next_phy;
    u16_encode(instant, &p_rsp[3]);
    m_link.pending_phy_update.phy.tx_phy = next_phy;
    m_link.pending_phy_update.phy.rx_phy = next_phy;
    m_link.pending_phy_update.instant = instant;
    m_link.pending_phy_update.valid = true;
    m_ctrl_rt.central.central_ctrl_proc.state = BLE_CENTRAL_CTRL_PROC_STATE_WAIT_INSTANT;
    return 5U;
}

static bool controller_central_adv_type_is_connectable(ble_ll_adv_pdu_type_t adv_type)
{
    return (adv_type == LL_ADV_IND) || (adv_type == LL_ADV_DIRECT_IND);
}

static bool controller_central_adv_type_is_scannable(ble_ll_adv_pdu_type_t adv_type)
{
    return (adv_type == LL_ADV_IND) || (adv_type == LL_ADV_SCAN_IND);
}

static bool controller_central_adv_type_is_reportable(ble_ll_adv_pdu_type_t adv_type)
{
    return (adv_type == LL_ADV_IND) ||
           (adv_type == LL_ADV_DIRECT_IND) ||
           (adv_type == LL_ADV_NONCONN_IND) ||
           (adv_type == LL_ADV_SCAN_IND);
}

static bool controller_central_scan_addr_equal(const ble_gap_addr_t *p_lhs, const ble_gap_addr_t *p_rhs)
{
    if ((p_lhs == NULL) || (p_rhs == NULL))
    {
        return false;
    }

    return (p_lhs->is_random == p_rhs->is_random) &&
           (memcmp(p_lhs->addr, p_rhs->addr, sizeof(p_lhs->addr)) == 0);
}

static bool controller_central_scan_addr_matches_pending_connect_candidate(const ble_gap_addr_t *p_addr)
{
    if ((p_addr == NULL) || !m_ctrl_rt.central.scan_rsp_connect_candidate_valid)
    {
        return false;
    }

    return controller_central_scan_addr_equal(p_addr, &m_ctrl_rt.central.scan_rsp_connect_candidate);
}

static bool controller_central_scan_addr_matches_filter(const ble_gap_addr_t *p_addr)
{
    if ((p_addr == NULL) || !m_ctrl_rt.central.scan_filter_enabled || !m_ctrl_rt.central.scan_filter.match_addr)
    {
        return false;
    }

    return (p_addr->is_random == m_ctrl_rt.central.scan_filter.addr.is_random) &&
           (memcmp(p_addr->addr, m_ctrl_rt.central.scan_filter.addr.addr, sizeof(p_addr->addr)) == 0);
}

static bool controller_central_adv_data_next(const uint8_t *p_adv_data,
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

static bool controller_central_adv_data_matches_name(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    const uint8_t *p_value;
    uint8_t offset = 0U;
    uint8_t ad_type;
    uint8_t value_len;
    size_t filter_name_len = 0U;

    while ((filter_name_len < sizeof(m_ctrl_rt.central.scan_filter.name)) &&
           (m_ctrl_rt.central.scan_filter.name[filter_name_len] != '\0'))
    {
        filter_name_len++;
    }

    if (filter_name_len == 0U)
    {
        return false;
    }

    while (controller_central_adv_data_next(p_adv_data, adv_data_len, &offset, &ad_type, &p_value, &value_len))
    {
        if ((ad_type != BLE_AD_TYPE_SHORT_LOCAL_NAME) &&
            (ad_type != BLE_AD_TYPE_COMPLETE_LOCAL_NAME))
        {
            continue;
        }

        if ((value_len == filter_name_len) &&
            (memcmp(p_value, m_ctrl_rt.central.scan_filter.name, filter_name_len) == 0))
        {
            return true;
        }
    }

    return false;
}

static bool controller_central_adv_data_contains_uuid_bytes(const uint8_t *p_adv_data,
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

    while (controller_central_adv_data_next(p_adv_data, adv_data_len, &offset, &ad_type, &p_value, &value_len))
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

static bool controller_central_adv_data_contains_service_uuid16(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    uint8_t uuid16[BLE_UUID16_LEN];

    u16_encode(m_ctrl_rt.central.scan_filter.service_uuid16, uuid16);
    return controller_central_adv_data_contains_uuid_bytes(p_adv_data,
                                                           adv_data_len,
                                                           uuid16,
                                                           BLE_UUID16_LEN,
                                                           BLE_AD_TYPE_INCOMPLETE_UUID16_LIST,
                                                           BLE_AD_TYPE_COMPLETE_UUID16_LIST);
}

static bool controller_central_adv_data_contains_service_uuid128(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    return controller_central_adv_data_contains_uuid_bytes(p_adv_data,
                                                           adv_data_len,
                                                           m_ctrl_rt.central.scan_filter.service_uuid128,
                                                           BLE_UUID128_LEN,
                                                           BLE_AD_TYPE_INCOMPLETE_UUID128_LIST,
                                                           BLE_AD_TYPE_COMPLETE_UUID128_LIST);
}

static bool controller_central_scan_filter_matches(const ble_gap_addr_t *p_addr, const ble_ll_adv_pdu_t *p_adv)
{
    const uint8_t *p_adv_data;
    uint8_t adv_data_len;

    if ((p_addr == NULL) || (p_adv == NULL) || !m_ctrl_rt.central.scan_filter_enabled)
    {
        return false;
    }

    if (m_ctrl_rt.central.scan_filter.match_addr &&
        !controller_central_scan_addr_matches_filter(p_addr))
    {
        return false;
    }

    adv_data_len = (p_adv->payload_length > BLE_ADV_ADVERTISER_ADDRESS_LEN)
                       ? (uint8_t)(p_adv->payload_length - BLE_ADV_ADVERTISER_ADDRESS_LEN)
                       : 0U;
    p_adv_data = p_adv->payload;

    if (m_ctrl_rt.central.scan_filter.match_name &&
        !controller_central_adv_data_matches_name(p_adv_data, adv_data_len))
    {
        return false;
    }

    if (m_ctrl_rt.central.scan_filter.match_service_uuid16 &&
        !controller_central_adv_data_contains_service_uuid16(p_adv_data, adv_data_len))
    {
        return false;
    }

    if (m_ctrl_rt.central.scan_filter.match_service_uuid128 &&
        !controller_central_adv_data_contains_service_uuid128(p_adv_data, adv_data_len))
    {
        return false;
    }

    return true;
}

static bool controller_central_scan_rsp_filter_matches(const ble_gap_addr_t *p_addr,
                                                       const ble_ll_scan_rsp_pdu_t *p_scan_rsp)
{
    const uint8_t *p_adv_data;
    uint8_t adv_data_len;

    if ((p_addr == NULL) || (p_scan_rsp == NULL) || !m_ctrl_rt.central.scan_filter_enabled)
    {
        return false;
    }

    if (m_ctrl_rt.central.scan_filter.match_addr &&
        !controller_central_scan_addr_matches_filter(p_addr))
    {
        return false;
    }

    adv_data_len = (p_scan_rsp->payload_length > BLE_ADV_ADVERTISER_ADDRESS_LEN)
                       ? (uint8_t)(p_scan_rsp->payload_length - BLE_ADV_ADVERTISER_ADDRESS_LEN)
                       : 0U;
    p_adv_data = p_scan_rsp->payload;

    if (m_ctrl_rt.central.scan_filter.match_name &&
        !controller_central_adv_data_matches_name(p_adv_data, adv_data_len))
    {
        return false;
    }

    if (m_ctrl_rt.central.scan_filter.match_service_uuid16 &&
        !controller_central_adv_data_contains_service_uuid16(p_adv_data, adv_data_len))
    {
        return false;
    }

    if (m_ctrl_rt.central.scan_filter.match_service_uuid128 &&
        !controller_central_adv_data_contains_service_uuid128(p_adv_data, adv_data_len))
    {
        return false;
    }

    return true;
}

static uint32_t controller_central_seed_next(uint32_t *p_state)
{
    *p_state = (*p_state * 1664525UL) + 1013904223UL;
    return *p_state;
}

static uint32_t controller_central_access_address_generate(const ble_gap_addr_t *p_peer_addr)
{
    uint32_t seed;
    uint8_t i;

    seed = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1] ^ 0xA55A3C5DU;
    if (p_peer_addr != NULL)
    {
        for (i = 0U; i < sizeof(p_peer_addr->addr); i++)
        {
            seed ^= ((uint32_t)p_peer_addr->addr[i]) << ((i % 4U) * 8U);
            seed = controller_central_seed_next(&seed);
        }
    }

    if ((seed == 0U) || (seed == 0x8E89BED6UL))
    {
        seed ^= 0xC3A55A3CU;
    }

    return seed;
}

static void controller_central_build_connect_request(const ble_gap_addr_t *p_peer_addr)
{
    uint32_t access_address;
    uint32_t crc_init;
    uint16_t conn_interval_units;
    uint16_t supervision_timeout_units;
    uint8_t i;

    (void)memset(&m_ctrl_rt.central.connect_req_pdu, 0, sizeof(m_ctrl_rt.central.connect_req_pdu));
    m_ctrl_rt.central.connect_req_pdu.header.pdu_type = LL_CONNECT_REQ;
    m_ctrl_rt.central.connect_req_pdu.header.rfu = 0U;
    m_ctrl_rt.central.connect_req_pdu.header.txadd = (uint8_t)(m_ctrl_rt.local_addr.txadd & 0x01U);
    m_ctrl_rt.central.connect_req_pdu.header.rxadd = (uint8_t)(p_peer_addr->is_random ? 1U : 0U);
    (void)memcpy(m_ctrl_rt.central.connect_req_pdu.initiator_address,
                 m_ctrl_rt.local_addr.addr,
                 sizeof(m_ctrl_rt.central.connect_req_pdu.initiator_address));
    (void)memcpy(m_ctrl_rt.central.connect_req_pdu.advertiser_address,
                 p_peer_addr->addr,
                 sizeof(m_ctrl_rt.central.connect_req_pdu.advertiser_address));

    access_address = controller_central_access_address_generate(p_peer_addr);
    crc_init = controller_central_seed_next(&access_address) & 0x00FFFFFFUL;
    if (crc_init == 0U)
    {
        crc_init = 0x00ABCDE1UL;
    }

    conn_interval_units = m_host.common.preferred_conn_params_valid
                              ? m_host.common.preferred_conn_params.min_conn_interval_1p25ms
                              : BLE_INITIATOR_CONN_INTERVAL_UNITS_DEFAULT;
    supervision_timeout_units = m_host.common.preferred_conn_params_valid
                                    ? m_host.common.preferred_conn_params.supervision_timeout_10ms
                                    : BLE_INITIATOR_SUPERVISION_TIMEOUT_UNITS_DEFAULT;

    m_ctrl_rt.central.connect_req_pdu.payload_length = (uint8_t)(sizeof(m_ctrl_rt.central.connect_req_pdu.initiator_address) +
                                                                 sizeof(m_ctrl_rt.central.connect_req_pdu.advertiser_address) +
                                                                 sizeof(m_ctrl_rt.central.connect_req_pdu.ll_data));
    (void)memcpy(m_ctrl_rt.central.connect_req_pdu.ll_data.access_address,
                 &access_address,
                 sizeof(m_ctrl_rt.central.connect_req_pdu.ll_data.access_address));
    m_ctrl_rt.central.connect_req_pdu.ll_data.crc_init[0] = (uint8_t)(crc_init & 0xFFU);
    m_ctrl_rt.central.connect_req_pdu.ll_data.crc_init[1] = (uint8_t)((crc_init >> 8) & 0xFFU);
    m_ctrl_rt.central.connect_req_pdu.ll_data.crc_init[2] = (uint8_t)((crc_init >> 16) & 0xFFU);
    m_ctrl_rt.central.connect_req_pdu.ll_data.win_size = BLE_INITIATOR_WIN_SIZE_UNITS;
    m_ctrl_rt.central.connect_req_pdu.ll_data.win_offset = BLE_INITIATOR_WIN_OFFSET_UNITS;
    m_ctrl_rt.central.connect_req_pdu.ll_data.interval = conn_interval_units;
    m_ctrl_rt.central.connect_req_pdu.ll_data.latency = m_host.common.preferred_conn_params_valid ? m_host.common.preferred_conn_params.slave_latency : 0U;
    m_ctrl_rt.central.connect_req_pdu.ll_data.timeout = supervision_timeout_units;
    for (i = 0U; i < 5U; i++)
    {
        m_ctrl_rt.central.connect_req_pdu.ll_data.channel_map[i] = 0xFFU;
    }
    m_ctrl_rt.central.connect_req_pdu.ll_data.channel_map[4] = 0x1FU;
    m_ctrl_rt.central.connect_req_pdu.ll_data.hop_increment = BLE_INITIATOR_HOP_INCREMENT;
    m_ctrl_rt.central.connect_req_pdu.ll_data.sca = 0U;
}

static void controller_central_publish_scan_report_internal(const ble_gap_addr_t *p_addr,
                                                            ble_ll_adv_pdu_type_t adv_type,
                                                            bool scan_response,
                                                            bool connectable,
                                                            bool scannable,
                                                            const uint8_t *p_payload,
                                                            uint8_t payload_length)
{
    ble_gap_scan_report_t report;
    uint8_t data_len = 0U;

    if ((p_addr == NULL) || ((p_payload == NULL) && (payload_length != 0U)))
    {
        return;
    }

    (void)memset(&report, 0, sizeof(report));
    report.addr = *p_addr;
    report.adv_type = adv_type;
    report.scan_response = scan_response;
    report.connectable = connectable;
    report.scannable = scannable;
    report.rssi = 0;

    if (payload_length > BLE_ADV_ADVERTISER_ADDRESS_LEN)
    {
        data_len = (uint8_t)(payload_length - BLE_ADV_ADVERTISER_ADDRESS_LEN);
        if (data_len > BLE_GAP_ADV_DATA_MAX_LEN)
        {
            data_len = BLE_GAP_ADV_DATA_MAX_LEN;
        }
        (void)memcpy(report.data, p_payload, data_len);
        report.data_len = data_len;
    }

    (void)ble_evt_notify_scan_report(&report);
}

static void controller_central_publish_scan_report(const ble_gap_addr_t *p_addr, const ble_ll_adv_pdu_t *p_adv)
{
    ble_ll_adv_pdu_type_t adv_type;

    if ((p_addr == NULL) || (p_adv == NULL))
    {
        return;
    }

    adv_type = (ble_ll_adv_pdu_type_t)p_adv->header.pdu_type;
    if (!controller_central_adv_type_is_reportable(adv_type))
    {
        return;
    }

    controller_central_publish_scan_report_internal(p_addr,
                                                    adv_type,
                                                    false,
                                                    controller_central_adv_type_is_connectable(adv_type),
                                                    controller_central_adv_type_is_scannable(adv_type),
                                                    p_adv->payload,
                                                    p_adv->payload_length);
}

static void controller_central_publish_scan_rsp_report(const ble_gap_addr_t *p_addr,
                                                       const ble_ll_scan_rsp_pdu_t *p_scan_rsp)
{
    if ((p_addr == NULL) || (p_scan_rsp == NULL))
    {
        return;
    }

    controller_central_publish_scan_report_internal(p_addr,
                                                    LL_SCAN_RSP,
                                                    true,
                                                    false,
                                                    false,
                                                    p_scan_rsp->payload,
                                                    p_scan_rsp->payload_length);
}

static bool controller_central_radio_has_pending_completion(void)
{
    return (NRF_RADIO->EVENTS_END != 0U) ||
           (NRF_RADIO->EVENTS_CRCOK != 0U) ||
           (NRF_RADIO->EVENTS_CRCERROR != 0U) ||
           (NRF_RADIO->EVENTS_DISABLED != 0U);
}

static bool controller_central_radio_is_rx_window_active(void)
{
    radio_state_t state = radio_get_state();

    return (state == RX_RU) ||
           (state == RX_IDLE) ||
           (state == RX);
}

static void controller_central_reset_scan_state(void)
{
    m_ctrl_rt.central.scan_req_tx_pending = false;
    m_ctrl_rt.central.connect_req_tx_pending = false;
    m_ctrl_rt.central.scan_rsp_target_valid = false;
    m_ctrl_rt.central.scan_rsp_target_connectable = false;
    m_ctrl_rt.central.scan_radio_phase = BLE_SCAN_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
}

static void controller_central_build_scan_request(const ble_gap_addr_t *p_peer_addr)
{
    if (p_peer_addr == NULL)
    {
        return;
    }

    (void)memset(&m_ctrl_rt.central.scan_req_pdu, 0, sizeof(m_ctrl_rt.central.scan_req_pdu));
    m_ctrl_rt.central.scan_req_pdu.header.pdu_type = LL_SCAN_REQ;
    m_ctrl_rt.central.scan_req_pdu.header.rfu = 0U;
    m_ctrl_rt.central.scan_req_pdu.header.txadd = (uint8_t)(m_ctrl_rt.local_addr.txadd & 0x01U);
    m_ctrl_rt.central.scan_req_pdu.header.rxadd = (uint8_t)(p_peer_addr->is_random ? 1U : 0U);
    m_ctrl_rt.central.scan_req_pdu.payload_length = BLE_SCAN_REQ_PAYLOAD_LEN;
    (void)memcpy(m_ctrl_rt.central.scan_req_pdu.scanner_address,
                 m_ctrl_rt.local_addr.addr,
                 sizeof(m_ctrl_rt.central.scan_req_pdu.scanner_address));
    (void)memcpy(m_ctrl_rt.central.scan_req_pdu.advertiser_address,
                 p_peer_addr->addr,
                 sizeof(m_ctrl_rt.central.scan_req_pdu.advertiser_address));
}

static bool controller_central_scan_response_targets_pending_request(const ble_ll_scan_rsp_pdu_t *p_scan_rsp)
{
    if ((p_scan_rsp == NULL) || !m_ctrl_rt.central.scan_rsp_target_valid)
    {
        return false;
    }

    if (p_scan_rsp->header.pdu_type != LL_SCAN_RSP)
    {
        return false;
    }

    if (p_scan_rsp->payload_length < BLE_ADV_ADVERTISER_ADDRESS_LEN)
    {
        return false;
    }

    if (p_scan_rsp->header.txadd != (uint8_t)(m_ctrl_rt.central.scan_rsp_target.is_random ? 1U : 0U))
    {
        return false;
    }

    if (p_scan_rsp->header.rxadd != (m_ctrl_rt.local_addr.txadd & 0x01U))
    {
        return false;
    }

    return memcmp(p_scan_rsp->advertiser_address,
                  m_ctrl_rt.central.scan_rsp_target.addr,
                  sizeof(m_ctrl_rt.central.scan_rsp_target.addr)) == 0;
}

static void controller_central_handle_scan_rsp_crc_ok(const ble_ll_scan_rsp_pdu_t *p_scan_rsp)
{
    if (!controller_central_scan_response_targets_pending_request(p_scan_rsp))
    {
        return;
    }

    controller_central_publish_scan_rsp_report(&m_ctrl_rt.central.scan_rsp_target, p_scan_rsp);

    if (m_ctrl_rt.central.scan_rsp_target_connectable &&
        m_ctrl_rt.central.scan_filter_enabled &&
        controller_central_scan_rsp_filter_matches(&m_ctrl_rt.central.scan_rsp_target, p_scan_rsp))
    {
        m_ctrl_rt.central.scan_rsp_connect_candidate = m_ctrl_rt.central.scan_rsp_target;
        m_ctrl_rt.central.scan_rsp_connect_candidate_valid = true;
    }
}

static void controller_central_handle_scan_crc_ok(const ble_ll_adv_pdu_t *p_adv_rx)
{
    ble_ll_adv_pdu_type_t adv_type;
    ble_gap_addr_t peer_addr;

    if (p_adv_rx == NULL)
    {
        return;
    }

    adv_type = (ble_ll_adv_pdu_type_t)p_adv_rx->header.pdu_type;
    if (!controller_central_adv_type_is_reportable(adv_type))
    {
        return;
    }

    peer_addr.is_random = (p_adv_rx->header.txadd != 0U);
    (void)memcpy(peer_addr.addr, p_adv_rx->advertiser_address, sizeof(peer_addr.addr));
    controller_central_publish_scan_report(&peer_addr, p_adv_rx);

    if (controller_central_scan_addr_matches_pending_connect_candidate(&peer_addr) &&
        controller_central_adv_type_is_connectable(adv_type))
    {
        controller_central_scan_timer_stop();
        m_ctrl_rt.central.connect_target = peer_addr;
        m_ctrl_rt.central.connect_target_valid = true;
        m_ctrl_rt.central.scan_rsp_connect_candidate_valid = false;
        m_ctrl_rt.central.scanning = false;
        controller_central_build_connect_request(&peer_addr);
        m_ctrl_rt.central.connect_req_tx_pending = true;
        return;
    }

    if (m_ctrl_rt.central.scan_filter_enabled &&
        controller_central_scan_filter_matches(&peer_addr, p_adv_rx) &&
        controller_central_adv_type_is_connectable(adv_type))
    {
        controller_central_scan_timer_stop();
        m_ctrl_rt.central.connect_target = peer_addr;
        m_ctrl_rt.central.connect_target_valid = true;
        m_ctrl_rt.central.scanning = false;
        controller_central_build_connect_request(&peer_addr);
        m_ctrl_rt.central.connect_req_tx_pending = true;
        m_ctrl_rt.central.scan_rsp_connect_candidate_valid = false;
        return;
    }

    if (m_host.central.scan_config.active &&
        controller_central_adv_type_is_scannable(adv_type))
    {
        controller_central_build_scan_request(&peer_addr);
        m_ctrl_rt.central.scan_rsp_target = peer_addr;
        m_ctrl_rt.central.scan_rsp_target_valid = true;
        m_ctrl_rt.central.scan_rsp_target_connectable =
            controller_central_adv_type_is_connectable(adv_type);
        m_ctrl_rt.central.scan_req_tx_pending = true;
    }
}

static void controller_central_scan_timer_start(uint32_t interval_ms)
{
    APP_ERROR_CHECK(app_timer_start(m_scan_timer_id, APP_TIMER_TICKS(interval_ms), NULL));
}

void controller_central_scan_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_scan_timer_id));
}

static void controller_central_scan_window_timer_start(uint32_t window_ms)
{
    APP_ERROR_CHECK(app_timer_start(m_scan_window_timer_id, APP_TIMER_TICKS(window_ms), NULL));
}

void controller_central_scan_window_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_scan_window_timer_id));
}

static void controller_central_scan_window_timeout_handler(void *p_context)
{
    uint32_t primask;

    (void)p_context;

    primask = irq_lock();
    if (m_ctrl_rt.central.scanning &&
        !m_link.connected &&
        ((m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED) ||
         (m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_SCAN_RSP_RX_DISABLED)) &&
        controller_central_radio_is_rx_window_active() &&
        !controller_central_radio_has_pending_completion())
    {
        controller_central_reset_scan_state();
        radio_disable();
    }
    irq_unlock(primask);
}

static void controller_central_scan_timer_handler(void *p_context)
{
    uint8_t ch;

    (void)p_context;

    if (m_link.connected || !m_ctrl_rt.central.scanning || (m_ctrl_rt.central.scan_radio_phase != BLE_SCAN_RADIO_PHASE_IDLE))
    {
        return;
    }

    ch = (uint8_t)(m_ctrl_rt.central.scan_channel_index % 3U);
    m_ctrl_rt.central.scan_channel_index = (uint8_t)((m_ctrl_rt.central.scan_channel_index + 1U) % 3U);
    (void)memset(&m_ctrl_rt.central.adv_rx_pdu, 0, sizeof(m_ctrl_rt.central.adv_rx_pdu));
    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_SCAN);
    radio_set_frequency(m_adv_freq_mhz_offset[ch]);
    radio_set_whiteiv(m_adv_channels[ch]);
    radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.central.adv_rx_pdu);
    m_ctrl_rt.central.scan_radio_phase = BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED;
    controller_set_mode_with_phy(RADIO_MODE_RX, BLE_LL_PHY_1M);

    controller_central_scan_window_timer_start(m_host.central.scan_config.window_ms);
}

void controller_central_stop_scanning_internal(void)
{
    controller_central_scan_timer_stop();
    controller_central_scan_window_timer_stop();
    m_ctrl_rt.central.scanning = false;
    m_ctrl_rt.central.connect_target_valid = false;
    m_ctrl_rt.central.scan_rsp_connect_candidate_valid = false;
    controller_central_reset_scan_state();
    if (!m_link.connected && (m_ctrl_rt.peripheral.adv_radio_phase == BLE_ADV_RADIO_PHASE_IDLE))
    {
        radio_disable();
        radio_enable_interrupt_mask(0U);
    }
}

void controller_central_start_scanning_internal(void)
{
    if (m_link.connected || m_ctrl_rt.central.scanning)
    {
        return;
    }

    controller_prepare_radio_common((uint8_t)sizeof(m_ctrl_rt.central.adv_rx_pdu.payload),
                                    m_adv_access_address,
                                    m_adv_crc_init,
                                    (uint32_t)&m_ctrl_rt.central.adv_rx_pdu);
    m_ctrl_rt.central.scanning = true;
    m_ctrl_rt.central.scan_channel_index = 0U;

    controller_central_scan_timer_stop();
    controller_central_scan_timer_start(m_host.central.scan_config.interval_ms);
    controller_central_scan_timer_handler(NULL);
}

bool controller_central_start_connecting(void)
{
    if (!m_ctrl_rt.central.scan_filter_enabled || m_link.connected)
    {
        return false;
    }

    m_ctrl_rt.central.connect_target_valid = false;
    controller_central_start_scanning_internal();
    return true;
}

static void controller_central_apply_connect_request(void)
{
    uint32_t first_event_delay_us;

    controller_prepare_connected_link(&m_ctrl_rt.central.connect_req_pdu,
                                      BLE_GAP_ROLE_CENTRAL,
                                      &m_ctrl_rt.central.connect_target);
    m_ctrl_rt.central.connect_target_valid = false;
    m_ctrl_rt.central.scanning = false;
    first_event_delay_us = (m_ctrl_rt.central.connect_req_pdu.ll_data.win_offset == 0U)
                               ? 2000U
                               : UNITS_1P25MS_TO_US(m_ctrl_rt.central.connect_req_pdu.ll_data.win_offset);
    controller_connected_timer_start(first_event_delay_us);
    (void)ble_evt_notify_gap(BLE_GAP_EVT_CONNECTED);
}

void controller_central_start_connection_event(void)
{
    bool new_tx_pdu;

    controller_hop_data_channel();
    m_ctrl_rt.conn.conn_rx_pdu.header = (ble_ll_data_header_t){0};
    m_ctrl_rt.conn.conn_rx_pdu.length = 0U;
    m_ctrl_rt.conn.conn_rx_process_pending = false;
    controller_reset_conn_tx_selection_state();
    m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED;
    new_tx_pdu = !m_ctrl_rt.conn.tx_unacked;

    if (new_tx_pdu)
    {
        if (!controller_load_pending_conn_tx_pdu_for_state(m_link.packet.next_expected_rx_sn, m_link.packet.tx_sn))
        {
            m_ctrl_rt.conn.conn_tx_pdu.header = controller_conn_header(BLE_LLID_CONTINUATION);
            m_ctrl_rt.conn.conn_tx_pdu.length = 0U;
        }
    }

    radio_enable_interrupt_mask(BLE_RADIO_IRQ_MASK_CONN_CENTRAL);
    controller_stage_conn_response(new_tx_pdu);
    controller_set_mode_with_phy(RADIO_MODE_TX, m_link.phy.tx_phy);
    radio_set_shorts(RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk | RADIO_SHORTS_READY_START_Msk);
    radio_start();
    /* PACKETPTR is latched when START executes, so updating it immediately after
       launching TX does not disturb the TX packet and prepares the upcoming
       DISABLED->RXEN turnaround to receive into conn_rx_pdu. */
    radio_set_packet_ptr((uint32_t)&m_ctrl_rt.conn.conn_rx_pdu);
}

static void controller_central_handle_connected_packet(void)
{
    bool tx_acked = m_ctrl_rt.conn.tx_unacked && (m_ctrl_rt.conn.conn_rx_pdu.header.nesn != m_link.packet.tx_sn);
    bool is_new_packet = (m_ctrl_rt.conn.conn_rx_pdu.header.sn == m_link.packet.next_expected_rx_sn);

    m_link.supervision.rx_seen_this_interval = true;
    if (tx_acked)
    {
        m_ctrl_rt.conn.tx_unacked = false;
        m_link.packet.tx_sn ^= 1U;
    }

    if (is_new_packet)
    {
        m_link.packet.next_expected_rx_sn ^= 1U;
    }

    m_ctrl_rt.conn.conn_rx_process_pending = is_new_packet;
    controller_reset_conn_tx_selection_state();
    m_link.supervision.started = true;
}

static void controller_central_handle_crc_error(void)
{
    m_ctrl_rt.conn.conn_rx_process_pending = false;
    controller_reset_conn_tx_selection_state();
    m_link.supervision.started = true;
}

static void controller_central_handle_connected_disabled(void)
{
    if (m_ctrl_rt.conn.conn_radio_phase == BLE_CONN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.conn.conn_radio_phase == BLE_CONN_RADIO_PHASE_WAIT_TX_DISABLED)
    {
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
        m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_WAIT_RX_DISABLED;

        return;
    }

    m_ctrl_rt.conn.conn_radio_phase = BLE_CONN_RADIO_PHASE_IDLE;
    radio_set_shorts(0U);
    if (m_ctrl_rt.conn.conn_rx_process_pending)
    {
        m_ctrl_rt.conn.conn_rx_process_pending = false;
        controller_process_received_conn_pdu();
    }
}

static void controller_central_handle_scan_disabled(void)
{
    if (m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED)
    {
        if (m_ctrl_rt.central.connect_req_tx_pending)
        {
            m_ctrl_rt.central.connect_req_tx_pending = false;
            controller_central_scan_window_timer_stop();
            m_ctrl_rt.central.scan_radio_phase = BLE_SCAN_RADIO_PHASE_WAIT_CONNECT_TX_DISABLED;
            radio_set_packet_ptr((uint32_t)&m_ctrl_rt.central.connect_req_pdu);
            radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
            radio_tx_enable();
            return;
        }

        if (m_ctrl_rt.central.scan_req_tx_pending)
        {
            m_ctrl_rt.central.scan_req_tx_pending = false;
            (void)memset(&m_ctrl_rt.central.scan_rsp_pdu, 0, sizeof(m_ctrl_rt.central.scan_rsp_pdu));
            m_ctrl_rt.central.scan_radio_phase = BLE_SCAN_RADIO_PHASE_WAIT_SCAN_REQ_TX_DISABLED;
            radio_tx_then_rx((uint32_t)&m_ctrl_rt.central.scan_req_pdu,
                             (uint32_t)&m_ctrl_rt.central.scan_rsp_pdu);
            return;
        }

        controller_central_scan_window_timer_stop();
        controller_central_reset_scan_state();
        return;
    }

    if (m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_SCAN_REQ_TX_DISABLED)
    {
        m_ctrl_rt.central.scan_radio_phase = BLE_SCAN_RADIO_PHASE_WAIT_SCAN_RSP_RX_DISABLED;
        radio_set_shorts(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk);
        return;
    }

    if (m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_SCAN_RSP_RX_DISABLED)
    {
        controller_central_scan_window_timer_stop();
        controller_central_reset_scan_state();
        return;
    }

    controller_central_scan_window_timer_stop();
    controller_central_reset_scan_state();
    controller_central_apply_connect_request();
}

void controller_central_handle_radio_event(radio_event_t evt, const ble_ll_adv_pdu_t *p_adv_rx)
{
    if (m_link.connected)
    {
        if (evt == RADIO_EVENT_CRC_OK)
        {
            controller_central_handle_connected_packet();
        }
        else if (evt == RADIO_EVENT_CRC_ERROR)
        {
            controller_central_handle_crc_error();
        }
        else if (evt == RADIO_EVENT_DISABLED)
        {
            controller_central_handle_connected_disabled();
        }
        return;
    }

    if (m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_IDLE)
    {
        return;
    }

    if (evt == RADIO_EVENT_DISABLED)
    {
        controller_central_handle_scan_disabled();
        return;
    }

    if ((m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_RX_DISABLED) &&
        (evt == RADIO_EVENT_CRC_OK))
    {
        controller_central_handle_scan_crc_ok(p_adv_rx);
        return;
    }

    if ((m_ctrl_rt.central.scan_radio_phase == BLE_SCAN_RADIO_PHASE_WAIT_SCAN_RSP_RX_DISABLED) &&
        (evt == RADIO_EVENT_CRC_OK))
    {
        controller_central_handle_scan_rsp_crc_ok(&m_ctrl_rt.central.scan_rsp_pdu);
    }
}

void controller_central_scan_timers_init(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_scan_timer_id, APP_TIMER_MODE_REPEATED, controller_central_scan_timer_handler));
    APP_ERROR_CHECK(app_timer_create(&m_scan_window_timer_id, APP_TIMER_MODE_SINGLE_SHOT, controller_central_scan_window_timeout_handler));
}
