/**
 * @file ble_gap.c
 * @author Surya Poudel
 * @brief GAP-facing host API implementation for nRF BLE stack
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_runtime_internal.h"

#include <string.h>

static bool ble_gap_conn_params_are_valid(const ble_gap_conn_params_t *p_params)
{
    if (p_params == NULL)
    {
        return false;
    }

    return (p_params->min_conn_interval_1p25ms != 0U) &&
           (p_params->max_conn_interval_1p25ms != 0U) &&
           (p_params->min_conn_interval_1p25ms <= p_params->max_conn_interval_1p25ms) &&
           (p_params->supervision_timeout_10ms != 0U);
}

static size_t ble_scan_filter_name_len(const char *p_name)
{
    size_t name_len = 0U;

    if (p_name == NULL)
    {
        return 0U;
    }

    while ((name_len <= BLE_GAP_SCAN_FILTER_NAME_MAX_LEN) && (p_name[name_len] != '\0'))
    {
        name_len++;
    }

    return name_len;
}

void ble_gap_set_device_name(const char *p_name)
{
    size_t name_len;

    m_host.gap_device_name[0] = '\0';

    if (p_name == NULL)
    {
        return;
    }

    name_len = strlen(p_name);
    if (name_len > BLE_GAP_DEVICE_NAME_MAX_LEN)
    {
        name_len = BLE_GAP_DEVICE_NAME_MAX_LEN;
    }

    (void)memcpy(m_host.gap_device_name, p_name, name_len);
    m_host.gap_device_name[name_len] = '\0';
}

void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params)
{
    m_host.preferred_conn_params_valid = false;
    m_host.preferred_conn_params = (ble_gap_conn_params_t){0};

    if (p_params == NULL)
    {
        return;
    }

    if (!ble_gap_conn_params_are_valid(p_params))
    {
        return;
    }

    m_host.preferred_conn_params = *p_params;
    m_host.preferred_conn_params_valid = true;
}

bool ble_gap_set_scan_filter(const ble_gap_scan_filter_t *p_filter)
{
    size_t name_len;

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) || (p_filter == NULL))
    {
        return false;
    }

    if (!p_filter->match_addr &&
        !p_filter->match_name &&
        !p_filter->match_service_uuid16 &&
        !p_filter->match_service_uuid128)
    {
        return false;
    }

    if (p_filter->match_name)
    {
        name_len = ble_scan_filter_name_len(p_filter->name);
        if ((name_len == 0U) || (name_len > BLE_GAP_SCAN_FILTER_NAME_MAX_LEN))
        {
            return false;
        }
    }

    if (p_filter->match_service_uuid16 && (p_filter->service_uuid16 == 0U))
    {
        return false;
    }

    m_ctrl_rt.connect_filter = *p_filter;
    m_ctrl_rt.connect_filter_enabled = true;
    m_ctrl_rt.connect_target_valid = false;
    return true;
}

void ble_gap_clear_scan_filter(void)
{
    m_ctrl_rt.connect_filter_enabled = false;
    m_ctrl_rt.connect_filter = (ble_gap_scan_filter_t){0};
    m_ctrl_rt.connect_target_valid = false;
}

bool ble_gap_request_conn_params_update(void)
{
    uint8_t sig_pdu[BLE_L2CAP_SIG_HDR_LEN + 8U];

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL) ||
        !m_link.connected ||
        !m_host.preferred_conn_params_valid)
    {
        return false;
    }

    sig_pdu[0] = BLE_L2CAP_SIG_CONN_PARAM_UPDATE_REQ;
    m_host.next_l2cap_sig_identifier++;
    if (m_host.next_l2cap_sig_identifier == 0U)
    {
        m_host.next_l2cap_sig_identifier = 1U;
    }
    sig_pdu[1] = m_host.next_l2cap_sig_identifier;
    u16_encode(8U, &sig_pdu[2]);
    u16_encode(m_host.preferred_conn_params.min_conn_interval_1p25ms, &sig_pdu[4]);
    u16_encode(m_host.preferred_conn_params.max_conn_interval_1p25ms, &sig_pdu[6]);
    u16_encode(m_host.preferred_conn_params.slave_latency, &sig_pdu[8]);
    u16_encode(m_host.preferred_conn_params.supervision_timeout_10ms, &sig_pdu[10]);

    return controller_queue_l2cap_payload(BLE_L2CAP_CID_SIGNALING, sig_pdu, sizeof(sig_pdu));
}

bool ble_gap_initiate_conn_update(const ble_gap_conn_params_t *p_params)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        !ble_gap_conn_params_are_valid(p_params))
    {
        return false;
    }

    return controller_initiate_conn_update(p_params);
}

void ble_adv_init(const ble_adv_config_t *p_config)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL))
    {
        return;
    }

    m_host.flags = (p_config != NULL) ? p_config->flags
                                      : (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                                                  BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    m_host.tx_power = (p_config != NULL) ? p_config->tx_power : 0;
    m_host.adv_name_type = (p_config != NULL) ? p_config->name_type : BLE_GAP_ADV_NAME_FULL;
    m_host.adv_short_name_length = (p_config != NULL) ? p_config->short_name_length : 0U;
    m_host.adv_interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U)) ? p_config->interval_ms
                                                                                     : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.included_service_uuid = ((p_config != NULL) && (p_config->p_included_service_uuid != NULL))
                                       ? *p_config->p_included_service_uuid
                                       : (ble_uuid_t)BLE_UUID_NONE_INIT;
    if (m_host.adv_name_type > BLE_GAP_ADV_NAME_SHORT)
    {
        m_host.adv_name_type = BLE_GAP_ADV_NAME_FULL;
    }
    if (m_host.adv_short_name_length > BLE_GAP_DEVICE_NAME_MAX_LEN)
    {
        m_host.adv_short_name_length = BLE_GAP_DEVICE_NAME_MAX_LEN;
    }
}

void ble_scan_init(const ble_scan_config_t *p_config)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    m_host.scan_config.interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U))
                                         ? p_config->interval_ms
                                         : BLE_SCAN_INTERVAL_MS_DEFAULT;
    m_host.scan_config.window_ms = ((p_config != NULL) && (p_config->window_ms != 0U))
                                       ? p_config->window_ms
                                       : BLE_SCAN_WINDOW_MS_DEFAULT;

    if (m_host.scan_config.window_ms > m_host.scan_config.interval_ms)
    {
        m_host.scan_config.window_ms = m_host.scan_config.interval_ms;
    }
}

void ble_start_advertising(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL))
    {
        return;
    }

    controller_start_advertising_internal();
}

void ble_start_scanning(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    controller_start_scanning_internal();
}

void ble_stop_scanning(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    controller_stop_scanning_internal();
}

bool ble_gap_connect(const ble_gap_addr_t *p_peer_addr)
{
    ble_gap_scan_filter_t filter;

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return false;
    }

    if (p_peer_addr == NULL)
    {
        return false;
    }

    filter = (ble_gap_scan_filter_t){
        .match_addr = true,
        .addr = *p_peer_addr,
    };

    if (!ble_gap_set_scan_filter(&filter))
    {
        return false;
    }

    return controller_start_connecting();
}

void ble_disconnect(void)
{
    controller_disconnect_internal();
}
