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
#include "ble_l2cap_internal.h"

#include "app_error.h"

#include <string.h>

#define BLE_CONN_PARAM_UPDATE_DELAY_MS 6000U

APP_TIMER_DEF(m_conn_param_update_timer_id);

bool ble_gap_is_connected(void)
{
    return m_link.connected;
}

void ble_gap_register_evt_handler(ble_gap_evt_handler_t handler)
{
    m_gap_evt_handler = handler;
}

void ble_gap_register_scan_report_handler(ble_gap_scan_report_handler_t handler)
{
    m_scan_report_handler = handler;
}

static void ble_conn_param_update_timeout_handler(void *p_context)
{
    (void)p_context;
    (void)ble_gap_request_conn_params_update();
}

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

    m_host.common.gap_device_name[0] = '\0';

    if (p_name == NULL)
    {
        return;
    }

    name_len = strlen(p_name);
    if (name_len > BLE_GAP_DEVICE_NAME_MAX_LEN)
    {
        name_len = BLE_GAP_DEVICE_NAME_MAX_LEN;
    }

    (void)memcpy(m_host.common.gap_device_name, p_name, name_len);
    m_host.common.gap_device_name[name_len] = '\0';
}

void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params)
{
    m_host.common.preferred_conn_params_valid = false;
    m_host.common.preferred_conn_params = (ble_gap_conn_params_t){0};

    if (p_params == NULL)
    {
        return;
    }

    if (!ble_gap_conn_params_are_valid(p_params))
    {
        return;
    }

    m_host.common.preferred_conn_params = *p_params;
    m_host.common.preferred_conn_params_valid = true;
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

    m_ctrl_rt.central.scan_filter = *p_filter;
    m_ctrl_rt.central.scan_filter_enabled = true;
    m_ctrl_rt.central.connect_target_valid = false;
    m_ctrl_rt.central.scan_rsp_connect_candidate_valid = false;
    return true;
}

void ble_gap_clear_scan_filter(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    m_ctrl_rt.central.scan_filter_enabled = false;
    m_ctrl_rt.central.scan_filter = (ble_gap_scan_filter_t){0};
    m_ctrl_rt.central.connect_target_valid = false;
    m_ctrl_rt.central.scan_rsp_connect_candidate_valid = false;
}

bool ble_gap_request_conn_params_update(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL) ||
        !m_link.connected ||
        !m_host.common.preferred_conn_params_valid)
    {
        return false;
    }

    return ble_l2cap_queue_conn_param_update_req(&m_host.common.preferred_conn_params);
}

void ble_conn_param_update_timer_init(void)
{
    APP_ERROR_CHECK(app_timer_create(&m_conn_param_update_timer_id,
                                     APP_TIMER_MODE_SINGLE_SHOT,
                                     ble_conn_param_update_timeout_handler));
}

void ble_conn_param_update_timer_start(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL) ||
        !m_link.connected ||
        !m_host.common.preferred_conn_params_valid)
    {
        return;
    }

    APP_ERROR_CHECK(app_timer_stop(m_conn_param_update_timer_id));
    APP_ERROR_CHECK(app_timer_start(m_conn_param_update_timer_id,
                                    APP_TIMER_TICKS(BLE_CONN_PARAM_UPDATE_DELAY_MS),
                                    NULL));
}

void ble_conn_param_update_timer_stop(void)
{
    APP_ERROR_CHECK(app_timer_stop(m_conn_param_update_timer_id));
}

bool ble_gap_initiate_conn_update(const ble_gap_conn_params_t *p_params)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        !ble_gap_conn_params_are_valid(p_params))
    {
        return false;
    }

    return controller_central_initiate_conn_update(p_params);
}

static bool ble_gap_adv_type_is_supported(ble_gap_adv_type_t adv_type)
{
    return (adv_type == BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED) ||
           (adv_type == BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED) ||
           (adv_type == BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
}

void ble_gap_adv_init(const ble_adv_config_t *p_config)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL))
    {
        return;
    }

    m_host.peripheral.flags = (p_config != NULL) ? p_config->flags
                                                 : (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                                                             BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    m_host.common.tx_power = (p_config != NULL) ? p_config->tx_power : 0;
    m_host.peripheral.adv_name_type = (p_config != NULL) ? p_config->name_type : BLE_GAP_ADV_NAME_FULL;
    m_host.peripheral.adv_short_name_length = (p_config != NULL) ? p_config->short_name_length : 0U;
    m_host.peripheral.adv_type = ((p_config != NULL) && ble_gap_adv_type_is_supported(p_config->adv_type))
                                     ? p_config->adv_type
                                     : BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_host.peripheral.adv_interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U)) ? p_config->interval_ms
                                                                                              : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.peripheral.adv_included_service_uuid = ((p_config != NULL) && (p_config->p_included_service_uuid != NULL))
                                                      ? *p_config->p_included_service_uuid
                                                      : (ble_uuid_t)BLE_UUID_NONE_INIT;
    m_host.peripheral.service_count = ((p_config != NULL) && (p_config->p_included_service_uuid != NULL)) ? 1U : 0U; // Defaults adv included service uuid type to complete list of 16-bit/128-bit UUIDs.

    if (m_host.peripheral.adv_name_type > BLE_GAP_ADV_NAME_SHORT)
    {
        m_host.peripheral.adv_name_type = BLE_GAP_ADV_NAME_FULL;
    }
    if (m_host.peripheral.adv_short_name_length > BLE_GAP_DEVICE_NAME_MAX_LEN)
    {
        m_host.peripheral.adv_short_name_length = BLE_GAP_DEVICE_NAME_MAX_LEN;
    }
}

void ble_gap_scan_init(const ble_scan_config_t *p_config)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    m_host.central.scan_config.interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U))
                                                 ? p_config->interval_ms
                                                 : BLE_SCAN_INTERVAL_MS_DEFAULT;
    m_host.central.scan_config.window_ms = ((p_config != NULL) && (p_config->window_ms != 0U))
                                               ? p_config->window_ms
                                               : BLE_SCAN_WINDOW_MS_DEFAULT;
    m_host.central.scan_config.active = (p_config != NULL) ? p_config->active : false;

    if (m_host.central.scan_config.window_ms > m_host.central.scan_config.interval_ms)
    {
        m_host.central.scan_config.window_ms = m_host.central.scan_config.interval_ms;
    }
}

void ble_gap_start_advertising(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL))
    {
        return;
    }

    controller_peripheral_start_advertising_internal();
}

void ble_gap_start_scanning(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    controller_central_start_scanning_internal();
}

void ble_gap_stop_scanning(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL))
    {
        return;
    }

    controller_central_stop_scanning_internal();
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

    return controller_central_start_connecting();
}

void ble_gap_disconnect(void)
{
    controller_disconnect_internal();
}
