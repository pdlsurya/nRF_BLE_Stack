/* SPDX-License-Identifier: MIT */

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

    m_host.common.local_device_name[0] = '\0';

    if (p_name == NULL)
    {
        return;
    }

    name_len = strlen(p_name);
    if (name_len > BLE_HOST_LOCAL_DEVICE_NAME_MAX_LEN)
    {
        name_len = BLE_HOST_LOCAL_DEVICE_NAME_MAX_LEN;
    }

    (void)memcpy(m_host.common.local_device_name, p_name, name_len);
    m_host.common.local_device_name[name_len] = '\0';
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

static bool ble_gap_adv_service_uuid_list_type_is_supported(ble_gap_adv_service_uuid_list_type_t type)
{
    return (type == BLE_GAP_ADV_SERVICE_UUID_LIST_INCOMPLETE_16) ||
           (type == BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_16) ||
           (type == BLE_GAP_ADV_SERVICE_UUID_LIST_INCOMPLETE_128) ||
           (type == BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_128);
}

static bool ble_gap_adv_service_uuid_lists_copy(const ble_gap_adv_data_config_t *p_src,
                                                ble_host_adv_data_t *p_dst)
{
    uint8_t list_idx;
    uint8_t uuid_idx;

    if ((p_src == NULL) || (p_dst == NULL))
    {
        return false;
    }

    if (p_src->service_uuid_list_count == 0U)
    {
        return true;
    }

    if ((p_src->p_service_uuid_lists == NULL) ||
        (p_src->service_uuid_list_count > BLE_GAP_ADV_SERVICE_UUID_LIST_MAX_COUNT))
    {
        return false;
    }

    for (list_idx = 0U; list_idx < p_src->service_uuid_list_count; list_idx++)
    {
        const ble_gap_adv_service_uuid_list_config_t *p_src_list = &p_src->p_service_uuid_lists[list_idx];

        if (!ble_gap_adv_service_uuid_list_type_is_supported(p_src_list->type) ||
            (p_src_list->uuid_count == 0U) ||
            (p_src_list->uuid_count > BLE_GAP_ADV_SERVICE_UUID_PER_LIST_MAX_COUNT) ||
            (p_src_list->p_uuids == NULL))
        {
            return false;
        }

        p_dst->service_uuid_lists[list_idx].type = p_src_list->type;
        p_dst->service_uuid_lists[list_idx].uuid_count = p_src_list->uuid_count;

        for (uuid_idx = 0U; uuid_idx < p_src_list->uuid_count; uuid_idx++)
        {
            p_dst->service_uuid_lists[list_idx].uuids[uuid_idx] = p_src_list->p_uuids[uuid_idx];
        }
    }

    p_dst->service_uuid_list_count = p_src->service_uuid_list_count;
    return true;
}

static bool ble_gap_adv_data_config_copy(const ble_gap_adv_data_config_t *p_src,
                                         ble_host_adv_data_t *p_dst)
{
    if ((p_src == NULL) || (p_dst == NULL))
    {
        return false;
    }

    *p_dst = (ble_host_adv_data_t){0};

    if (p_src->p_name != NULL)
    {
        p_dst->name = *p_src->p_name;
        if (p_dst->name.name_type > BLE_GAP_ADV_NAME_SHORT)
        {
            p_dst->name.name_type = BLE_GAP_ADV_NAME_FULL;
        }
        if (p_dst->name.short_name_length > BLE_HOST_LOCAL_DEVICE_NAME_MAX_LEN)
        {
            p_dst->name.short_name_length = BLE_HOST_LOCAL_DEVICE_NAME_MAX_LEN;
        }
        p_dst->name_present = true;
    }

    if (p_src->p_tx_power != NULL)
    {
        p_dst->tx_power = *p_src->p_tx_power;
        p_dst->tx_power_present = true;
    }

    if (!ble_gap_adv_service_uuid_lists_copy(p_src, p_dst))
    {
        return false;
    }

    if (p_src->p_service_data != NULL)
    {
        if ((p_src->p_service_data->data_len > BLE_GAP_ADV_FIELD_DATA_MAX_LEN) ||
            ((p_src->p_service_data->data_len != 0U) &&
             (p_src->p_service_data->p_data == NULL)))
        {
            return false;
        }

        p_dst->service_data = *p_src->p_service_data;
        p_dst->service_data_present = true;
    }

    if (p_src->p_manufacturer_data != NULL)
    {
        if ((p_src->p_manufacturer_data->data_len > BLE_GAP_ADV_FIELD_DATA_MAX_LEN) ||
            ((p_src->p_manufacturer_data->data_len != 0U) &&
             (p_src->p_manufacturer_data->p_data == NULL)))
        {
            return false;
        }

        p_dst->manufacturer_data = *p_src->p_manufacturer_data;
        p_dst->manufacturer_data_present = true;
    }

    return true;
}

bool ble_gap_adv_init(const ble_gap_adv_config_t *p_config)
{
    ble_gap_adv_type_t adv_type;
    static const ble_gap_adv_name_config_t default_name = {
        .name_type = BLE_GAP_ADV_NAME_FULL,
        .short_name_length = 0U,
    };
    static const ble_gap_adv_data_config_t empty_adv_data = {0};
    const ble_gap_adv_data_config_t default_adv_data = {
        .p_name = &default_name,
    };
    const ble_gap_adv_data_config_t *p_adv_data;
    const ble_gap_adv_data_config_t *p_scan_response_data;

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL))
    {
        return false;
    }

    adv_type = ((p_config != NULL) && ble_gap_adv_type_is_supported(p_config->adv_type))
                   ? p_config->adv_type
                   : BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;

    p_adv_data = (p_config != NULL) ? &p_config->adv_data : &default_adv_data;
    p_scan_response_data = (p_config != NULL) ? &p_config->scan_response_data
                                              : &empty_adv_data;

    if (!ble_gap_adv_data_config_copy(p_adv_data, &m_host.peripheral.adv_data) ||
        !ble_gap_adv_data_config_copy(p_scan_response_data, &m_host.peripheral.scan_response_data))
    {
        return false;
    }

    m_host.peripheral.flags = (p_config != NULL) ? p_config->flags
                                                 : (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                                                             BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    m_host.peripheral.adv_type = adv_type;
    m_host.peripheral.adv_interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U))
                                            ? p_config->interval_ms
                                            : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.common.tx_power = m_host.peripheral.adv_data.tx_power_present
                                 ? m_host.peripheral.adv_data.tx_power
                                 : (m_host.peripheral.scan_response_data.tx_power_present
                                        ? m_host.peripheral.scan_response_data.tx_power
                                        : 0);
    return true;
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
