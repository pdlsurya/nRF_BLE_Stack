/**
 * @file ble_stack.c
 * @author Surya Poudel
 * @brief Public API implementation for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "nrf_ble.h"

#include <string.h>

#include "ble_gatt_server.h"
#include "ble_internal.h"

static const ble_gap_conn_params_t m_gap_conn_params_default = {
    .min_conn_interval_units = BLE_GAP_CONN_INTERVAL_MIN_DEFAULT_UNITS,
    .max_conn_interval_units = BLE_GAP_CONN_INTERVAL_MAX_DEFAULT_UNITS,
    .slave_latency = BLE_GAP_CONN_SLAVE_LATENCY_DEFAULT,
    .supervision_timeout_units = BLE_GAP_CONN_SUP_TIMEOUT_DEFAULT_UNITS,
};

static const ble_adv_config_t m_adv_config_default = {
    .flags = 0x06U,
    .tx_power = 0,
    .interval_ms = BLE_ADV_INTERVAL_MS_DEFAULT,
    .included_service_uuid = 0U,
    .p_service_data = NULL,
};

void ble_gap_set_device_name(const char *p_name)
{
    size_t name_len;

    m_host.adv_name[0] = '\0';
    m_host.adv_name_len = 0U;

    if ((p_name == NULL) || (p_name[0] == '\0'))
    {
        return;
    }

    name_len = strlen(p_name);
    if (name_len > BLE_ADV_NAME_MAX_LEN)
    {
        name_len = BLE_ADV_NAME_MAX_LEN;
    }
    if (name_len == 0U)
    {
        return;
    }

    (void)memcpy(m_host.adv_name, p_name, name_len);
    m_host.adv_name[name_len] = '\0';
    m_host.adv_name_len = (uint8_t)name_len;
}

bool ble_is_connected(void)
{
    return m_link.connected;
}

bool ble_characteristic_notifications_enabled(const ble_gatt_characteristic_t *p_characteristic)
{
    return ble_gatt_server_notifications_enabled(p_characteristic);
}

void ble_disconnect(void)
{
    controller_disconnect_internal();
}

void ble_register_evt_handler(ble_evt_handler_t handler)
{
    m_evt_handler = handler;
}

void ble_stack_init(void)
{
    bool adv_timer_created = m_controller.adv_timer_created;

    m_host = (ble_host_t){
        .flags = m_adv_config_default.flags,
        .adv_interval_ms = m_adv_config_default.interval_ms,
        .gap_conn_params = m_gap_conn_params_default,
    };
    m_controller = (ble_controller_t){.adv_timer_created = adv_timer_created};
    m_link = (ble_link_t){0};
    m_ctrl_rt = (ble_ctrl_runtime_t){0};
    m_evt_handler = NULL;
    controller_load_identity_address();

    ble_evt_dispatch_init();
    controller_runtime_init();
}

void ble_adv_init(const ble_adv_config_t *p_config)
{
    if (p_config == NULL)
    {
        p_config = &m_adv_config_default;
    }

    m_host.flags = p_config->flags;
    m_host.tx_power = p_config->tx_power;
    m_host.adv_interval_ms = (p_config->interval_ms != 0U) ? p_config->interval_ms : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.included_service_uuid = p_config->included_service_uuid;
    m_host.has_service_data = false;

    if (p_config->p_service_data != NULL)
    {
        m_host.service_data = *p_config->p_service_data;
        m_host.has_service_data = true;
    }
}

void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params)
{
    m_host.gap_conn_params = (p_params != NULL) ? *p_params : m_gap_conn_params_default;
}

bool ble_notify_characteristic(const ble_gatt_characteristic_t *p_characteristic)
{
    uint8_t att_pdu[BLE_ATT_GATT_MAX_MTU];
    uint16_t att_len;

    if ((p_characteristic == NULL) || !m_link.connected)
    {
        return false;
    }

    att_len = ble_gatt_server_build_notification(p_characteristic->value_handle, att_pdu, sizeof(att_pdu));
    if (att_len == 0U)
    {
        return false;
    }

    return controller_queue_att_payload(att_pdu, att_len);
}
