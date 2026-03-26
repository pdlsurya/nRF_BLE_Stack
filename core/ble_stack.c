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
    .p_adv_name = NULL,
    .flags = 0x06U,
    .tx_power = 0,
    .interval_ms = BLE_ADV_INTERVAL_MS_DEFAULT,
    .included_service_uuid = 0U,
    .p_service_data = NULL,
};

static bool ble_diag_take_u8(volatile uint8_t *p_queue,
                             volatile uint8_t *p_ridx,
                             uint8_t widx,
                             uint8_t mask,
                             uint8_t *p_value)
{
    uint8_t ridx;

    if (p_value == NULL)
    {
        return false;
    }

    ridx = *p_ridx;
    if (ridx == widx)
    {
        return false;
    }

    *p_value = p_queue[ridx];
    *p_ridx = (uint8_t)((ridx + 1U) & mask);
    return true;
}

static void ble_adv_name_set(const char *p_name)
{
    size_t name_len = 0U;

    m_host.adv_name[0] = '\0';
    m_host.adv_name_len = 0U;
    m_host.has_adv_name = false;

    if (p_name == NULL)
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
    m_host.has_adv_name = true;
}

bool ble_is_connected(void)
{
    return m_link.connected;
}

bool ble_characteristic_notifications_enabled(const ble_gatt_characteristic_t *p_characteristic)
{
    return ble_gatt_server_notifications_enabled(p_characteristic);
}

bool ble_diag_take_first_data_packet(void)
{
    bool pending = m_diag.first_data_pending;
    m_diag.first_data_pending = false;
    return pending;
}

bool ble_diag_take_first_ctrl_opcode(uint8_t *p_opcode)
{
    return ble_diag_take_u8(m_diag.ctrl_opcode_q,
                            &m_diag.ctrl_q_ridx,
                            m_diag.ctrl_q_widx,
                            0x07U,
                            p_opcode);
}

bool ble_diag_take_first_att_opcode(uint8_t *p_opcode)
{
    return ble_diag_take_u8(m_diag.att_opcode_q,
                            &m_diag.att_q_ridx,
                            m_diag.att_q_widx,
                            0x07U,
                            p_opcode);
}

bool ble_diag_take_packet_trace(ble_diag_packet_trace_t *p_trace)
{
    uint8_t ridx;

    if (p_trace == NULL)
    {
        return false;
    }

    ridx = m_diag.packet_q_ridx;
    if (ridx == m_diag.packet_q_widx)
    {
        return false;
    }

    *p_trace = m_diag.packet_q[ridx];
    m_diag.packet_q_ridx = (uint8_t)((ridx + 1U) & 0x0FU);
    return true;
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

    (void)memset(&m_host, 0, sizeof(m_host));
    (void)memset(&m_controller, 0, sizeof(m_controller));
    (void)memset(&m_link, 0, sizeof(m_link));
    (void)memset(&m_ctrl_rt, 0, sizeof(m_ctrl_rt));
    m_host.flags = 0x06U;
    m_host.adv_interval_ms = BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.gap_conn_params = m_gap_conn_params_default;
    m_evt_handler = NULL;
    (void)memset(&m_diag, 0, sizeof(m_diag));
    m_controller.adv_timer_created = adv_timer_created;
    controller_load_identity_address();

    ble_evt_dispatch_init();
    controller_runtime_init();
}

void ble_adv_init(const ble_adv_config_t *p_config)
{
    const ble_adv_config_t *p_active = (p_config != NULL) ? p_config : &m_adv_config_default;

    m_host.flags = p_active->flags;
    m_host.tx_power = p_active->tx_power;
    m_host.adv_interval_ms = (p_active->interval_ms != 0U) ? p_active->interval_ms : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.included_service_uuid = p_active->included_service_uuid;
    m_host.has_included_service_uuid = (p_active->included_service_uuid != 0U);
    m_host.has_service_data = false;
    ble_adv_name_set(p_active->p_adv_name);

    if (p_active->p_service_data != NULL)
    {
        m_host.service_data = *p_active->p_service_data;
        m_host.has_service_data = true;
    }
}

void ble_gap_init(const ble_gap_conn_params_t *p_params)
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
