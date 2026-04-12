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

#include "app_error.h"
#include "ble_gatt_server.h"
#include "ble_internal.h"

APP_TIMER_DEF(m_conn_param_update_timer_id);

static void ble_conn_param_update_timeout_handler(void *p_context)
{
    (void)p_context;
    (void)ble_gap_update_conn_params();
}

uint16_t ble_uuid_encoded_len(const ble_uuid_t *p_uuid)
{
    if (p_uuid == NULL)
    {
        return 0U;
    }

    if (p_uuid->type == BLE_UUID_TYPE_SIG_16)
    {
        return BLE_UUID16_LEN;
    }

    if ((p_uuid->type == BLE_UUID_TYPE_VENDOR_16) && m_host.vendor_uuid_base_set)
    {
        return BLE_UUID128_LEN;
    }

    return 0U;
}

bool ble_uuid_is_valid(const ble_uuid_t *p_uuid)
{
    return ble_uuid_encoded_len(p_uuid) != 0U;
}

bool ble_uuid_encode(const ble_uuid_t *p_uuid, uint8_t *p_dst)
{
    if ((p_uuid == NULL) || (p_dst == NULL))
    {
        return false;
    }

    if (p_uuid->type == BLE_UUID_TYPE_SIG_16)
    {
        u16_encode(p_uuid->value, p_dst);
        return true;
    }

    if ((p_uuid->type == BLE_UUID_TYPE_VENDOR_16) && m_host.vendor_uuid_base_set)
    {
        (void)memcpy(p_dst, m_host.vendor_uuid_base, BLE_UUID128_LEN);
        /* BLE carries 128-bit UUIDs in little-endian byte order. Patching
         * bytes 12..13 here corresponds to the canonical 0000XXXX-... slot. */
        u16_encode(p_uuid->value, &p_dst[12]);
        return true;
    }

    return false;
}

bool ble_uuid_matches_bytes(const ble_uuid_t *p_uuid, const uint8_t *p_uuid_bytes, uint16_t uuid_len)
{
    uint8_t encoded_uuid[BLE_UUID128_LEN];

    if ((p_uuid == NULL) || (p_uuid_bytes == NULL) || (ble_uuid_encoded_len(p_uuid) != uuid_len))
    {
        return false;
    }

    if (!ble_uuid_encode(p_uuid, encoded_uuid))
    {
        return false;
    }

    return memcmp(encoded_uuid, p_uuid_bytes, uuid_len) == 0;
}

void ble_gap_set_device_name(const char *p_name)
{
    size_t name_len;

    m_host.adv_name[0] = '\0';

    if (p_name == NULL)
    {
        return;
    }

    name_len = strlen(p_name);
    if (name_len > BLE_ADV_NAME_MAX_LEN)
    {
        name_len = BLE_ADV_NAME_MAX_LEN;
    }

    (void)memcpy(m_host.adv_name, p_name, name_len);
    m_host.adv_name[name_len] = '\0';
}

void ble_gap_set_conn_params(const ble_gap_conn_params_t *p_params)
{
    m_host.preferred_conn_params_valid = false;
    m_host.preferred_conn_params = (ble_gap_conn_params_t){0};

    if (p_params == NULL)
    {
        return;
    }

    if ((p_params->min_conn_interval_1p25ms == 0U) ||
        (p_params->max_conn_interval_1p25ms == 0U) ||
        (p_params->min_conn_interval_1p25ms > p_params->max_conn_interval_1p25ms) ||
        (p_params->supervision_timeout_10ms == 0U))
    {
        return;
    }

    m_host.preferred_conn_params = *p_params;
    m_host.preferred_conn_params_valid = true;
}

bool ble_gap_update_conn_params(void)
{
    uint8_t sig_pdu[BLE_L2CAP_SIG_HDR_LEN + 8U];

    if (!m_link.connected || !m_host.preferred_conn_params_valid)
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

void ble_conn_param_update_timer_init(void)
{
    ret_code_t err = app_timer_create(&m_conn_param_update_timer_id, APP_TIMER_MODE_SINGLE_SHOT, ble_conn_param_update_timeout_handler);

    if ((err != NRF_SUCCESS) && (err != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err);
    }
}

void ble_conn_param_update_timer_start(void)
{
    ret_code_t err;

    if (!m_host.preferred_conn_params_valid)
    {
        return;
    }

    err = app_timer_stop(m_conn_param_update_timer_id);
    if ((err != NRF_SUCCESS) && (err != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err);
    }

    err = app_timer_start(m_conn_param_update_timer_id, APP_TIMER_TICKS(BLE_CONN_PARAM_UPDATE_DELAY_MS), NULL);
    APP_ERROR_CHECK(err);
}

void ble_conn_param_update_timer_stop(void)
{
    ret_code_t err = app_timer_stop(m_conn_param_update_timer_id);

    if ((err != NRF_SUCCESS) && (err != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err);
    }
}

void ble_uuid_set_vendor_base(const uint8_t uuid128[BLE_UUID128_LEN])
{
    if (uuid128 == NULL)
    {
        m_host.vendor_uuid_base_set = false;
        (void)memset(m_host.vendor_uuid_base, 0, sizeof(m_host.vendor_uuid_base));
        return;
    }

    (void)memcpy(m_host.vendor_uuid_base, uuid128, BLE_UUID128_LEN);
    m_host.vendor_uuid_base_set = true;
}

bool ble_is_connected(void)
{
    return m_link.connected;
}

void ble_register_evt_handler(ble_evt_handler_t handler)
{
    m_evt_handler = handler;
}

void ble_stack_init(void)
{
    m_host = (ble_host_t){
        .flags = (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                           BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED),
        .adv_interval_ms = BLE_ADV_INTERVAL_MS_DEFAULT,
    };
    m_link = (ble_link_t){0};
    m_ctrl_rt = (ble_ctrl_runtime_t){0};
    m_evt_handler = NULL;
    controller_load_identity_address();

    ble_evt_dispatch_init();
    controller_runtime_init();
    ble_conn_param_update_timer_init();
}

void ble_adv_init(const ble_adv_config_t *p_config)
{
    m_host.flags = (p_config != NULL) ? p_config->flags
                                      : (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                                                  BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    m_host.tx_power = (p_config != NULL) ? p_config->tx_power : 0;
    m_host.adv_interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U)) ? p_config->interval_ms
                                                                                     : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.included_service_uuid = ((p_config != NULL) && (p_config->p_included_service_uuid != NULL))
                                       ? *p_config->p_included_service_uuid
                                       : (ble_uuid_t)BLE_UUID_NONE_INIT;
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

    return controller_queue_l2cap_payload(BLE_L2CAP_CID_ATT, att_pdu, att_len);
}
