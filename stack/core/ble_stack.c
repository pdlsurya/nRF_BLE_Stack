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
}

void ble_adv_init(const ble_adv_config_t *p_config)
{
    m_host.flags = (p_config != NULL) ? p_config->flags
                                      : (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                                                  BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    m_host.tx_power = (p_config != NULL) ? p_config->tx_power : 0;
    m_host.adv_interval_ms = ((p_config != NULL) && (p_config->interval_ms != 0U)) ? p_config->interval_ms
                                                                                     : BLE_ADV_INTERVAL_MS_DEFAULT;
    m_host.included_service_uuid = (p_config != NULL) ? p_config->included_service_uuid : 0U;
    m_host.service_data = ((p_config != NULL) && (p_config->p_service_data != NULL)) ? *p_config->p_service_data
                                                                                       : (ble_service_data_t){0};
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
