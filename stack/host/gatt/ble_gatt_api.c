/**
 * @file ble_gatt_api.c
 * @author Surya Poudel
 * @brief Public GATT helper APIs for nRF BLE stack
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_gatt_server.h"

#include "ble_runtime_internal.h"

bool ble_notify_characteristic(const ble_gatt_characteristic_t *p_characteristic)
{
    uint8_t att_pdu[BLE_ATT_MAX_MTU];
    uint16_t att_len;

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL) ||
        (p_characteristic == NULL) ||
        !m_link.connected)
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

bool ble_indicate_characteristic(const ble_gatt_characteristic_t *p_characteristic)
{
    uint8_t att_pdu[BLE_ATT_MAX_MTU];
    uint16_t att_len;

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_PERIPHERAL) ||
        (p_characteristic == NULL) ||
        !m_link.connected)
    {
        return false;
    }

    att_len = ble_gatt_server_build_indication(p_characteristic->value_handle, att_pdu, sizeof(att_pdu));
    if (att_len == 0U)
    {
        return false;
    }

    if (!controller_queue_l2cap_payload(BLE_L2CAP_CID_ATT, att_pdu, att_len))
    {
        return false;
    }

    ble_gatt_server_mark_indication_pending();
    return true;
}
