/**
 * @file ble_stack.c
 * @author Surya Poudel
 * @brief Stack bootstrap and shared public lifecycle APIs for nRF BLE stack
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_runtime_internal.h"

#include "app_error.h"

bool ble_is_connected(void)
{
    return m_link.connected;
}

void ble_gap_register_evt_handler(ble_gap_evt_handler_t handler)
{
    m_gap_evt_handler = handler;
}

void ble_register_scan_report_handler(ble_gap_scan_report_handler_t handler)
{
    m_scan_report_handler = handler;
}

void ble_stack_init(ble_gap_role_t role)
{
    APP_ERROR_CHECK_BOOL((role == BLE_GAP_ROLE_PERIPHERAL) ||
                         (role == BLE_GAP_ROLE_CENTRAL));

    m_host = (ble_host_t){
        .configured_role = role,
        .flags = (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                           BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED),
        .adv_interval_ms = BLE_ADV_INTERVAL_MS_DEFAULT,
        .scan_config = {
            .interval_ms = BLE_SCAN_INTERVAL_MS_DEFAULT,
            .window_ms = BLE_SCAN_WINDOW_MS_DEFAULT,
        },
        .adv_name_type = BLE_GAP_ADV_NAME_FULL,
    };
    m_link = (ble_link_t){0};
    m_ctrl_rt = (ble_ctrl_runtime_t){0};
    m_gap_evt_handler = NULL;
    m_scan_report_handler = NULL;
    m_gatt_server_evt_handler = NULL;
    m_gatt_client_evt_handler = NULL;
    controller_load_identity_address();

    ble_evt_dispatch_init();
    controller_runtime_init();
    ble_gatt_client_init();
}
