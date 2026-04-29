/**
 * @file ble_runtime.c
 * @author Surya Poudel
 * @brief Shared runtime state, utilities, and deferred event dispatch for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_runtime_internal.h"

#include <string.h>

const uint8_t m_adv_channels[3] = {37U, 38U, 39U};
const uint8_t m_adv_freq_mhz_offset[3] = {2U, 26U, 80U};
const uint8_t m_adv_access_address[4] = {0xD6U, 0xBEU, 0x89U, 0x8EU};
const uint8_t m_data_channel_freq[37] = {
    4U, 6U, 8U, 10U, 12U, 14U, 16U, 18U, 20U, 22U, 24U, 28U, 30U,
    32U, 34U, 36U, 38U, 40U, 42U, 44U, 46U, 48U, 50U, 52U, 54U, 56U,
    58U, 60U, 62U, 64U, 66U, 68U, 70U, 72U, 74U, 76U, 78U};
const uint32_t m_ble_crc_poly = 0x100065BU;
const uint32_t m_adv_crc_init = 0x555555U;
ble_host_t m_host;
ble_link_t m_link;
ble_gap_evt_handler_t m_gap_evt_handler;
ble_gap_scan_report_handler_t m_scan_report_handler;
ble_gatt_server_evt_handler_t m_gatt_server_evt_handler;
ble_gatt_client_evt_handler_t m_gatt_client_evt_handler;
ble_ctrl_runtime_t m_ctrl_rt;
static ble_evt_dispatch_state_t m_evt_dispatch;

static bool ble_evt_post(const ble_deferred_evt_t *p_evt)
{
    uint32_t primask;
    uint8_t next_widx;

    primask = irq_lock();

    next_widx = (uint8_t)((m_evt_dispatch.widx + 1U) % BLE_EVT_QUEUE_SIZE);
    if (next_widx == m_evt_dispatch.ridx)
    {
        irq_unlock(primask);
        return false;
    }

    m_evt_dispatch.q[m_evt_dispatch.widx] = *p_evt;
    m_evt_dispatch.widx = next_widx;
    irq_unlock(primask);
    NVIC_SetPendingIRQ(SWI1_EGU1_IRQn);
    return true;
}

uint16_t u16_decode(const uint8_t *p_src)
{
    return (uint16_t)p_src[0] | ((uint16_t)p_src[1] << 8);
}

void u16_encode(uint16_t value, uint8_t *p_dst)
{
    p_dst[0] = (uint8_t)(value & 0xFFU);
    p_dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void ble_evt_dispatch_init(void)
{
    (void)memset((void *)&m_evt_dispatch, 0, sizeof(m_evt_dispatch));
    NVIC_ClearPendingIRQ(SWI1_EGU1_IRQn);
    NVIC_SetPriority(SWI1_EGU1_IRQn, BLE_EVT_IRQ_PRIORITY);
    NVIC_EnableIRQ(SWI1_EGU1_IRQn);
}

bool ble_evt_notify_gap(ble_gap_evt_type_t evt_type)
{
    ble_deferred_evt_t evt;

    if (m_gap_evt_handler == NULL)
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GAP;
    evt.params.gap_evt.evt_type = evt_type;
    evt.params.gap_evt.params.conn_interval_ms = m_link.conn.conn_interval_ms;
    evt.params.gap_evt.params.slave_latency = m_link.conn.slave_latency;
    evt.params.gap_evt.params.supervision_timeout_ms = m_link.conn.supervision_timeout_ms;
    evt.params.gap_evt.params.tx_phy = m_link.phy.tx_phy;
    evt.params.gap_evt.params.rx_phy = m_link.phy.rx_phy;
    evt.params.gap_evt.params.role = m_link.role;
    evt.params.gap_evt.params.peer_addr = m_link.peer_addr;
    return ble_evt_post(&evt);
}

bool ble_evt_notify_gatt_characteristic(ble_gatt_char_evt_type_t evt_type,
                                        ble_gatt_characteristic_t *p_characteristic)
{
    ble_deferred_evt_t evt;

    if ((p_characteristic == NULL) || (p_characteristic->evt_handler == NULL))
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GATT_CHARACTERISTIC;
    evt.params.gatt_characteristic.evt_type = evt_type;
    evt.params.gatt_characteristic.p_characteristic = p_characteristic;
    return ble_evt_post(&evt);
}

bool ble_evt_notify_scan_report(const ble_gap_scan_report_t *p_report)
{
    ble_deferred_evt_t evt;

    if ((p_report == NULL) || (m_scan_report_handler == NULL))
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_SCAN_REPORT;
    evt.params.scan_report = *p_report;
    return ble_evt_post(&evt);
}

bool ble_evt_notify_gatt_server_mtu_exchange(uint16_t requested_mtu,
                                             uint16_t response_mtu,
                                             uint16_t effective_mtu)
{
    ble_deferred_evt_t evt;

    if (m_gatt_server_evt_handler == NULL)
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GATT_SERVER;
    evt.params.gatt_server_evt.evt_type = BLE_GATT_SERVER_EVT_MTU_EXCHANGE;
    evt.params.gatt_server_evt.params.requested_mtu = requested_mtu;
    evt.params.gatt_server_evt.params.response_mtu = response_mtu;
    evt.params.gatt_server_evt.params.effective_mtu = effective_mtu;
    return ble_evt_post(&evt);
}

bool ble_evt_notify_gatt_client(const ble_gatt_client_evt_t *p_evt)
{
    ble_deferred_evt_t evt;

    if ((p_evt == NULL) || (m_gatt_client_evt_handler == NULL))
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GATT_CLIENT;
    evt.params.gatt_client = *p_evt;
    return ble_evt_post(&evt);
}

void SWI1_EGU1_IRQHandler(void)
{
    ble_deferred_evt_t evt;

    for (;;)
    {
        uint32_t primask = irq_lock();

        if (m_evt_dispatch.ridx == m_evt_dispatch.widx)
        {
            irq_unlock(primask);
            return;
        }

        evt = m_evt_dispatch.q[m_evt_dispatch.ridx];
        m_evt_dispatch.ridx = (uint8_t)((m_evt_dispatch.ridx + 1U) % BLE_EVT_QUEUE_SIZE);
        irq_unlock(primask);

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_GAP) &&
            (evt.params.gap_evt.evt_type == BLE_GAP_EVT_CONNECTED))
        {
            ble_conn_param_update_timer_start();
        }
        else if ((evt.kind == BLE_DEFERRED_EVT_KIND_GAP) &&
                 (evt.params.gap_evt.evt_type == BLE_GAP_EVT_DISCONNECTED))
        {
            ble_conn_param_update_timer_stop();
        }

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_GAP) &&
            (m_gap_evt_handler != NULL))
        {
            m_gap_evt_handler(&evt.params.gap_evt);
            continue;
        }

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_GATT_SERVER) &&
            (m_gatt_server_evt_handler != NULL))
        {
            m_gatt_server_evt_handler(&evt.params.gatt_server_evt);
            continue;
        }

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_GATT_CHARACTERISTIC) &&
            (evt.params.gatt_characteristic.p_characteristic != NULL) &&
            (evt.params.gatt_characteristic.p_characteristic->evt_handler != NULL))
        {
            ble_gatt_char_evt_t gatt_evt = {
                .evt_type = evt.params.gatt_characteristic.evt_type,
                .p_characteristic = evt.params.gatt_characteristic.p_characteristic,
                .notifications_enabled =
                    (evt.params.gatt_characteristic.evt_type == BLE_GATT_CHAR_EVT_NOTIFY_ENABLED),
                .indications_enabled =
                    (evt.params.gatt_characteristic.evt_type == BLE_GATT_CHAR_EVT_INDICATE_ENABLED),
            };

            evt.params.gatt_characteristic.p_characteristic->evt_handler(&gatt_evt);
            continue;
        }

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_SCAN_REPORT) &&
            (m_scan_report_handler != NULL))
        {
            m_scan_report_handler(&evt.params.scan_report);
            continue;
        }

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_GATT_CLIENT) &&
            (m_gatt_client_evt_handler != NULL))
        {
            m_gatt_client_evt_handler(&evt.params.gatt_client);
        }
    }
}

void controller_load_identity_address(void)
{
    uint64_t seed;
    uint64_t dev_id_seed;
    uint8_t i;

    seed = (uint64_t)NRF_FICR->DEVICEADDR[0] |
           (((uint64_t)NRF_FICR->DEVICEADDR[1] & 0xFFFFULL) << 32);
    dev_id_seed = (uint64_t)NRF_FICR->DEVICEID[0] |
                  ((uint64_t)NRF_FICR->DEVICEID[1] << 32);
    seed ^= dev_id_seed;
    seed ^= BLE_IDENTITY_SALT;

    if ((seed == 0ULL) || (seed == 0xFFFFFFFFFFFFFFFFULL))
    {
        seed = 0x7A6B5C4D3E2FULL;
    }

    for (i = 0U; i < 6U; i++)
    {
        m_ctrl_rt.adv_address[i] = (uint8_t)((seed >> (8U * i)) & 0xFFU);
    }

    m_ctrl_rt.adv_txadd = 1U;
    m_ctrl_rt.adv_address[5] |= 0xC0U;
    if (((m_ctrl_rt.adv_address[0] == 0x00U) &&
         (m_ctrl_rt.adv_address[1] == 0x00U) &&
         (m_ctrl_rt.adv_address[2] == 0x00U) &&
         (m_ctrl_rt.adv_address[3] == 0x00U) &&
         (m_ctrl_rt.adv_address[4] == 0x00U) &&
         (m_ctrl_rt.adv_address[5] == 0x00U)) ||
        ((m_ctrl_rt.adv_address[0] == 0xFFU) &&
         (m_ctrl_rt.adv_address[1] == 0xFFU) &&
         (m_ctrl_rt.adv_address[2] == 0xFFU) &&
         (m_ctrl_rt.adv_address[3] == 0xFFU) &&
         (m_ctrl_rt.adv_address[4] == 0xFFU) &&
         (m_ctrl_rt.adv_address[5] == 0xFFU)))
    {
        m_ctrl_rt.adv_address[0] ^= 0x5AU;
    }
}
