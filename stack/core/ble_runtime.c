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

#include "ble_internal.h"

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
ble_controller_t m_controller;
ble_link_t m_link;
ble_evt_handler_t m_evt_handler;
ble_ctrl_runtime_t m_ctrl_rt;
static ble_evt_dispatch_state_t m_evt_dispatch;

static bool ble_evt_queue_push(const ble_deferred_evt_t *p_evt)
{
    uint32_t primask;
    uint8_t next_widx;

    if (p_evt == NULL)
    {
        return false;
    }

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
    return true;
}

static bool ble_evt_post(const ble_deferred_evt_t *p_evt)
{
    if (!ble_evt_queue_push(p_evt))
    {
        return false;
    }

    NVIC_SetPendingIRQ(SWI1_EGU1_IRQn);
    return true;
}

static bool ble_evt_queue_pop(ble_deferred_evt_t *p_evt)
{
    uint32_t primask;

    if (p_evt == NULL)
    {
        return false;
    }

    primask = irq_lock();

    if (m_evt_dispatch.ridx == m_evt_dispatch.widx)
    {
        irq_unlock(primask);
        return false;
    }

    *p_evt = m_evt_dispatch.q[m_evt_dispatch.ridx];
    m_evt_dispatch.ridx = (uint8_t)((m_evt_dispatch.ridx + 1U) % BLE_EVT_QUEUE_SIZE);
    irq_unlock(primask);
    return true;
}

static void ble_evt_init(ble_evt_t *p_evt, ble_evt_type_t evt_type)
{
    (void)memset(p_evt, 0, sizeof(*p_evt));
    p_evt->evt_type = evt_type;
    p_evt->conn_interval_ms = m_link.conn_interval_ms;
    p_evt->supervision_timeout_ms = m_link.supervision_timeout_ms;
}

uint16_t u16_decode(const uint8_t *p_src)
{
    return (uint16_t)p_src[0] | ((uint16_t)p_src[1] << 8);
}

uint8_t adv_pdu_type_get(const ble_ll_adv_pdu_t *p_pdu)
{
    return p_pdu->header.pdu_type;
}

static bool addr_is_all(const uint8_t *addr, uint8_t value)
{
    uint8_t i;

    for (i = 0U; i < 6U; i++)
    {
        if (addr[i] != value)
        {
            return false;
        }
    }

    return true;
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

bool ble_evt_notify_gap(ble_evt_type_t evt_type)
{
    ble_deferred_evt_t evt;

    if ((m_evt_handler == NULL) || (evt_type == BLE_GATT_EVT_MTU_EXCHANGE))
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GAP;
    ble_evt_init(&evt.params.stack_evt, evt_type);
    return ble_evt_post(&evt);
}

bool ble_evt_notify_gatt_characteristic(ble_gatt_evt_type_t evt_type,
                                        ble_gatt_characteristic_t *p_characteristic,
                                        const uint8_t *p_data,
                                        uint16_t len,
                                        bool notifications_enabled)
{
    ble_deferred_evt_t evt;

    if ((p_characteristic == NULL) || (p_characteristic->evt_handler == NULL))
    {
        return false;
    }

    if (len > BLE_GATT_MAX_VALUE_LEN)
    {
        len = BLE_GATT_MAX_VALUE_LEN;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GATT_CHARACTERISTIC;
    evt.params.gatt_characteristic.evt_type = evt_type;
    evt.params.gatt_characteristic.p_characteristic = p_characteristic;
    if ((p_data != NULL) && (len > 0U))
    {
        (void)memcpy(evt.params.gatt_characteristic.data, p_data, len);
    }
    evt.params.gatt_characteristic.len = len;
    evt.params.gatt_characteristic.notifications_enabled = notifications_enabled;
    return ble_evt_post(&evt);
}

bool ble_evt_notify_gatt_mtu_exchange(uint16_t requested_mtu,
                                      uint16_t response_mtu,
                                      uint16_t effective_mtu)
{
    ble_deferred_evt_t evt;

    if (m_evt_handler == NULL)
    {
        return false;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.kind = BLE_DEFERRED_EVT_KIND_GATT;
    ble_evt_init(&evt.params.stack_evt, BLE_GATT_EVT_MTU_EXCHANGE);
    evt.params.stack_evt.requested_mtu = requested_mtu;
    evt.params.stack_evt.response_mtu = response_mtu;
    evt.params.stack_evt.effective_mtu = effective_mtu;
    return ble_evt_post(&evt);
}

static void ble_gatt_evt_dispatch(const ble_deferred_evt_t *p_evt)
{
    ble_gatt_evt_t gatt_evt = {
        .evt_type = p_evt->params.gatt_characteristic.evt_type,
        .p_characteristic = p_evt->params.gatt_characteristic.p_characteristic,
        .p_data = (p_evt->params.gatt_characteristic.evt_type == BLE_GATT_EVT_WRITE) ?
                      p_evt->params.gatt_characteristic.data :
                      NULL,
        .len = p_evt->params.gatt_characteristic.len,
        .notifications_enabled = p_evt->params.gatt_characteristic.notifications_enabled,
    };

    p_evt->params.gatt_characteristic.p_characteristic->evt_handler(&gatt_evt);
}

void SWI1_EGU1_IRQHandler(void)
{
    ble_deferred_evt_t evt;

    while (ble_evt_queue_pop(&evt))
    {
        if (((evt.kind == BLE_DEFERRED_EVT_KIND_GAP) ||
             (evt.kind == BLE_DEFERRED_EVT_KIND_GATT)) &&
            (m_evt_handler != NULL))
        {
            m_evt_handler(&evt.params.stack_evt);
            continue;
        }

        if ((evt.kind == BLE_DEFERRED_EVT_KIND_GATT_CHARACTERISTIC) &&
            (evt.params.gatt_characteristic.p_characteristic != NULL) &&
            (evt.params.gatt_characteristic.p_characteristic->evt_handler != NULL))
        {
            ble_gatt_evt_dispatch(&evt);
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
    if (addr_is_all(m_ctrl_rt.adv_address, 0x00U) || addr_is_all(m_ctrl_rt.adv_address, 0xFFU))
    {
        m_ctrl_rt.adv_address[0] ^= 0x5AU;
    }
}
