/**
 * @file ble_uuid.c
 * @author Surya Poudel
 * @brief Shared UUID helpers for nRF BLE stack
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_runtime_internal.h"

#include <string.h>

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

    if (p_uuid->type == BLE_UUID_TYPE_RAW_128)
    {
        return BLE_UUID128_LEN;
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
        u16_encode(p_uuid->value.uuid16, p_dst);
        return true;
    }

    if (p_uuid->type == BLE_UUID_TYPE_RAW_128)
    {
        (void)memcpy(p_dst, p_uuid->value.uuid128, BLE_UUID128_LEN);
        return true;
    }

    if ((p_uuid->type == BLE_UUID_TYPE_VENDOR_16) && m_host.vendor_uuid_base_set)
    {
        (void)memcpy(p_dst, m_host.vendor_uuid_base, BLE_UUID128_LEN);
        /* BLE carries 128-bit UUIDs in little-endian byte order. Patching
         * bytes 12..13 here corresponds to the canonical 0000XXXX-... slot. */
        u16_encode(p_uuid->value.uuid16, &p_dst[12]);
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
