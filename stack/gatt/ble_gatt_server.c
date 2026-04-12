/**
 * @file ble_gatt_server.c
 * @author Surya Poudel
 * @brief ATT and GATT server implementation for nRF BLE stack
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_gatt_server.h"

#include <string.h>

#include "ble_internal.h"

#define BLE_ATT_MTU_DEFAULT 23U

#define BLE_ATT_PERM_READ 0x01U
#define BLE_ATT_PERM_WRITE 0x02U

#define BLE_ATT_ERR_INVALID_HANDLE 0x01U
#define BLE_ATT_ERR_READ_NOT_PERMITTED 0x02U
#define BLE_ATT_ERR_WRITE_NOT_PERMITTED 0x03U
#define BLE_ATT_ERR_INVALID_PDU 0x04U
#define BLE_ATT_ERR_REQUEST_NOT_SUPPORTED 0x06U
#define BLE_ATT_ERR_INVALID_ATTRIBUTE_VALUE_LEN 0x0DU
#define BLE_ATT_ERR_VALUE_NOT_ALLOWED 0x13U
#define BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND 0x0AU
#define BLE_ATT_ERR_UNSUPPORTED_GROUP_TYPE 0x10U

#define BLE_UUID_PRIMARY_SERVICE 0x2800U
#define BLE_UUID_CHARACTERISTIC 0x2803U
#define BLE_UUID_CCCD 0x2902U
#define BLE_UUID_GATT_SERVICE 0x1801U
#define BLE_UUID_DEVICE_NAME 0x2A00U
#define BLE_UUID_APPEARANCE 0x2A01U

#define BLE_ATT_OP_ERROR_RSP 0x01U
#define BLE_ATT_OP_MTU_REQ 0x02U
#define BLE_ATT_OP_MTU_RSP 0x03U
#define BLE_ATT_OP_FIND_INFO_REQ 0x04U
#define BLE_ATT_OP_FIND_INFO_RSP 0x05U
#define BLE_ATT_OP_FIND_TYPE_REQ 0x06U
#define BLE_ATT_OP_FIND_TYPE_RSP 0x07U
#define BLE_ATT_OP_READ_BY_TYPE_REQ 0x08U
#define BLE_ATT_OP_READ_BY_TYPE_RSP 0x09U
#define BLE_ATT_OP_READ_REQ 0x0AU
#define BLE_ATT_OP_READ_RSP 0x0BU
#define BLE_ATT_OP_READ_BY_GROUP_REQ 0x10U
#define BLE_ATT_OP_READ_BY_GROUP_RSP 0x11U
#define BLE_ATT_OP_WRITE_REQ 0x12U
#define BLE_ATT_OP_WRITE_RSP 0x13U
#define BLE_ATT_OP_NOTIFY 0x1BU
#define BLE_ATT_OP_INDICATE 0x1DU
#define BLE_ATT_OP_CONFIRM 0x1EU
#define BLE_ATT_OP_WRITE_CMD 0x52U

#define HANDLE_GAP_SERVICE 0x0001U
#define HANDLE_GAP_DEVNAME_DECL 0x0002U
#define HANDLE_GAP_DEVNAME_VALUE 0x0003U
#define HANDLE_GAP_APPEAR_DECL 0x0004U
#define HANDLE_GAP_APPEAR_VALUE 0x0005U
#define HANDLE_GATT_SERVICE 0x0006U
#define HANDLE_FIRST_CUSTOM 0x0007U

#define BLE_GATT_CHAR_PROP_SUPPORTED                                \
    (BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_WRITE |           \
     BLE_GATT_CHAR_PROP_WRITE_NO_RESP | BLE_GATT_CHAR_PROP_NOTIFY | \
     BLE_GATT_CHAR_PROP_INDICATE)

#define BLE_ATT_GATT_FIXED_ATTR_COUNT 6U
#define BLE_ATT_GATT_MAX_ATTRS \
    (BLE_ATT_GATT_FIXED_ATTR_COUNT + BLE_GATT_MAX_SERVICES + (BLE_GATT_MAX_CHARACTERISTICS * 3U))

typedef struct
{
    uint16_t handle;
    ble_uuid_t uuid;
    uint8_t permissions;
    uint8_t *p_value;
    uint16_t len;
    uint16_t max_len;
    ble_gatt_characteristic_t *p_characteristic;
} gatt_attr_t;

typedef struct
{
    ble_gatt_characteristic_t *p_characteristic;
    uint8_t decl_value[19];
    uint16_t decl_len;
    uint8_t cccd_value[2];
} gatt_char_runtime_t;

typedef struct
{
    uint8_t uuid_value[BLE_UUID128_LEN];
    uint16_t uuid_len;
} gatt_service_runtime_t;

typedef struct
{
    uint8_t gap_service[2];
    uint8_t gap_devname_decl[5];
    uint8_t gap_devname_value[20];
    uint8_t gap_appear_decl[5];
    uint8_t gap_appear_value[2];
    uint8_t gatt_service[2];
} gatt_fixed_attr_values_t;

static uint16_t m_att_mtu = BLE_ATT_MTU_DEFAULT;
static gatt_attr_t m_gatt_db[BLE_ATT_GATT_MAX_ATTRS];
static uint16_t m_gatt_db_count;
static gatt_char_runtime_t m_char_runtime[BLE_GATT_MAX_CHARACTERISTICS];
static uint8_t m_char_runtime_count;
static gatt_service_runtime_t m_service_runtime[BLE_GATT_MAX_SERVICES];
static uint8_t m_service_runtime_count;
static gatt_fixed_attr_values_t m_fixed_attr = {
    .gap_service = {0x00U, 0x18U},
    .gap_devname_decl = {BLE_GATT_CHAR_PROP_READ, HANDLE_GAP_DEVNAME_VALUE, 0x00U, 0x00U, 0x2AU},
    .gap_devname_value = "nrf-ble",
    .gap_appear_decl = {BLE_GATT_CHAR_PROP_READ, HANDLE_GAP_APPEAR_VALUE, 0x00U, 0x01U, 0x2AU},
    .gap_appear_value = {0x00U, 0x00U},
    .gatt_service = {0x01U, 0x18U},
};
static uint16_t m_gap_devname_value_len = 7U;
static bool m_indication_pending;

static uint16_t gatt_local_max_mtu(void)
{
    uint16_t max_octets = BLE_LL_DATA_LEN_DEFAULT_OCTETS;
    uint16_t max_mtu;

    if (m_link.connected)
    {
        max_octets = (m_link.packet.max_tx_octets < m_link.packet.max_rx_octets)
                         ? m_link.packet.max_tx_octets
                         : m_link.packet.max_rx_octets;
    }

    if (max_octets <= BLE_L2CAP_HDR_LEN)
    {
        return BLE_ATT_MTU_DEFAULT;
    }

    max_mtu = (uint16_t)(max_octets - BLE_L2CAP_HDR_LEN);
    if (max_mtu < BLE_ATT_MTU_DEFAULT)
    {
        return BLE_ATT_MTU_DEFAULT;
    }

    return (max_mtu > BLE_ATT_GATT_MAX_MTU) ? BLE_ATT_GATT_MAX_MTU : max_mtu;
}

static uint16_t att_error_rsp(uint8_t *p_rsp, uint16_t max_rsp_len, uint8_t req_opcode, uint16_t handle, uint8_t error_code)
{
    if ((p_rsp == NULL) || (max_rsp_len < 5U))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_ERROR_RSP;
    p_rsp[1] = req_opcode;
    u16_encode(handle, &p_rsp[2]);
    p_rsp[4] = error_code;
    return 5U;
}

static bool gatt_db_add_attr(uint16_t handle, const ble_uuid_t *p_uuid, uint8_t permissions, uint8_t *p_value, uint16_t len, uint16_t max_len, ble_gatt_characteristic_t *p_characteristic)
{
    gatt_attr_t *p_attr;

    if ((m_gatt_db_count >= BLE_ATT_GATT_MAX_ATTRS) || !ble_uuid_is_valid(p_uuid) || (p_value == NULL) || (max_len == 0U))
    {
        return false;
    }

    p_attr = &m_gatt_db[m_gatt_db_count];
    p_attr->handle = handle;
    p_attr->uuid = *p_uuid;
    p_attr->permissions = permissions;
    p_attr->p_value = p_value;
    p_attr->len = len;
    p_attr->max_len = max_len;
    p_attr->p_characteristic = p_characteristic;
    m_gatt_db_count++;
    return true;
}

static gatt_attr_t *att_find_attr(uint16_t handle)
{
    uint16_t i;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if (m_gatt_db[i].handle == handle)
        {
            return &m_gatt_db[i];
        }
    }

    return NULL;
}

static uint16_t gatt_attr_len_get(const gatt_attr_t *p_attr)
{
    if ((p_attr != NULL) && (p_attr->p_characteristic != NULL) && (p_attr->handle == p_attr->p_characteristic->value_handle))
    {
        return p_attr->p_characteristic->value_len;
    }

    return (p_attr != NULL) ? p_attr->len : 0U;
}

static gatt_char_runtime_t *gatt_find_char_runtime(uint16_t handle, bool cccd_handle)
{
    uint8_t i;

    for (i = 0U; i < m_char_runtime_count; i++)
    {
        if ((cccd_handle ? m_char_runtime[i].p_characteristic->cccd_handle
                         : m_char_runtime[i].p_characteristic->value_handle) == handle)
        {
            return &m_char_runtime[i];
        }
    }

    return NULL;
}

static uint16_t att_service_end_handle(uint16_t service_handle)
{
    uint16_t end;
    uint16_t i;

    if (m_gatt_db_count == 0U)
    {
        return 0U;
    }

    end = m_gatt_db[m_gatt_db_count - 1U].handle;
    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if ((m_gatt_db[i].uuid.type == BLE_UUID_TYPE_SIG_16) && (m_gatt_db[i].uuid.value == BLE_UUID_PRIMARY_SERVICE) && (m_gatt_db[i].handle > service_handle))
        {
            end = (uint16_t)(m_gatt_db[i].handle - 1U);
            break;
        }
    }

    return end;
}

static uint16_t att_handle_find_info(uint8_t *p_rsp, uint16_t max_rsp_len, uint16_t start_handle, uint16_t end_handle)
{
    uint16_t out_len = 2U;
    uint16_t i;
    uint16_t uuid_len = 0U;
    uint8_t uuid_format = 0U;
    uint8_t uuid_bytes[BLE_UUID128_LEN];

    if ((p_rsp == NULL) || (max_rsp_len < 2U))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_FIND_INFO_RSP;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if ((m_gatt_db[i].handle < start_handle) || (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }

        if (uuid_len == 0U)
        {
            uuid_len = ble_uuid_encoded_len(&m_gatt_db[i].uuid);
            uuid_format = (uuid_len == BLE_UUID128_LEN) ? 0x02U : 0x01U;
            p_rsp[1] = uuid_format;
        }
        if (ble_uuid_encoded_len(&m_gatt_db[i].uuid) != uuid_len)
        {
            continue;
        }
        if ((out_len + 2U + uuid_len) > max_rsp_len)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &p_rsp[out_len]);
        (void)ble_uuid_encode(&m_gatt_db[i].uuid, uuid_bytes);
        (void)memcpy(&p_rsp[out_len + 2U], uuid_bytes, uuid_len);
        out_len = (uint16_t)(out_len + 2U + uuid_len);
    }

    if (out_len == 2U)
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_FIND_INFO_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_find_by_type_value(uint16_t start_handle, uint16_t end_handle, uint16_t type_uuid, const uint8_t *p_value, uint16_t value_len, uint8_t *p_rsp, uint16_t max_rsp_len)
{
    uint16_t out_len = 1U;
    uint16_t i;

    if ((type_uuid != BLE_UUID_PRIMARY_SERVICE) || (p_value == NULL) || ((value_len != BLE_UUID16_LEN) && (value_len != BLE_UUID128_LEN)))
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_FIND_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }
    if ((p_rsp == NULL) || (max_rsp_len < 1U))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_FIND_TYPE_RSP;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if ((m_gatt_db[i].uuid.type != BLE_UUID_TYPE_SIG_16) ||
            (m_gatt_db[i].uuid.value != BLE_UUID_PRIMARY_SERVICE) ||
            (m_gatt_db[i].handle < start_handle) ||
            (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }
        if ((gatt_attr_len_get(&m_gatt_db[i]) != value_len) || (memcmp(m_gatt_db[i].p_value, p_value, value_len) != 0))
        {
            continue;
        }
        if ((out_len + 4U) > max_rsp_len)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &p_rsp[out_len]);
        u16_encode(att_service_end_handle(m_gatt_db[i].handle), &p_rsp[out_len + 2U]);
        out_len = (uint16_t)(out_len + 4U);
    }

    if (out_len == 1U)
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_FIND_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_read_by_group_type(uint8_t *p_rsp, uint16_t max_rsp_len, uint16_t start_handle, uint16_t end_handle, const uint8_t *p_group_uuid, uint16_t group_uuid_len)
{
    uint16_t out_len = 2U;
    uint16_t entry_len = 0U;
    uint16_t i;

    if ((p_group_uuid == NULL) || (group_uuid_len != BLE_UUID16_LEN) || (u16_decode(p_group_uuid) != BLE_UUID_PRIMARY_SERVICE))
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_BY_GROUP_REQ, start_handle, BLE_ATT_ERR_UNSUPPORTED_GROUP_TYPE);
    }
    if ((p_rsp == NULL) || (max_rsp_len < 2U))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_READ_BY_GROUP_RSP;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        uint16_t value_len;

        if ((m_gatt_db[i].uuid.type != BLE_UUID_TYPE_SIG_16) ||
            (m_gatt_db[i].uuid.value != BLE_UUID_PRIMARY_SERVICE) ||
            (m_gatt_db[i].handle < start_handle) ||
            (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }

        value_len = gatt_attr_len_get(&m_gatt_db[i]);
        if (entry_len == 0U)
        {
            entry_len = (uint16_t)(4U + value_len);
            if ((entry_len + 2U) > max_rsp_len)
            {
                return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_BY_GROUP_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
            }
            p_rsp[1] = (uint8_t)entry_len;
        }

        if ((uint16_t)(4U + value_len) != entry_len)
        {
            continue;
        }
        if ((out_len + entry_len) > max_rsp_len)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &p_rsp[out_len]);
        u16_encode(att_service_end_handle(m_gatt_db[i].handle), &p_rsp[out_len + 2U]);
        (void)memcpy(&p_rsp[out_len + 4U], m_gatt_db[i].p_value, value_len);
        out_len = (uint16_t)(out_len + entry_len);
    }

    if (out_len == 2U)
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_BY_GROUP_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_read_by_type(uint8_t *p_rsp, uint16_t max_rsp_len, uint16_t start_handle, uint16_t end_handle, const uint8_t *p_type_uuid, uint16_t type_uuid_len)
{
    uint16_t out_len = 2U;
    uint16_t entry_len = 0U;
    uint16_t i;

    if ((p_rsp == NULL) || (max_rsp_len < 2U) || (p_type_uuid == NULL) || ((type_uuid_len != BLE_UUID16_LEN) && (type_uuid_len != BLE_UUID128_LEN)))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_READ_BY_TYPE_RSP;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        uint16_t value_len;

        if ((m_gatt_db[i].handle < start_handle) || (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }
        if (!ble_uuid_matches_bytes(&m_gatt_db[i].uuid, p_type_uuid, type_uuid_len))
        {
            continue;
        }
        if ((m_gatt_db[i].permissions & BLE_ATT_PERM_READ) == 0U)
        {
            continue;
        }

        value_len = gatt_attr_len_get(&m_gatt_db[i]);
        if (value_len > m_gatt_db[i].max_len)
        {
            value_len = m_gatt_db[i].max_len;
        }

        if (entry_len == 0U)
        {
            entry_len = (uint16_t)(2U + value_len);
            if ((entry_len + 2U) > max_rsp_len)
            {
                return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_BY_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
            }
            p_rsp[1] = (uint8_t)entry_len;
        }

        if ((uint16_t)(2U + value_len) != entry_len)
        {
            continue;
        }
        if ((out_len + entry_len) > max_rsp_len)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &p_rsp[out_len]);
        (void)memcpy(&p_rsp[out_len + 2U], m_gatt_db[i].p_value, value_len);
        out_len = (uint16_t)(out_len + entry_len);
    }

    if (out_len == 2U)
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_BY_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_read(uint8_t *p_rsp, uint16_t max_rsp_len, uint16_t handle)
{
    gatt_attr_t *p_attr = att_find_attr(handle);
    uint16_t len;

    if (p_attr == NULL)
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_REQ, handle, BLE_ATT_ERR_INVALID_HANDLE);
    }
    if ((p_attr->permissions & BLE_ATT_PERM_READ) == 0U)
    {
        return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_READ_REQ, handle, BLE_ATT_ERR_READ_NOT_PERMITTED);
    }

    len = gatt_attr_len_get(p_attr);
    if (len > p_attr->max_len)
    {
        len = p_attr->max_len;
    }
    if ((uint16_t)(1U + len) > max_rsp_len)
    {
        len = (uint16_t)(max_rsp_len - 1U);
    }

    p_rsp[0] = BLE_ATT_OP_READ_RSP;
    (void)memcpy(&p_rsp[1], p_attr->p_value, len);
    return (uint16_t)(1U + len);
}

static uint16_t att_handle_write_req(uint8_t *p_rsp, uint16_t max_rsp_len, uint16_t handle, const uint8_t *p_data, uint16_t len, bool with_rsp)
{
    gatt_attr_t *p_attr = att_find_attr(handle);
    gatt_char_runtime_t *p_char_runtime;
    uint16_t cccd_allowed = 0U;
    uint8_t old_cccd0 = 0U;

    if (p_attr == NULL)
    {
        return with_rsp ? att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_INVALID_HANDLE) : 0U;
    }
    if ((p_attr->permissions & BLE_ATT_PERM_WRITE) == 0U)
    {
        return with_rsp ? att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_WRITE_NOT_PERMITTED) : 0U;
    }

    p_char_runtime = gatt_find_char_runtime(handle, true);
    if (p_char_runtime != NULL)
    {
        old_cccd0 = p_char_runtime->cccd_value[0];
    }
    if ((p_char_runtime != NULL) && (len != 2U))
    {
        return with_rsp ? att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_INVALID_ATTRIBUTE_VALUE_LEN) : 0U;
    }
    if (p_char_runtime != NULL)
    {
        if ((p_char_runtime->p_characteristic->properties & BLE_GATT_CHAR_PROP_NOTIFY) != 0U)
        {
            cccd_allowed |= 0x0001U;
        }
        if ((p_char_runtime->p_characteristic->properties & BLE_GATT_CHAR_PROP_INDICATE) != 0U)
        {
            cccd_allowed |= 0x0002U;
        }
        if ((u16_decode(p_data) & (uint16_t)(~cccd_allowed)) != 0U)
        {
            return with_rsp ? att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_VALUE_NOT_ALLOWED) : 0U;
        }
    }
    if ((p_char_runtime == NULL) && (p_attr->p_characteristic != NULL))
    {
        if (with_rsp && ((p_attr->p_characteristic->properties & BLE_GATT_CHAR_PROP_WRITE) == 0U))
        {
            return att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_WRITE_NOT_PERMITTED);
        }
        if (!with_rsp && ((p_attr->p_characteristic->properties & BLE_GATT_CHAR_PROP_WRITE_NO_RESP) == 0U))
        {
            return 0U;
        }
    }

    if (len > p_attr->max_len)
    {
        return with_rsp ? att_error_rsp(p_rsp, max_rsp_len, BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_INVALID_ATTRIBUTE_VALUE_LEN) : 0U;
    }
    if ((len > 0U) && (p_data != NULL))
    {
        (void)memcpy(p_attr->p_value, p_data, len);
    }
    p_attr->len = len;
    if ((p_attr->p_characteristic != NULL) && (handle == p_attr->p_characteristic->value_handle))
    {
        p_attr->p_characteristic->value_len = len;
    }

    if (p_char_runtime != NULL)
    {
        if ((old_cccd0 & 0x01U) != (p_char_runtime->cccd_value[0] & 0x01U))
        {
            (void)ble_evt_notify_gatt_characteristic((p_char_runtime->cccd_value[0] & 0x01U) != 0U
                                                         ? BLE_GATT_CHAR_EVT_NOTIFY_ENABLED
                                                         : BLE_GATT_CHAR_EVT_NOTIFY_DISABLED,
                                                     p_char_runtime->p_characteristic);
        }
        if ((old_cccd0 & 0x02U) != (p_char_runtime->cccd_value[0] & 0x02U))
        {
            (void)ble_evt_notify_gatt_characteristic((p_char_runtime->cccd_value[0] & 0x02U) != 0U
                                                         ? BLE_GATT_CHAR_EVT_INDICATE_ENABLED
                                                         : BLE_GATT_CHAR_EVT_INDICATE_DISABLED,
                                                     p_char_runtime->p_characteristic);
        }
    }
    else if (p_attr->p_characteristic != NULL)
    {
        (void)ble_evt_notify_gatt_characteristic(BLE_GATT_CHAR_EVT_WRITE, p_attr->p_characteristic);
    }

    if (!with_rsp)
    {
        return 0U;
    }

    if ((p_rsp == NULL) || (max_rsp_len < 1U))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_WRITE_RSP;
    return 1U;
}

static bool gatt_build_fixed_attrs(void)
{
    return gatt_db_add_attr(HANDLE_GAP_SERVICE, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_PRIMARY_SERVICE), BLE_ATT_PERM_READ, m_fixed_attr.gap_service, sizeof(m_fixed_attr.gap_service), sizeof(m_fixed_attr.gap_service), NULL) &&
           gatt_db_add_attr(HANDLE_GAP_DEVNAME_DECL, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_CHARACTERISTIC), BLE_ATT_PERM_READ, m_fixed_attr.gap_devname_decl, sizeof(m_fixed_attr.gap_devname_decl), sizeof(m_fixed_attr.gap_devname_decl), NULL) &&
           gatt_db_add_attr(HANDLE_GAP_DEVNAME_VALUE, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_DEVICE_NAME), BLE_ATT_PERM_READ, m_fixed_attr.gap_devname_value, m_gap_devname_value_len, sizeof(m_fixed_attr.gap_devname_value), NULL) &&
           gatt_db_add_attr(HANDLE_GAP_APPEAR_DECL, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_CHARACTERISTIC), BLE_ATT_PERM_READ, m_fixed_attr.gap_appear_decl, sizeof(m_fixed_attr.gap_appear_decl), sizeof(m_fixed_attr.gap_appear_decl), NULL) &&
           gatt_db_add_attr(HANDLE_GAP_APPEAR_VALUE, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_APPEARANCE), BLE_ATT_PERM_READ, m_fixed_attr.gap_appear_value, sizeof(m_fixed_attr.gap_appear_value), sizeof(m_fixed_attr.gap_appear_value), NULL) &&
           gatt_db_add_attr(HANDLE_GATT_SERVICE, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_PRIMARY_SERVICE), BLE_ATT_PERM_READ, m_fixed_attr.gatt_service, sizeof(m_fixed_attr.gatt_service), sizeof(m_fixed_attr.gatt_service), NULL);
}

static bool gatt_add_service(ble_gatt_service_t *p_service, uint16_t *p_next_handle)
{
    gatt_service_runtime_t *p_service_runtime;

    if ((p_service->p_characteristics == NULL) || (p_service->characteristic_count == 0U))
    {
        return false;
    }
    if (!ble_uuid_is_valid(&p_service->uuid) || (m_service_runtime_count >= BLE_GATT_MAX_SERVICES))
    {
        return false;
    }

    p_service->service_handle = *p_next_handle;
    p_service_runtime = &m_service_runtime[m_service_runtime_count];
    p_service_runtime->uuid_len = ble_uuid_encoded_len(&p_service->uuid);
    if (!ble_uuid_encode(&p_service->uuid, p_service_runtime->uuid_value))
    {
        return false;
    }

    if (!gatt_db_add_attr(*p_next_handle, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_PRIMARY_SERVICE), BLE_ATT_PERM_READ, p_service_runtime->uuid_value, p_service_runtime->uuid_len, p_service_runtime->uuid_len, NULL))
    {
        return false;
    }

    m_service_runtime_count++;
    (*p_next_handle)++;
    return true;
}

static bool gatt_add_characteristic(ble_gatt_characteristic_t *p_characteristic, uint16_t *p_next_handle)
{
    gatt_char_runtime_t *p_char_runtime;
    uint8_t permissions = 0U;

    if ((p_characteristic->p_value == NULL) ||
        !ble_uuid_is_valid(&p_characteristic->uuid) ||
        (p_characteristic->max_len == 0U) ||
        (p_characteristic->max_len > BLE_GATT_MAX_VALUE_LEN) ||
        (p_characteristic->properties == 0U) ||
        ((p_characteristic->properties & (uint8_t)(~BLE_GATT_CHAR_PROP_SUPPORTED)) != 0U) ||
        (m_char_runtime_count >= BLE_GATT_MAX_CHARACTERISTICS))
    {
        return false;
    }
    if (p_characteristic->value_len > p_characteristic->max_len)
    {
        p_characteristic->value_len = p_characteristic->max_len;
    }
    if ((p_characteristic->properties & BLE_GATT_CHAR_PROP_READ) != 0U)
    {
        permissions |= BLE_ATT_PERM_READ;
    }
    if ((p_characteristic->properties & (BLE_GATT_CHAR_PROP_WRITE | BLE_GATT_CHAR_PROP_WRITE_NO_RESP)) != 0U)
    {
        permissions |= BLE_ATT_PERM_WRITE;
    }

    p_char_runtime = &m_char_runtime[m_char_runtime_count];
    (void)memset(p_char_runtime, 0, sizeof(*p_char_runtime));
    p_char_runtime->p_characteristic = p_characteristic;
    p_char_runtime->decl_value[0] = p_characteristic->properties;
    u16_encode((uint16_t)(*p_next_handle + 1U), &p_char_runtime->decl_value[1]);
    if (!ble_uuid_encode(&p_characteristic->uuid, &p_char_runtime->decl_value[3]))
    {
        return false;
    }
    p_char_runtime->decl_len = (uint16_t)(3U + ble_uuid_encoded_len(&p_characteristic->uuid));

    p_characteristic->value_handle = (uint16_t)(*p_next_handle + 1U);
    p_characteristic->cccd_handle = 0U;

    if (!gatt_db_add_attr(*p_next_handle, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_CHARACTERISTIC), BLE_ATT_PERM_READ, p_char_runtime->decl_value, p_char_runtime->decl_len, p_char_runtime->decl_len, NULL))
    {
        return false;
    }

    (*p_next_handle)++;

    if (!gatt_db_add_attr(*p_next_handle, &p_characteristic->uuid, permissions, p_characteristic->p_value, p_characteristic->value_len, p_characteristic->max_len, p_characteristic))
    {
        return false;
    }

    (*p_next_handle)++;

    if ((p_characteristic->properties & (BLE_GATT_CHAR_PROP_NOTIFY | BLE_GATT_CHAR_PROP_INDICATE)) != 0U)
    {
        p_characteristic->cccd_handle = *p_next_handle;
        if (!gatt_db_add_attr(*p_next_handle, &(ble_uuid_t)BLE_UUID_SIG16_INIT(BLE_UUID_CCCD), (uint8_t)(BLE_ATT_PERM_READ | BLE_ATT_PERM_WRITE), p_char_runtime->cccd_value, sizeof(p_char_runtime->cccd_value), sizeof(p_char_runtime->cccd_value), p_characteristic))
        {
            return false;
        }
        (*p_next_handle)++;
    }

    m_char_runtime_count++;
    return true;
}

static bool gatt_build_custom_attrs(ble_gatt_service_t *p_services, uint8_t service_count)
{
    uint16_t next_handle = HANDLE_FIRST_CUSTOM;
    uint8_t service_idx;

    if ((p_services == NULL) || (service_count == 0U))
    {
        return true;
    }

    if (service_count > BLE_GATT_MAX_SERVICES)
    {
        return false;
    }

    for (service_idx = 0U; service_idx < service_count; service_idx++)
    {
        ble_gatt_service_t *p_service = &p_services[service_idx];
        uint8_t char_idx;

        if (!gatt_add_service(p_service, &next_handle))
        {
            return false;
        }

        for (char_idx = 0U; char_idx < p_service->characteristic_count; char_idx++)
        {
            if (!gatt_add_characteristic(&p_service->p_characteristics[char_idx], &next_handle))
            {
                return false;
            }
        }
    }

    return true;
}

bool ble_gatt_server_init(ble_gatt_service_t *p_services, uint8_t service_count)
{
    const char *p_name = (m_host.adv_name[0] != '\0') ? m_host.adv_name : "nrf-ble";
    size_t name_len;
    uint8_t service_idx;

    m_att_mtu = BLE_ATT_MTU_DEFAULT;
    m_gatt_db_count = 0U;
    m_char_runtime_count = 0U;
    m_service_runtime_count = 0U;
    (void)memset(m_gatt_db, 0, sizeof(m_gatt_db));
    (void)memset(m_char_runtime, 0, sizeof(m_char_runtime));
    (void)memset(m_service_runtime, 0, sizeof(m_service_runtime));

    name_len = strlen(p_name);
    if (name_len > sizeof(m_fixed_attr.gap_devname_value))
    {
        name_len = sizeof(m_fixed_attr.gap_devname_value);
    }

    (void)memset(m_fixed_attr.gap_devname_value, 0, sizeof(m_fixed_attr.gap_devname_value));
    (void)memcpy(m_fixed_attr.gap_devname_value, p_name, name_len);
    m_gap_devname_value_len = (uint16_t)name_len;

    if (p_services != NULL)
    {
        for (service_idx = 0U; service_idx < service_count; service_idx++)
        {
            ble_gatt_service_t *p_service = &p_services[service_idx];
            uint8_t char_idx;

            p_service->service_handle = 0U;
            for (char_idx = 0U; char_idx < p_service->characteristic_count; char_idx++)
            {
                p_service->p_characteristics[char_idx].value_handle = 0U;
                p_service->p_characteristics[char_idx].cccd_handle = 0U;
            }
        }
    }

    if (!gatt_build_fixed_attrs() || !gatt_build_custom_attrs(p_services, service_count))
    {
        return false;
    }

    ble_gatt_server_reset_connection_state();
    return true;
}

void ble_gatt_server_reset_connection_state(void)
{
    uint8_t i;

    m_att_mtu = BLE_ATT_MTU_DEFAULT;
    m_indication_pending = false;

    for (i = 0U; i < m_char_runtime_count; i++)
    {
        m_char_runtime[i].cccd_value[0] = 0U;
        m_char_runtime[i].cccd_value[1] = 0U;
    }
}

static uint16_t ble_gatt_server_build_value_pdu(uint16_t value_handle, uint8_t required_property, uint8_t cccd_mask, uint8_t opcode, uint8_t *p_att, uint16_t max_len)
{
    gatt_char_runtime_t *p_char_runtime = gatt_find_char_runtime(value_handle, false);
    uint16_t value_len;

    if (max_len > m_att_mtu)
    {
        max_len = m_att_mtu;
    }

    if ((p_att == NULL) ||
        (p_char_runtime == NULL) ||
        ((p_char_runtime->p_characteristic->properties & required_property) == 0U) ||
        ((p_char_runtime->cccd_value[0] & cccd_mask) == 0U))
    {
        return 0U;
    }

    value_len = p_char_runtime->p_characteristic->value_len;
    if (value_len > p_char_runtime->p_characteristic->max_len)
    {
        value_len = p_char_runtime->p_characteristic->max_len;
    }
    if ((uint16_t)(3U + value_len) > max_len)
    {
        return 0U;
    }

    p_att[0] = opcode;
    u16_encode(value_handle, &p_att[1]);
    (void)memcpy(&p_att[3], p_char_runtime->p_characteristic->p_value, value_len);
    return (uint16_t)(3U + value_len);
}

uint16_t ble_gatt_server_build_notification(uint16_t value_handle, uint8_t *p_att, uint16_t max_len)
{
    return ble_gatt_server_build_value_pdu(value_handle, BLE_GATT_CHAR_PROP_NOTIFY, 0x01U, BLE_ATT_OP_NOTIFY, p_att, max_len);
}

uint16_t ble_gatt_server_build_indication(uint16_t value_handle, uint8_t *p_att, uint16_t max_len)
{
    if (m_indication_pending)
    {
        return 0U;
    }

    return ble_gatt_server_build_value_pdu(value_handle, BLE_GATT_CHAR_PROP_INDICATE, 0x02U, BLE_ATT_OP_INDICATE, p_att, max_len);
}

void ble_gatt_server_mark_indication_pending(void)
{
    m_indication_pending = true;
}

uint16_t ble_gatt_server_process_request(const uint8_t *p_att, uint16_t att_len, uint8_t *p_rsp, uint16_t rsp_max_len)
{
    uint8_t opcode;
    uint16_t max_rsp_len;
    uint16_t rsp_len = 0U;

    if ((p_rsp == NULL) || (rsp_max_len == 0U) || (p_att == NULL) || (att_len == 0U))
    {
        return 0U;
    }

    max_rsp_len = (rsp_max_len < m_att_mtu) ? rsp_max_len : m_att_mtu;
    opcode = p_att[0];
    switch (opcode)
    {
    case BLE_ATT_OP_MTU_REQ:
    {
        uint16_t local_mtu;
        uint16_t requested_mtu;

        if (att_len < 3U)
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        requested_mtu = u16_decode(&p_att[1]);
        local_mtu = gatt_local_max_mtu();
        m_att_mtu = (requested_mtu < local_mtu) ? requested_mtu : local_mtu;
        if (max_rsp_len < 3U)
        {
            return 0U;
        }
        p_rsp[0] = BLE_ATT_OP_MTU_RSP;
        u16_encode(local_mtu, &p_rsp[1]);
        (void)ble_evt_notify_gatt_mtu_exchange(requested_mtu, local_mtu, m_att_mtu);
        rsp_len = 3U;
        break;
    }

    case BLE_ATT_OP_FIND_INFO_REQ:
        if (att_len < 5U)
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_find_info(p_rsp, max_rsp_len, u16_decode(&p_att[1]), u16_decode(&p_att[3]));
        break;

    case BLE_ATT_OP_FIND_TYPE_REQ:
        if (att_len < 7U)
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_find_by_type_value(u16_decode(&p_att[1]),
                                                u16_decode(&p_att[3]),
                                                u16_decode(&p_att[5]),
                                                &p_att[7],
                                                (uint16_t)(att_len - 7U),
                                                p_rsp,
                                                max_rsp_len);
        break;

    case BLE_ATT_OP_READ_BY_GROUP_REQ:
        if ((att_len < 7U) || (((uint16_t)(att_len - 5U) != BLE_UUID16_LEN) && ((uint16_t)(att_len - 5U) != BLE_UUID128_LEN)))
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_read_by_group_type(p_rsp, max_rsp_len, u16_decode(&p_att[1]), u16_decode(&p_att[3]), &p_att[5], (uint16_t)(att_len - 5U));
        break;

    case BLE_ATT_OP_READ_BY_TYPE_REQ:
        if ((att_len < 7U) || (((uint16_t)(att_len - 5U) != BLE_UUID16_LEN) && ((uint16_t)(att_len - 5U) != BLE_UUID128_LEN)))
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_read_by_type(p_rsp, max_rsp_len, u16_decode(&p_att[1]), u16_decode(&p_att[3]), &p_att[5], (uint16_t)(att_len - 5U));
        break;

    case BLE_ATT_OP_READ_REQ:
        if (att_len < 3U)
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_read(p_rsp, max_rsp_len, u16_decode(&p_att[1]));
        break;

    case BLE_ATT_OP_WRITE_REQ:
        if (att_len < 3U)
        {
            rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_write_req(p_rsp, max_rsp_len, u16_decode(&p_att[1]), &p_att[3], (uint16_t)(att_len - 3U), true);
        break;

    case BLE_ATT_OP_WRITE_CMD:
        if (att_len >= 3U)
        {
            (void)att_handle_write_req(NULL, 0U, u16_decode(&p_att[1]), &p_att[3], (uint16_t)(att_len - 3U), false);
        }
        return 0U;

    case BLE_ATT_OP_CONFIRM:
        if (att_len == 1U)
        {
            m_indication_pending = false;
        }
        return 0U;

    default:
        rsp_len = att_error_rsp(p_rsp, max_rsp_len, opcode, 0x0000U, BLE_ATT_ERR_REQUEST_NOT_SUPPORTED);
        break;
    }

    return rsp_len;
}
