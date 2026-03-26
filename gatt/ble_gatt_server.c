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

#define BLE_ATT_PERM_READ  0x01U
#define BLE_ATT_PERM_WRITE 0x02U

#define BLE_ATT_ERR_INVALID_HANDLE              0x01U
#define BLE_ATT_ERR_READ_NOT_PERMITTED         0x02U
#define BLE_ATT_ERR_WRITE_NOT_PERMITTED        0x03U
#define BLE_ATT_ERR_INVALID_PDU                0x04U
#define BLE_ATT_ERR_REQUEST_NOT_SUPPORTED      0x06U
#define BLE_ATT_ERR_INVALID_ATTRIBUTE_VALUE_LEN 0x0DU
#define BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND        0x0AU
#define BLE_ATT_ERR_UNSUPPORTED_GROUP_TYPE     0x10U

#define BLE_UUID_PRIMARY_SERVICE 0x2800U
#define BLE_UUID_CHARACTERISTIC  0x2803U
#define BLE_UUID_CCCD            0x2902U
#define BLE_UUID_GATT_SERVICE    0x1801U
#define BLE_UUID_DEVICE_NAME     0x2A00U
#define BLE_UUID_APPEARANCE      0x2A01U

#define BLE_ATT_OP_ERROR_RSP         0x01U
#define BLE_ATT_OP_MTU_REQ           0x02U
#define BLE_ATT_OP_MTU_RSP           0x03U
#define BLE_ATT_OP_FIND_INFO_REQ     0x04U
#define BLE_ATT_OP_FIND_INFO_RSP     0x05U
#define BLE_ATT_OP_FIND_TYPE_REQ     0x06U
#define BLE_ATT_OP_FIND_TYPE_RSP     0x07U
#define BLE_ATT_OP_READ_BY_TYPE_REQ  0x08U
#define BLE_ATT_OP_READ_BY_TYPE_RSP  0x09U
#define BLE_ATT_OP_READ_REQ          0x0AU
#define BLE_ATT_OP_READ_RSP          0x0BU
#define BLE_ATT_OP_READ_BY_GROUP_REQ 0x10U
#define BLE_ATT_OP_READ_BY_GROUP_RSP 0x11U
#define BLE_ATT_OP_WRITE_REQ         0x12U
#define BLE_ATT_OP_WRITE_RSP         0x13U
#define BLE_ATT_OP_NOTIFY            0x1BU
#define BLE_ATT_OP_WRITE_CMD         0x52U

#define HANDLE_GAP_SERVICE      0x0001U
#define HANDLE_GAP_DEVNAME_DECL 0x0002U
#define HANDLE_GAP_DEVNAME_VALUE 0x0003U
#define HANDLE_GAP_APPEAR_DECL  0x0004U
#define HANDLE_GAP_APPEAR_VALUE 0x0005U
#define HANDLE_GATT_SERVICE     0x0006U
#define HANDLE_FIRST_CUSTOM     0x0007U

#define BLE_GATT_CHAR_PROP_SUPPORTED \
    (BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_WRITE | \
     BLE_GATT_CHAR_PROP_WRITE_NO_RESP | BLE_GATT_CHAR_PROP_NOTIFY)

#define BLE_ATT_GATT_FIXED_ATTR_COUNT 6U
#define BLE_ATT_GATT_MAX_ATTRS \
    (BLE_ATT_GATT_FIXED_ATTR_COUNT + BLE_GATT_MAX_SERVICES + (BLE_GATT_MAX_CHARACTERISTICS * 3U))

typedef struct
{
    uint16_t handle;
    uint16_t uuid;
    uint8_t permissions;
    uint8_t *p_value;
    uint16_t *p_len;
    uint16_t max_len;
    ble_gatt_characteristic_t *p_characteristic;
} gatt_attr_t;

typedef struct
{
    ble_gatt_service_t *p_service;
    uint8_t value[2];
    uint16_t value_len;
} gatt_service_runtime_t;

typedef struct
{
    ble_gatt_service_t *p_service;
    ble_gatt_characteristic_t *p_characteristic;
    uint8_t decl_value[5];
    uint16_t decl_len;
    uint8_t cccd_value[2];
    uint16_t cccd_len;
    bool notifications_enabled;
} gatt_char_runtime_t;

static uint16_t m_att_mtu = BLE_ATT_MTU_DEFAULT;
static uint8_t m_att_rsp[BLE_ATT_GATT_MAX_MTU];
static gatt_attr_t m_gatt_db[BLE_ATT_GATT_MAX_ATTRS];
static uint16_t m_gatt_db_count;
static gatt_service_runtime_t m_service_runtime[BLE_GATT_MAX_SERVICES];
static uint8_t m_service_runtime_count;
static gatt_char_runtime_t m_char_runtime[BLE_GATT_MAX_CHARACTERISTICS];
static uint8_t m_char_runtime_count;

static uint8_t m_attr_gap_service[] = {0x00U, 0x18U};
static uint16_t m_attr_gap_service_len = 2U;
static uint8_t m_attr_gap_devname_decl[] = {BLE_GATT_CHAR_PROP_READ, HANDLE_GAP_DEVNAME_VALUE, 0x00U, 0x00U, 0x2AU};
static uint16_t m_attr_gap_devname_decl_len = 5U;
static uint8_t m_attr_gap_devname_value[20] = "nrf52-ble";
static uint16_t m_attr_gap_devname_value_len = 8U;
static uint8_t m_attr_gap_appear_decl[] = {BLE_GATT_CHAR_PROP_READ, HANDLE_GAP_APPEAR_VALUE, 0x00U, 0x01U, 0x2AU};
static uint16_t m_attr_gap_appear_decl_len = 5U;
static uint8_t m_attr_gap_appear_value[] = {0x00U, 0x00U};
static uint16_t m_attr_gap_appear_value_len = 2U;
static uint8_t m_attr_gatt_service[] = {0x01U, 0x18U};
static uint16_t m_attr_gatt_service_len = 2U;

static uint16_t att_error_rsp(uint8_t req_opcode, uint16_t handle, uint8_t error_code)
{
    m_att_rsp[0] = BLE_ATT_OP_ERROR_RSP;
    m_att_rsp[1] = req_opcode;
    u16_encode(handle, &m_att_rsp[2]);
    m_att_rsp[4] = error_code;
    return 5U;
}

static bool gatt_db_add_attr(uint16_t handle,
                             uint16_t uuid,
                             uint8_t permissions,
                             uint8_t *p_value,
                             uint16_t *p_len,
                             uint16_t max_len,
                             ble_gatt_characteristic_t *p_characteristic)
{
    gatt_attr_t *p_attr;

    if ((m_gatt_db_count >= BLE_ATT_GATT_MAX_ATTRS) || (p_value == NULL) || (p_len == NULL) || (max_len == 0U))
    {
        return false;
    }

    p_attr = &m_gatt_db[m_gatt_db_count];
    p_attr->handle = handle;
    p_attr->uuid = uuid;
    p_attr->permissions = permissions;
    p_attr->p_value = p_value;
    p_attr->p_len = p_len;
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

static gatt_char_runtime_t *gatt_find_char_runtime_by_value_handle(uint16_t value_handle)
{
    uint8_t i;

    for (i = 0U; i < m_char_runtime_count; i++)
    {
        if (m_char_runtime[i].p_characteristic->value_handle == value_handle)
        {
            return &m_char_runtime[i];
        }
    }

    return NULL;
}

static gatt_char_runtime_t *gatt_find_char_runtime_by_cccd_handle(uint16_t cccd_handle)
{
    uint8_t i;

    for (i = 0U; i < m_char_runtime_count; i++)
    {
        if (m_char_runtime[i].p_characteristic->cccd_handle == cccd_handle)
        {
            return &m_char_runtime[i];
        }
    }

    return NULL;
}

static gatt_char_runtime_t *gatt_find_char_runtime_by_characteristic(const ble_gatt_characteristic_t *p_characteristic)
{
    uint8_t i;

    if (p_characteristic == NULL)
    {
        return NULL;
    }

    for (i = 0U; i < m_char_runtime_count; i++)
    {
        if (m_char_runtime[i].p_characteristic == p_characteristic)
        {
            return &m_char_runtime[i];
        }
    }

    return NULL;
}

static uint8_t gatt_props_to_permissions(uint8_t properties)
{
    uint8_t permissions = 0U;

    if ((properties & BLE_GATT_CHAR_PROP_READ) != 0U)
    {
        permissions |= BLE_ATT_PERM_READ;
    }
    if ((properties & (BLE_GATT_CHAR_PROP_WRITE | BLE_GATT_CHAR_PROP_WRITE_NO_RESP)) != 0U)
    {
        permissions |= BLE_ATT_PERM_WRITE;
    }

    return permissions;
}

static void gatt_reset_runtime_tables(void)
{
    m_att_mtu = BLE_ATT_MTU_DEFAULT;
    m_gatt_db_count = 0U;
    m_service_runtime_count = 0U;
    m_char_runtime_count = 0U;
    (void)memset(m_gatt_db, 0, sizeof(m_gatt_db));
    (void)memset(m_service_runtime, 0, sizeof(m_service_runtime));
    (void)memset(m_char_runtime, 0, sizeof(m_char_runtime));
}

static void gatt_reset_user_handles(ble_gatt_service_t *p_services, uint8_t service_count)
{
    uint8_t service_idx;

    if (p_services == NULL)
    {
        return;
    }

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

static bool gatt_characteristic_is_valid(ble_gatt_characteristic_t *p_characteristic)
{
    if ((p_characteristic->p_value == NULL) ||
        (p_characteristic->p_value_len == NULL) ||
        (p_characteristic->max_len == 0U) ||
        (p_characteristic->max_len > BLE_GATT_MAX_VALUE_LEN))
    {
        return false;
    }

    if ((p_characteristic->properties == 0U) ||
        ((p_characteristic->properties & (uint8_t)(~BLE_GATT_CHAR_PROP_SUPPORTED)) != 0U))
    {
        return false;
    }

    if (*p_characteristic->p_value_len > p_characteristic->max_len)
    {
        *p_characteristic->p_value_len = p_characteristic->max_len;
    }

    return true;
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
        if ((m_gatt_db[i].uuid == BLE_UUID_PRIMARY_SERVICE) &&
            (m_gatt_db[i].handle > service_handle))
        {
            end = (uint16_t)(m_gatt_db[i].handle - 1U);
            break;
        }
    }

    return end;
}

static uint16_t att_handle_find_info(uint16_t start_handle, uint16_t end_handle)
{
    uint16_t out_len = 2U;
    uint16_t i;

    m_att_rsp[0] = BLE_ATT_OP_FIND_INFO_RSP;
    m_att_rsp[1] = 0x01U;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if ((m_gatt_db[i].handle < start_handle) || (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }
        if ((out_len + 4U) > m_att_mtu)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &m_att_rsp[out_len]);
        u16_encode(m_gatt_db[i].uuid, &m_att_rsp[out_len + 2U]);
        out_len = (uint16_t)(out_len + 4U);
    }

    if (out_len == 2U)
    {
        return att_error_rsp(BLE_ATT_OP_FIND_INFO_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_find_by_type_value(uint16_t start_handle,
                                              uint16_t end_handle,
                                              uint16_t type_uuid,
                                              const uint8_t *p_value,
                                              uint16_t value_len)
{
    uint16_t out_len = 1U;
    uint16_t i;

    if ((type_uuid != BLE_UUID_PRIMARY_SERVICE) || (p_value == NULL) || (value_len != 2U))
    {
        return att_error_rsp(BLE_ATT_OP_FIND_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    m_att_rsp[0] = BLE_ATT_OP_FIND_TYPE_RSP;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if ((m_gatt_db[i].uuid != BLE_UUID_PRIMARY_SERVICE) ||
            (m_gatt_db[i].handle < start_handle) ||
            (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }
        if (u16_decode(m_gatt_db[i].p_value) != u16_decode(p_value))
        {
            continue;
        }
        if ((out_len + 4U) > m_att_mtu)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &m_att_rsp[out_len]);
        u16_encode(att_service_end_handle(m_gatt_db[i].handle), &m_att_rsp[out_len + 2U]);
        out_len = (uint16_t)(out_len + 4U);
    }

    if (out_len == 1U)
    {
        return att_error_rsp(BLE_ATT_OP_FIND_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_read_by_group_type(uint16_t start_handle, uint16_t end_handle, uint16_t group_uuid)
{
    uint16_t out_len = 2U;
    uint16_t i;

    if (group_uuid != BLE_UUID_PRIMARY_SERVICE)
    {
        return att_error_rsp(BLE_ATT_OP_READ_BY_GROUP_REQ, start_handle, BLE_ATT_ERR_UNSUPPORTED_GROUP_TYPE);
    }

    m_att_rsp[0] = BLE_ATT_OP_READ_BY_GROUP_RSP;
    m_att_rsp[1] = 6U;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        if ((m_gatt_db[i].uuid != BLE_UUID_PRIMARY_SERVICE) ||
            (m_gatt_db[i].handle < start_handle) ||
            (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }
        if ((out_len + 6U) > m_att_mtu)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &m_att_rsp[out_len]);
        u16_encode(att_service_end_handle(m_gatt_db[i].handle), &m_att_rsp[out_len + 2U]);
        u16_encode(u16_decode(m_gatt_db[i].p_value), &m_att_rsp[out_len + 4U]);
        out_len = (uint16_t)(out_len + 6U);
    }

    if (out_len == 2U)
    {
        return att_error_rsp(BLE_ATT_OP_READ_BY_GROUP_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_read_by_type(uint16_t start_handle, uint16_t end_handle, uint16_t type_uuid)
{
    uint16_t out_len = 2U;
    uint16_t entry_len = 0U;
    uint16_t i;

    m_att_rsp[0] = BLE_ATT_OP_READ_BY_TYPE_RSP;

    for (i = 0U; i < m_gatt_db_count; i++)
    {
        uint16_t value_len;

        if ((m_gatt_db[i].handle < start_handle) || (m_gatt_db[i].handle > end_handle))
        {
            continue;
        }
        if (m_gatt_db[i].uuid != type_uuid)
        {
            continue;
        }
        if ((m_gatt_db[i].permissions & BLE_ATT_PERM_READ) == 0U)
        {
            continue;
        }

        value_len = *m_gatt_db[i].p_len;
        if (value_len > m_gatt_db[i].max_len)
        {
            value_len = m_gatt_db[i].max_len;
        }

        if (entry_len == 0U)
        {
            entry_len = (uint16_t)(2U + value_len);
            if ((entry_len + 2U) > m_att_mtu)
            {
                return att_error_rsp(BLE_ATT_OP_READ_BY_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
            }
            m_att_rsp[1] = (uint8_t)entry_len;
        }

        if ((uint16_t)(2U + value_len) != entry_len)
        {
            continue;
        }
        if ((out_len + entry_len) > m_att_mtu)
        {
            break;
        }

        u16_encode(m_gatt_db[i].handle, &m_att_rsp[out_len]);
        (void)memcpy(&m_att_rsp[out_len + 2U], m_gatt_db[i].p_value, value_len);
        out_len = (uint16_t)(out_len + entry_len);
    }

    if (out_len == 2U)
    {
        return att_error_rsp(BLE_ATT_OP_READ_BY_TYPE_REQ, start_handle, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }

    return out_len;
}

static uint16_t att_handle_read(uint16_t handle)
{
    gatt_attr_t *p_attr = att_find_attr(handle);
    uint16_t len;

    if (p_attr == NULL)
    {
        return att_error_rsp(BLE_ATT_OP_READ_REQ, handle, BLE_ATT_ERR_INVALID_HANDLE);
    }
    if ((p_attr->permissions & BLE_ATT_PERM_READ) == 0U)
    {
        return att_error_rsp(BLE_ATT_OP_READ_REQ, handle, BLE_ATT_ERR_READ_NOT_PERMITTED);
    }

    len = *p_attr->p_len;
    if (len > p_attr->max_len)
    {
        len = p_attr->max_len;
    }
    if ((uint16_t)(1U + len) > m_att_mtu)
    {
        len = (uint16_t)(m_att_mtu - 1U);
    }

    m_att_rsp[0] = BLE_ATT_OP_READ_RSP;
    (void)memcpy(&m_att_rsp[1], p_attr->p_value, len);
    return (uint16_t)(1U + len);
}

static uint16_t att_handle_write_req(uint16_t handle, const uint8_t *p_data, uint16_t len, bool with_rsp)
{
    gatt_attr_t *p_attr = att_find_attr(handle);
    gatt_char_runtime_t *p_char_runtime;

    if (p_attr == NULL)
    {
        return with_rsp ? att_error_rsp(BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_INVALID_HANDLE) : 0U;
    }
    if ((p_attr->permissions & BLE_ATT_PERM_WRITE) == 0U)
    {
        return with_rsp ? att_error_rsp(BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_WRITE_NOT_PERMITTED) : 0U;
    }

    p_char_runtime = gatt_find_char_runtime_by_cccd_handle(handle);
    if ((p_char_runtime != NULL) && (len != 2U))
    {
        return with_rsp ? att_error_rsp(BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_INVALID_ATTRIBUTE_VALUE_LEN) : 0U;
    }
    if ((p_char_runtime == NULL) && (p_attr->p_characteristic != NULL))
    {
        if (with_rsp && ((p_attr->p_characteristic->properties & BLE_GATT_CHAR_PROP_WRITE) == 0U))
        {
            return att_error_rsp(BLE_ATT_OP_WRITE_REQ, handle, BLE_ATT_ERR_WRITE_NOT_PERMITTED);
        }
        if (!with_rsp &&
            ((p_attr->p_characteristic->properties & BLE_GATT_CHAR_PROP_WRITE_NO_RESP) == 0U))
        {
            return 0U;
        }
    }

    if (len > p_attr->max_len)
    {
        return with_rsp ? att_error_rsp(BLE_ATT_OP_WRITE_REQ,
                                        handle,
                                        BLE_ATT_ERR_INVALID_ATTRIBUTE_VALUE_LEN)
                        : 0U;
    }
    if ((len > 0U) && (p_data != NULL))
    {
        (void)memcpy(p_attr->p_value, p_data, len);
    }
    *p_attr->p_len = len;

    if (p_char_runtime != NULL)
    {
        p_char_runtime->notifications_enabled = ((p_char_runtime->cccd_value[0] & 0x01U) != 0U);
        (void)ble_evt_notify_gatt(p_char_runtime->notifications_enabled ? BLE_GATT_EVT_NOTIFY_ENABLED
                                                                        : BLE_GATT_EVT_NOTIFY_DISABLED,
                                  p_char_runtime->p_characteristic,
                                  p_char_runtime->cccd_value,
                                  len,
                                  p_char_runtime->notifications_enabled);
    }
    else if (p_attr->p_characteristic != NULL)
    {
        (void)ble_evt_notify_gatt(BLE_GATT_EVT_WRITE,
                                  p_attr->p_characteristic,
                                  p_attr->p_value,
                                  len,
                                  false);
    }

    if (!with_rsp)
    {
        return 0U;
    }

    m_att_rsp[0] = BLE_ATT_OP_WRITE_RSP;
    return 1U;
}

static bool gatt_build_fixed_attrs(void)
{
    return gatt_db_add_attr(HANDLE_GAP_SERVICE,
                            BLE_UUID_PRIMARY_SERVICE,
                            BLE_ATT_PERM_READ,
                            m_attr_gap_service,
                            &m_attr_gap_service_len,
                            sizeof(m_attr_gap_service),
                            NULL) &&
           gatt_db_add_attr(HANDLE_GAP_DEVNAME_DECL,
                            BLE_UUID_CHARACTERISTIC,
                            BLE_ATT_PERM_READ,
                            m_attr_gap_devname_decl,
                            &m_attr_gap_devname_decl_len,
                            sizeof(m_attr_gap_devname_decl),
                            NULL) &&
           gatt_db_add_attr(HANDLE_GAP_DEVNAME_VALUE,
                            BLE_UUID_DEVICE_NAME,
                            BLE_ATT_PERM_READ,
                            m_attr_gap_devname_value,
                            &m_attr_gap_devname_value_len,
                            sizeof(m_attr_gap_devname_value),
                            NULL) &&
           gatt_db_add_attr(HANDLE_GAP_APPEAR_DECL,
                            BLE_UUID_CHARACTERISTIC,
                            BLE_ATT_PERM_READ,
                            m_attr_gap_appear_decl,
                            &m_attr_gap_appear_decl_len,
                            sizeof(m_attr_gap_appear_decl),
                            NULL) &&
           gatt_db_add_attr(HANDLE_GAP_APPEAR_VALUE,
                            BLE_UUID_APPEARANCE,
                            BLE_ATT_PERM_READ,
                            m_attr_gap_appear_value,
                            &m_attr_gap_appear_value_len,
                            sizeof(m_attr_gap_appear_value),
                            NULL) &&
           gatt_db_add_attr(HANDLE_GATT_SERVICE,
                            BLE_UUID_PRIMARY_SERVICE,
                            BLE_ATT_PERM_READ,
                            m_attr_gatt_service,
                            &m_attr_gatt_service_len,
                            sizeof(m_attr_gatt_service),
                            NULL);
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
        gatt_service_runtime_t *p_service_runtime;
        uint8_t char_idx;

        if ((p_service->p_characteristics == NULL) || (p_service->characteristic_count == 0U))
        {
            return false;
        }
        if (m_service_runtime_count >= BLE_GATT_MAX_SERVICES)
        {
            return false;
        }

        p_service_runtime = &m_service_runtime[m_service_runtime_count];
        p_service_runtime->p_service = p_service;
        p_service_runtime->value[0] = (uint8_t)(p_service->uuid & 0xFFU);
        p_service_runtime->value[1] = (uint8_t)((p_service->uuid >> 8) & 0xFFU);
        p_service_runtime->value_len = 2U;
        p_service->service_handle = next_handle;

        if (!gatt_db_add_attr(next_handle,
                              BLE_UUID_PRIMARY_SERVICE,
                              BLE_ATT_PERM_READ,
                              p_service_runtime->value,
                              &p_service_runtime->value_len,
                              sizeof(p_service_runtime->value),
                              NULL))
        {
            return false;
        }

        next_handle++;
        m_service_runtime_count++;

        for (char_idx = 0U; char_idx < p_service->characteristic_count; char_idx++)
        {
            ble_gatt_characteristic_t *p_characteristic = &p_service->p_characteristics[char_idx];
            gatt_char_runtime_t *p_char_runtime;

            if (!gatt_characteristic_is_valid(p_characteristic))
            {
                return false;
            }

            if (m_char_runtime_count >= BLE_GATT_MAX_CHARACTERISTICS)
            {
                return false;
            }

            p_char_runtime = &m_char_runtime[m_char_runtime_count];
            (void)memset(p_char_runtime, 0, sizeof(*p_char_runtime));
            p_char_runtime->p_service = p_service;
            p_char_runtime->p_characteristic = p_characteristic;
            p_char_runtime->decl_len = 5U;
            p_char_runtime->cccd_len = 2U;
            p_char_runtime->decl_value[0] = p_characteristic->properties;
            u16_encode((uint16_t)(next_handle + 1U), &p_char_runtime->decl_value[1]);
            u16_encode(p_characteristic->uuid, &p_char_runtime->decl_value[3]);

            p_characteristic->value_handle = (uint16_t)(next_handle + 1U);
            p_characteristic->cccd_handle = 0U;

            if (!gatt_db_add_attr(next_handle,
                                  BLE_UUID_CHARACTERISTIC,
                                  BLE_ATT_PERM_READ,
                                  p_char_runtime->decl_value,
                                  &p_char_runtime->decl_len,
                                  sizeof(p_char_runtime->decl_value),
                                  NULL))
            {
                return false;
            }

            next_handle++;

            if (!gatt_db_add_attr(next_handle,
                                  p_characteristic->uuid,
                                  gatt_props_to_permissions(p_characteristic->properties),
                                  p_characteristic->p_value,
                                  p_characteristic->p_value_len,
                                  p_characteristic->max_len,
                                  p_characteristic))
            {
                return false;
            }

            next_handle++;

            if ((p_characteristic->properties & BLE_GATT_CHAR_PROP_NOTIFY) != 0U)
            {
                p_characteristic->cccd_handle = next_handle;
                if (!gatt_db_add_attr(next_handle,
                                      BLE_UUID_CCCD,
                                      (uint8_t)(BLE_ATT_PERM_READ | BLE_ATT_PERM_WRITE),
                                      p_char_runtime->cccd_value,
                                      &p_char_runtime->cccd_len,
                                      sizeof(p_char_runtime->cccd_value),
                                      p_characteristic))
                {
                    return false;
                }
                next_handle++;
            }

            m_char_runtime_count++;
        }
    }

    return true;
}

bool ble_gatt_server_init(ble_gatt_service_t *p_services,
                          uint8_t service_count)
{
    const char *p_name = m_host.has_adv_name ? m_host.adv_name : "nrf52-ble";
    size_t name_len;

    gatt_reset_runtime_tables();

    name_len = strlen(p_name);
    if (name_len > sizeof(m_attr_gap_devname_value))
    {
        name_len = sizeof(m_attr_gap_devname_value);
    }

    (void)memset(m_attr_gap_devname_value, 0, sizeof(m_attr_gap_devname_value));
    (void)memcpy(m_attr_gap_devname_value, p_name, name_len);
    m_attr_gap_devname_value_len = (uint16_t)name_len;

    m_attr_gap_appear_value[0] = 0x00U;
    m_attr_gap_appear_value[1] = 0x00U;

    gatt_reset_user_handles(p_services, service_count);

    if (!gatt_build_fixed_attrs())
    {
        return false;
    }

    if (!gatt_build_custom_attrs(p_services, service_count))
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

    for (i = 0U; i < m_char_runtime_count; i++)
    {
        m_char_runtime[i].notifications_enabled = false;
        m_char_runtime[i].cccd_value[0] = 0U;
        m_char_runtime[i].cccd_value[1] = 0U;
        m_char_runtime[i].cccd_len = 2U;
    }
}

bool ble_gatt_server_notifications_enabled(const ble_gatt_characteristic_t *p_characteristic)
{
    gatt_char_runtime_t *p_char_runtime = gatt_find_char_runtime_by_characteristic(p_characteristic);

    if (p_char_runtime == NULL)
    {
        return false;
    }

    return p_char_runtime->notifications_enabled;
}

uint16_t ble_gatt_server_build_notification(uint16_t value_handle, uint8_t *p_att, uint16_t max_len)
{
    gatt_char_runtime_t *p_char_runtime = gatt_find_char_runtime_by_value_handle(value_handle);
    uint16_t value_len;

    if ((p_att == NULL) || (p_char_runtime == NULL) || !p_char_runtime->notifications_enabled)
    {
        return 0U;
    }

    value_len = *p_char_runtime->p_characteristic->p_value_len;
    if (value_len > p_char_runtime->p_characteristic->max_len)
    {
        value_len = p_char_runtime->p_characteristic->max_len;
    }
    if ((uint16_t)(3U + value_len) > max_len)
    {
        return 0U;
    }

    p_att[0] = BLE_ATT_OP_NOTIFY;
    u16_encode(value_handle, &p_att[1]);
    (void)memcpy(&p_att[3], p_char_runtime->p_characteristic->p_value, value_len);
    return (uint16_t)(3U + value_len);
}

uint16_t ble_gatt_server_process_request(const uint8_t *p_att,
                                         uint16_t att_len,
                                         uint8_t *p_rsp,
                                         uint16_t rsp_max_len,
                                         bool *p_has_response)
{
    uint8_t opcode;
    uint16_t rsp_len = 0U;

    if ((p_has_response == NULL) || (p_rsp == NULL) || (rsp_max_len == 0U))
    {
        return 0U;
    }

    *p_has_response = true;

    if ((p_att == NULL) || (att_len == 0U))
    {
        return 0U;
    }

    opcode = p_att[0];
    switch (opcode)
    {
    case BLE_ATT_OP_MTU_REQ:
    {
        uint16_t requested_mtu;

        if (att_len < 3U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        requested_mtu = u16_decode(&p_att[1]);
        m_att_mtu = requested_mtu;
        if (m_att_mtu > BLE_ATT_MTU_DEFAULT)
        {
            m_att_mtu = BLE_ATT_MTU_DEFAULT;
        }
        if (m_att_mtu < BLE_ATT_MTU_DEFAULT)
        {
            m_att_mtu = BLE_ATT_MTU_DEFAULT;
        }
        m_att_rsp[0] = BLE_ATT_OP_MTU_RSP;
        u16_encode(BLE_ATT_MTU_DEFAULT, &m_att_rsp[1]);
        (void)ble_evt_notify_mtu_exchange(requested_mtu,
                                          BLE_ATT_MTU_DEFAULT,
                                          m_att_mtu);
        rsp_len = 3U;
        break;
    }

    case BLE_ATT_OP_FIND_INFO_REQ:
        if (att_len < 5U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_find_info(u16_decode(&p_att[1]), u16_decode(&p_att[3]));
        break;

    case BLE_ATT_OP_FIND_TYPE_REQ:
        if (att_len < 7U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_find_by_type_value(u16_decode(&p_att[1]),
                                                u16_decode(&p_att[3]),
                                                u16_decode(&p_att[5]),
                                                &p_att[7],
                                                (uint16_t)(att_len - 7U));
        break;

    case BLE_ATT_OP_READ_BY_GROUP_REQ:
        if (att_len < 7U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_read_by_group_type(u16_decode(&p_att[1]),
                                                u16_decode(&p_att[3]),
                                                u16_decode(&p_att[5]));
        break;

    case BLE_ATT_OP_READ_BY_TYPE_REQ:
        if (att_len < 7U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_read_by_type(u16_decode(&p_att[1]),
                                          u16_decode(&p_att[3]),
                                          u16_decode(&p_att[5]));
        break;

    case BLE_ATT_OP_READ_REQ:
        if (att_len < 3U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_read(u16_decode(&p_att[1]));
        break;

    case BLE_ATT_OP_WRITE_REQ:
        if (att_len < 3U)
        {
            rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
            break;
        }
        rsp_len = att_handle_write_req(u16_decode(&p_att[1]),
                                       &p_att[3],
                                       (uint16_t)(att_len - 3U),
                                       true);
        break;

    case BLE_ATT_OP_WRITE_CMD:
        if (att_len >= 3U)
        {
            (void)att_handle_write_req(u16_decode(&p_att[1]),
                                       &p_att[3],
                                       (uint16_t)(att_len - 3U),
                                       false);
        }
        *p_has_response = false;
        rsp_len = 0U;
        break;

    default:
        rsp_len = att_error_rsp(opcode, 0x0000U, BLE_ATT_ERR_REQUEST_NOT_SUPPORTED);
        break;
    }

    if (rsp_len > rsp_max_len)
    {
        rsp_len = rsp_max_len;
    }
    if (rsp_len > 0U)
    {
        (void)memcpy(p_rsp, m_att_rsp, rsp_len);
    }

    return rsp_len;
}
