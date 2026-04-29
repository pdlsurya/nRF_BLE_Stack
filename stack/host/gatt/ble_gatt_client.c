/**
 * @file ble_gatt_client.c
 * @author Surya Poudel
 * @brief ATT and GATT client implementation for nRF BLE stack
 * @version 0.1
 * @date 2026-04-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ble_runtime_internal.h"
#include "ble_att_defs.h"

#include <string.h>

#define BLE_UUID_PRIMARY_SERVICE 0x2800U
#define BLE_UUID_CHARACTERISTIC 0x2803U

typedef struct
{
    ble_gatt_client_procedure_t procedure;
    uint16_t mtu;
    uint16_t requested_mtu;
    uint16_t range_start;
    uint16_t range_end;
    uint16_t pending_handle;
    ble_uuid_t service_uuid;
} ble_gatt_client_state_t;

static ble_gatt_client_state_t m_client;

static uint16_t gatt_client_clamp_mtu(uint16_t mtu)
{
    if (mtu < BLE_ATT_MTU_DEFAULT)
    {
        return BLE_ATT_MTU_DEFAULT;
    }

    return (mtu > BLE_ATT_MAX_MTU) ? BLE_ATT_MAX_MTU : mtu;
}

static void gatt_client_uuid_set_from_bytes(ble_uuid_t *p_uuid, const uint8_t *p_src, uint8_t len)
{
    if (p_uuid == NULL)
    {
        return;
    }

    (void)memset(p_uuid, 0, sizeof(*p_uuid));
    if ((p_src == NULL) || ((len != BLE_UUID16_LEN) && (len != BLE_UUID128_LEN)))
    {
        return;
    }

    if (len == BLE_UUID16_LEN)
    {
        p_uuid->type = BLE_UUID_TYPE_SIG_16;
        p_uuid->value.uuid16 = u16_decode(p_src);
        return;
    }

    p_uuid->type = BLE_UUID_TYPE_RAW_128;
    (void)memcpy(p_uuid->value.uuid128, p_src, BLE_UUID128_LEN);
}

static bool gatt_client_uuid_is_valid(const ble_uuid_t *p_uuid)
{
    if (p_uuid == NULL)
    {
        return false;
    }

    return ble_uuid_encoded_len(p_uuid) != 0U;
}

static uint16_t gatt_client_error_rsp(uint8_t *p_rsp,
                                      uint16_t rsp_max_len,
                                      uint8_t request_opcode,
                                      uint16_t handle,
                                      uint8_t error_code)
{
    if ((p_rsp == NULL) || (rsp_max_len < 5U))
    {
        return 0U;
    }

    p_rsp[0] = BLE_ATT_OP_ERROR_RESPONSE;
    p_rsp[1] = request_opcode;
    u16_encode(handle, &p_rsp[2]);
    p_rsp[4] = error_code;
    return 5U;
}

static void gatt_client_emit_complete(ble_gatt_client_procedure_t procedure)
{
    ble_gatt_client_evt_t evt;

    if (procedure == BLE_GATT_CLIENT_PROC_NONE)
    {
        return;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.evt_type = BLE_GATT_CLIENT_EVT_PROCEDURE_COMPLETE;
    evt.params.complete.procedure = procedure;
    (void)ble_evt_notify_gatt_client(&evt);
}

static void gatt_client_emit_error(ble_gatt_client_procedure_t procedure,
                                   uint8_t request_opcode,
                                   uint16_t handle,
                                   uint8_t error_code)
{
    ble_gatt_client_evt_t evt;

    (void)memset(&evt, 0, sizeof(evt));
    evt.evt_type = BLE_GATT_CLIENT_EVT_ERROR_RSP;
    evt.params.error.procedure = procedure;
    evt.params.error.request_opcode = request_opcode;
    evt.params.error.handle = handle;
    evt.params.error.error_code = error_code;
    (void)ble_evt_notify_gatt_client(&evt);
}

static void gatt_client_clear_procedure(void)
{
    m_client.procedure = BLE_GATT_CLIENT_PROC_NONE;
    m_client.requested_mtu = 0U;
    m_client.range_start = 0U;
    m_client.range_end = 0U;
    m_client.pending_handle = 0U;
    m_client.service_uuid = (ble_uuid_t)BLE_UUID_NONE_INIT;
}

static bool gatt_client_queue_att(const uint8_t *p_att,
                                  uint16_t att_len)
{
    return controller_queue_l2cap_payload(BLE_L2CAP_CID_ATT, p_att, att_len);
}

static bool gatt_client_queue_discover_primary_services(uint16_t start_handle)
{
    uint8_t req[7];

    req[0] = BLE_ATT_OP_READ_BY_GROUP_TYPE_REQUEST;
    u16_encode(start_handle, &req[1]);
    u16_encode(m_client.range_end, &req[3]);
    u16_encode(BLE_UUID_PRIMARY_SERVICE, &req[5]);
    return gatt_client_queue_att(req, sizeof(req));
}

static bool gatt_client_queue_discover_primary_services_by_uuid(uint16_t start_handle)
{
    uint8_t req[7U + BLE_UUID128_LEN];
    uint8_t uuid_bytes[BLE_UUID128_LEN];
    uint16_t uuid_len;
    uint16_t pdu_len;

    uuid_len = ble_uuid_encoded_len(&m_client.service_uuid);
    if ((uuid_len == 0U) || !ble_uuid_encode(&m_client.service_uuid, uuid_bytes))
    {
        return false;
    }

    req[0] = BLE_ATT_OP_FIND_BY_TYPE_VALUE_REQUEST;
    u16_encode(start_handle, &req[1]);
    u16_encode(m_client.range_end, &req[3]);
    u16_encode(BLE_UUID_PRIMARY_SERVICE, &req[5]);
    (void)memcpy(&req[7], uuid_bytes, uuid_len);
    pdu_len = (uint16_t)(7U + uuid_len);
    return gatt_client_queue_att(req, pdu_len);
}

static bool gatt_client_queue_discover_characteristics(uint16_t start_handle)
{
    uint8_t req[7];

    req[0] = BLE_ATT_OP_READ_BY_TYPE_REQUEST;
    u16_encode(start_handle, &req[1]);
    u16_encode(m_client.range_end, &req[3]);
    u16_encode(BLE_UUID_CHARACTERISTIC, &req[5]);
    return gatt_client_queue_att(req, sizeof(req));
}

static bool gatt_client_queue_discover_descriptors(uint16_t start_handle)
{
    uint8_t req[5];

    req[0] = BLE_ATT_OP_FIND_INFORMATION_REQUEST;
    u16_encode(start_handle, &req[1]);
    u16_encode(m_client.range_end, &req[3]);
    return gatt_client_queue_att(req, sizeof(req));
}

static void gatt_client_handle_service_rsp(const uint8_t *p_att,
                                           uint16_t att_len)
{
    uint8_t item_len;
    uint16_t offset;
    uint16_t last_end_handle = 0U;

    if ((att_len < 2U) || (p_att == NULL))
    {
        return;
    }

    item_len = p_att[1];
    if ((item_len != 6U) && (item_len != 20U))
    {
        gatt_client_emit_error(m_client.procedure,
                               BLE_ATT_OP_READ_BY_GROUP_TYPE_REQUEST,
                               m_client.range_start,
                               BLE_ATT_ERR_INVALID_PDU);
        gatt_client_clear_procedure();
        return;
    }

    for (offset = 2U; (uint16_t)(offset + item_len) <= att_len; offset = (uint16_t)(offset + item_len))
    {
        ble_gatt_client_evt_t evt;

        (void)memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_GATT_CLIENT_EVT_SERVICE_DISCOVERED;
        evt.params.service.start_handle = u16_decode(&p_att[offset]);
        evt.params.service.end_handle = u16_decode(&p_att[offset + 2U]);
        gatt_client_uuid_set_from_bytes(&evt.params.service.uuid,
                                        &p_att[offset + 4U],
                                        (uint8_t)(item_len - 4U));
        (void)ble_evt_notify_gatt_client(&evt);
        last_end_handle = evt.params.service.end_handle;
    }

    if ((last_end_handle == 0U) || (last_end_handle >= m_client.range_end))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
        return;
    }

    m_client.range_start = (uint16_t)(last_end_handle + 1U);
    if (!gatt_client_queue_discover_primary_services(m_client.range_start))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
    }
}

static void gatt_client_handle_find_type_service_rsp(const uint8_t *p_att,
                                                     uint16_t att_len)
{
    uint16_t offset;
    uint16_t last_end_handle = 0U;

    if ((p_att == NULL) || (att_len < 5U) || (((att_len - 1U) % 4U) != 0U))
    {
        gatt_client_emit_error(m_client.procedure,
                               BLE_ATT_OP_FIND_BY_TYPE_VALUE_REQUEST,
                               m_client.range_start,
                               BLE_ATT_ERR_INVALID_PDU);
        gatt_client_clear_procedure();
        return;
    }

    for (offset = 1U; (uint16_t)(offset + 4U) <= att_len; offset = (uint16_t)(offset + 4U))
    {
        ble_gatt_client_evt_t evt;

        (void)memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_GATT_CLIENT_EVT_SERVICE_DISCOVERED;
        evt.params.service.start_handle = u16_decode(&p_att[offset]);
        evt.params.service.end_handle = u16_decode(&p_att[offset + 2U]);
        evt.params.service.uuid = m_client.service_uuid;
        (void)ble_evt_notify_gatt_client(&evt);
        last_end_handle = evt.params.service.end_handle;
    }

    if ((last_end_handle == 0U) || (last_end_handle >= m_client.range_end))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
        return;
    }

    m_client.range_start = (uint16_t)(last_end_handle + 1U);
    if (!gatt_client_queue_discover_primary_services_by_uuid(m_client.range_start))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
    }
}

static void gatt_client_handle_characteristic_rsp(const uint8_t *p_att,
                                                  uint16_t att_len)
{
    uint8_t item_len;
    uint16_t offset;
    uint16_t last_decl_handle = 0U;

    if ((att_len < 2U) || (p_att == NULL))
    {
        return;
    }

    item_len = p_att[1];
    if ((item_len != 7U) && (item_len != 21U))
    {
        gatt_client_emit_error(m_client.procedure,
                               BLE_ATT_OP_READ_BY_TYPE_REQUEST,
                               m_client.range_start,
                               BLE_ATT_ERR_INVALID_PDU);
        gatt_client_clear_procedure();
        return;
    }

    for (offset = 2U; (uint16_t)(offset + item_len) <= att_len; offset = (uint16_t)(offset + item_len))
    {
        ble_gatt_client_evt_t evt;

        (void)memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_GATT_CLIENT_EVT_CHARACTERISTIC_DISCOVERED;
        evt.params.characteristic.declaration_handle = u16_decode(&p_att[offset]);
        evt.params.characteristic.properties = p_att[offset + 2U];
        evt.params.characteristic.value_handle = u16_decode(&p_att[offset + 3U]);
        gatt_client_uuid_set_from_bytes(&evt.params.characteristic.uuid,
                                        &p_att[offset + 5U],
                                        (uint8_t)(item_len - 5U));
        (void)ble_evt_notify_gatt_client(&evt);
        last_decl_handle = evt.params.characteristic.declaration_handle;
    }

    if ((last_decl_handle == 0U) || (last_decl_handle >= m_client.range_end))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
        return;
    }

    m_client.range_start = (uint16_t)(last_decl_handle + 1U);
    if (!gatt_client_queue_discover_characteristics(m_client.range_start))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
    }
}

static void gatt_client_handle_descriptor_rsp(const uint8_t *p_att,
                                              uint16_t att_len)
{
    uint8_t format;
    uint8_t item_len;
    uint16_t offset;
    uint16_t last_handle = 0U;

    if ((att_len < 2U) || (p_att == NULL))
    {
        return;
    }

    format = p_att[1];
    if (format == 0x01U)
    {
        item_len = 4U;
    }
    else if (format == 0x02U)
    {
        item_len = 18U;
    }
    else
    {
        gatt_client_emit_error(m_client.procedure,
                               BLE_ATT_OP_FIND_INFORMATION_REQUEST,
                               m_client.range_start,
                               BLE_ATT_ERR_INVALID_PDU);
        gatt_client_clear_procedure();
        return;
    }

    for (offset = 2U; (uint16_t)(offset + item_len) <= att_len; offset = (uint16_t)(offset + item_len))
    {
        ble_gatt_client_evt_t evt;

        (void)memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_GATT_CLIENT_EVT_DESCRIPTOR_DISCOVERED;
        evt.params.descriptor.handle = u16_decode(&p_att[offset]);
        gatt_client_uuid_set_from_bytes(&evt.params.descriptor.uuid,
                                        &p_att[offset + 2U],
                                        (format == 0x01U) ? BLE_UUID16_LEN : BLE_UUID128_LEN);
        (void)ble_evt_notify_gatt_client(&evt);
        last_handle = evt.params.descriptor.handle;
    }

    if ((last_handle == 0U) || (last_handle >= m_client.range_end))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
        return;
    }

    m_client.range_start = (uint16_t)(last_handle + 1U);
    if (!gatt_client_queue_discover_descriptors(m_client.range_start))
    {
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
    }
}

static void gatt_client_handle_read_rsp(const uint8_t *p_att,
                                        uint16_t att_len)
{
    ble_gatt_client_evt_t evt;
    uint16_t value_len;

    if ((p_att == NULL) || (att_len < 1U))
    {
        return;
    }

    value_len = (uint16_t)(att_len - 1U);
    if (value_len > BLE_ATT_MAX_VALUE_LEN)
    {
        value_len = BLE_ATT_MAX_VALUE_LEN;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.evt_type = BLE_GATT_CLIENT_EVT_READ_RSP;
    evt.params.read.handle = m_client.pending_handle;
    evt.params.read.len = value_len;
    if (value_len > 0U)
    {
        (void)memcpy(evt.params.read.data, &p_att[1], value_len);
    }
    (void)ble_evt_notify_gatt_client(&evt);
    gatt_client_emit_complete(m_client.procedure);
    gatt_client_clear_procedure();
}

static void gatt_client_handle_write_rsp(void)
{
    ble_gatt_client_evt_t evt;
    ble_gatt_client_procedure_t procedure = m_client.procedure;

    (void)memset(&evt, 0, sizeof(evt));
    evt.evt_type = BLE_GATT_CLIENT_EVT_WRITE_RSP;
    evt.params.write.handle = m_client.pending_handle;
    (void)ble_evt_notify_gatt_client(&evt);
    gatt_client_emit_complete(procedure);
    gatt_client_clear_procedure();
}

static void gatt_client_handle_hvx(uint8_t evt_type,
                                   const uint8_t *p_att,
                                   uint16_t att_len)
{
    ble_gatt_client_evt_t evt;
    uint16_t value_len;

    if ((p_att == NULL) || (att_len < 3U))
    {
        return;
    }

    value_len = (uint16_t)(att_len - 3U);
    if (value_len > BLE_ATT_MAX_VALUE_LEN)
    {
        value_len = BLE_ATT_MAX_VALUE_LEN;
    }

    (void)memset(&evt, 0, sizeof(evt));
    evt.evt_type = evt_type;
    evt.params.hvx.handle = u16_decode(&p_att[1]);
    evt.params.hvx.len = value_len;
    if (value_len > 0U)
    {
        (void)memcpy(evt.params.hvx.data, &p_att[3], value_len);
    }
    (void)ble_evt_notify_gatt_client(&evt);
}

static void gatt_client_handle_error_rsp(const uint8_t *p_att,
                                         uint16_t att_len)
{
    uint8_t request_opcode;
    uint16_t handle;
    uint8_t error_code;
    ble_gatt_client_procedure_t procedure = m_client.procedure;

    if ((p_att == NULL) || (att_len < 5U))
    {
        return;
    }

    request_opcode = p_att[1];
    handle = u16_decode(&p_att[2]);
    error_code = p_att[4];

    if (((procedure == BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES) && (request_opcode == BLE_ATT_OP_READ_BY_GROUP_TYPE_REQUEST) &&
         (error_code == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)) ||
        ((procedure == BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES_BY_UUID) && (request_opcode == BLE_ATT_OP_FIND_BY_TYPE_VALUE_REQUEST) &&
         (error_code == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)) ||
        ((procedure == BLE_GATT_CLIENT_PROC_DISCOVER_CHARACTERISTICS) && (request_opcode == BLE_ATT_OP_READ_BY_TYPE_REQUEST) &&
         (error_code == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)) ||
        ((procedure == BLE_GATT_CLIENT_PROC_DISCOVER_DESCRIPTORS) && (request_opcode == BLE_ATT_OP_FIND_INFORMATION_REQUEST) &&
         (error_code == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)))
    {
        gatt_client_emit_complete(procedure);
        gatt_client_clear_procedure();
        return;
    }

    gatt_client_emit_error(procedure, request_opcode, handle, error_code);
    gatt_client_clear_procedure();
}

void ble_gatt_client_init(void)
{
    (void)memset(&m_client, 0, sizeof(m_client));
    m_client.mtu = BLE_ATT_MTU_DEFAULT;
}

void ble_gatt_client_reset_connection_state(void)
{
    ble_gatt_client_init();
}

void ble_gatt_client_register_evt_handler(ble_gatt_client_evt_handler_t handler)
{
    m_gatt_client_evt_handler = handler;
}

bool ble_gatt_client_is_busy(void)
{
    return m_client.procedure != BLE_GATT_CLIENT_PROC_NONE;
}

bool ble_gatt_client_exchange_mtu(uint16_t requested_mtu)
{
    uint8_t req[3];

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) || !m_link.connected || ble_gatt_client_is_busy())
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_MTU_EXCHANGE;
    m_client.requested_mtu = gatt_client_clamp_mtu(requested_mtu);
    req[0] = BLE_ATT_OP_EXCHANGE_MTU_REQUEST;
    u16_encode(m_client.requested_mtu, &req[1]);

    if (!gatt_client_queue_att(req, sizeof(req)))
    {
        gatt_client_clear_procedure();
        return false;
    }

    return true;
}

bool ble_gatt_client_discover_primary_services(void)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) || !m_link.connected || ble_gatt_client_is_busy())
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES;
    m_client.range_start = 0x0001U;
    m_client.range_end = 0xFFFFU;

    if (!gatt_client_queue_discover_primary_services(m_client.range_start))
    {
        gatt_client_clear_procedure();
        return false;
    }

    return true;
}

bool ble_gatt_client_discover_primary_services_by_uuid(const ble_uuid_t *p_uuid)
{
    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        !m_link.connected ||
        ble_gatt_client_is_busy() ||
        !gatt_client_uuid_is_valid(p_uuid))
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES_BY_UUID;
    m_client.range_start = 0x0001U;
    m_client.range_end = 0xFFFFU;
    m_client.service_uuid = *p_uuid;

    if (!gatt_client_queue_discover_primary_services_by_uuid(m_client.range_start))
    {
        gatt_client_clear_procedure();
        return false;
    }

    return true;
}

bool ble_gatt_client_discover_characteristics(uint16_t start_handle, uint16_t end_handle)
{
    if (!m_link.connected ||
        !ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        ble_gatt_client_is_busy() ||
        (start_handle == 0U) ||
        (start_handle > end_handle))
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_DISCOVER_CHARACTERISTICS;
    m_client.range_start = start_handle;
    m_client.range_end = end_handle;

    if (!gatt_client_queue_discover_characteristics(m_client.range_start))
    {
        gatt_client_clear_procedure();
        return false;
    }

    return true;
}

bool ble_gatt_client_discover_descriptors(uint16_t start_handle, uint16_t end_handle)
{
    if (!m_link.connected ||
        !ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        ble_gatt_client_is_busy() ||
        (start_handle == 0U) ||
        (start_handle > end_handle))
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_DISCOVER_DESCRIPTORS;
    m_client.range_start = start_handle;
    m_client.range_end = end_handle;

    if (!gatt_client_queue_discover_descriptors(m_client.range_start))
    {
        gatt_client_clear_procedure();
        return false;
    }

    return true;
}

bool ble_gatt_client_read(uint16_t handle)
{
    uint8_t req[3];

    if (!ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        !m_link.connected ||
        ble_gatt_client_is_busy() ||
        (handle == 0U))
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_READ;
    m_client.pending_handle = handle;
    req[0] = BLE_ATT_OP_READ_REQUEST;
    u16_encode(handle, &req[1]);

    if (!gatt_client_queue_att(req, sizeof(req)))
    {
        gatt_client_clear_procedure();
        return false;
    }

    return true;
}

bool ble_gatt_client_write(uint16_t handle,
                           const uint8_t *p_data,
                           uint16_t len,
                           bool with_response)
{
    uint8_t req[BLE_ATT_MAX_MTU];
    uint16_t pdu_len;

    if (!m_link.connected ||
        !ble_host_role_is_configured(BLE_GAP_ROLE_CENTRAL) ||
        ((len > 0U) && (p_data == NULL)) ||
        (handle == 0U) ||
        ((uint16_t)(len + 3U) > sizeof(req)) ||
        (with_response && ble_gatt_client_is_busy()))
    {
        return false;
    }

    req[0] = with_response ? BLE_ATT_OP_WRITE_REQUEST : BLE_ATT_OP_WRITE_COMMAND;
    u16_encode(handle, &req[1]);
    if (len > 0U)
    {
        (void)memcpy(&req[3], p_data, len);
    }
    pdu_len = (uint16_t)(len + 3U);

    if (with_response)
    {
        m_client.procedure = BLE_GATT_CLIENT_PROC_WRITE;
        m_client.pending_handle = handle;
    }

    if (!gatt_client_queue_att(req, pdu_len))
    {
        if (with_response)
        {
            gatt_client_clear_procedure();
        }
        return false;
    }

    return true;
}

bool ble_gatt_client_write_cccd(uint16_t cccd_handle, bool enable_notifications, bool enable_indications)
{
    uint8_t cccd[2];

    cccd[0] = 0U;
    cccd[1] = 0U;
    if (enable_notifications)
    {
        cccd[0] |= 0x01U;
    }
    if (enable_indications)
    {
        cccd[0] |= 0x02U;
    }

    if (!ble_gatt_client_write(cccd_handle, cccd, sizeof(cccd), true))
    {
        return false;
    }

    m_client.procedure = BLE_GATT_CLIENT_PROC_WRITE_CCCD;
    return true;
}

uint16_t ble_gatt_client_process_pdu(const uint8_t *p_att,
                                     uint16_t att_len,
                                     uint8_t *p_rsp,
                                     uint16_t rsp_max_len)
{
    uint8_t opcode;

    if ((p_att == NULL) || (att_len == 0U))
    {
        return 0U;
    }

    opcode = p_att[0];
    switch (opcode)
    {
    case BLE_ATT_OP_EXCHANGE_MTU_REQUEST:
    {
        uint16_t requested_mtu;
        uint16_t local_mtu;
        uint16_t negotiated_mtu;
        ble_gatt_client_evt_t evt;

        if (att_len < 3U)
        {
            return gatt_client_error_rsp(p_rsp, rsp_max_len, opcode, 0x0000U, BLE_ATT_ERR_INVALID_PDU);
        }

        requested_mtu = u16_decode(&p_att[1]);
        local_mtu = BLE_ATT_MAX_MTU;
        negotiated_mtu = (requested_mtu < local_mtu) ? requested_mtu : local_mtu;
        m_client.mtu = negotiated_mtu;

        if ((p_rsp == NULL) || (rsp_max_len < 3U))
        {
            return 0U;
        }

        p_rsp[0] = BLE_ATT_OP_EXCHANGE_MTU_RESPONSE;
        u16_encode(local_mtu, &p_rsp[1]);

        (void)memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_GATT_CLIENT_EVT_MTU_EXCHANGED;
        evt.params.mtu.requested_mtu = requested_mtu;
        evt.params.mtu.negotiated_mtu = negotiated_mtu;
        (void)ble_evt_notify_gatt_client(&evt);
        return 3U;
    }

    case BLE_ATT_OP_EXCHANGE_MTU_RESPONSE:
    {
        uint16_t response_mtu;
        uint16_t negotiated_mtu;
        uint16_t local_mtu;
        ble_gatt_client_evt_t evt;

        if ((m_client.procedure != BLE_GATT_CLIENT_PROC_MTU_EXCHANGE) || (att_len < 3U))
        {
            return 0U;
        }

        response_mtu = u16_decode(&p_att[1]);
        local_mtu = gatt_client_clamp_mtu(m_client.requested_mtu);
        negotiated_mtu = (response_mtu < local_mtu) ? response_mtu : local_mtu;
        m_client.mtu = negotiated_mtu;

        (void)memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_GATT_CLIENT_EVT_MTU_EXCHANGED;
        evt.params.mtu.requested_mtu = m_client.requested_mtu;
        evt.params.mtu.negotiated_mtu = negotiated_mtu;
        (void)ble_evt_notify_gatt_client(&evt);
        gatt_client_emit_complete(m_client.procedure);
        gatt_client_clear_procedure();
        return 0U;
    }

    case BLE_ATT_OP_FIND_INFORMATION_RESPONSE:
        if (m_client.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_DESCRIPTORS)
        {
            gatt_client_handle_descriptor_rsp(p_att, att_len);
        }
        return 0U;

    case BLE_ATT_OP_FIND_BY_TYPE_VALUE_RESPONSE:
        if (m_client.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES_BY_UUID)
        {
            gatt_client_handle_find_type_service_rsp(p_att, att_len);
        }
        return 0U;

    case BLE_ATT_OP_READ_BY_TYPE_RESPONSE:
        if (m_client.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_CHARACTERISTICS)
        {
            gatt_client_handle_characteristic_rsp(p_att, att_len);
        }
        return 0U;

    case BLE_ATT_OP_READ_RESPONSE:
        if (m_client.procedure == BLE_GATT_CLIENT_PROC_READ)
        {
            gatt_client_handle_read_rsp(p_att, att_len);
        }
        return 0U;

    case BLE_ATT_OP_READ_BY_GROUP_TYPE_RESPONSE:
        if (m_client.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES)
        {
            gatt_client_handle_service_rsp(p_att, att_len);
        }
        return 0U;

    case BLE_ATT_OP_WRITE_RESPONSE:
        if ((m_client.procedure == BLE_GATT_CLIENT_PROC_WRITE) ||
            (m_client.procedure == BLE_GATT_CLIENT_PROC_WRITE_CCCD))
        {
            gatt_client_handle_write_rsp();
        }
        return 0U;

    case BLE_ATT_OP_HANDLE_VALUE_NOTIFICATION:
        gatt_client_handle_hvx(BLE_GATT_CLIENT_EVT_NOTIFICATION, p_att, att_len);
        return 0U;

    case BLE_ATT_OP_HANDLE_VALUE_INDICATION:
        gatt_client_handle_hvx(BLE_GATT_CLIENT_EVT_INDICATION, p_att, att_len);
        if ((p_rsp == NULL) || (rsp_max_len < 1U))
        {
            return 0U;
        }
        p_rsp[0] = BLE_ATT_OP_HANDLE_VALUE_CONFIRMATION;
        return 1U;

    case BLE_ATT_OP_ERROR_RESPONSE:
        gatt_client_handle_error_rsp(p_att, att_len);
        return 0U;

    case BLE_ATT_OP_HANDLE_VALUE_CONFIRMATION:
    case BLE_ATT_OP_WRITE_COMMAND:
        return 0U;

    default:
        return gatt_client_error_rsp(p_rsp,
                                     rsp_max_len,
                                     opcode,
                                     (att_len >= 3U) ? u16_decode(&p_att[1]) : 0x0000U,
                                     (opcode == BLE_ATT_OP_READ_REQUEST) ? BLE_ATT_ERR_INVALID_HANDLE
                                                                     : BLE_ATT_ERR_REQUEST_NOT_SUPPORTED);
    }
}
