/**
 * @file ble_gatt_client.h
 * @author Surya Poudel
 * @brief ATT and GATT client interface for nRF BLE stack
 * @version 0.1
 * @date 2026-04-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_GATT_CLIENT_H__
#define BLE_GATT_CLIENT_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_att_defs.h"
#include "ble_uuid.h"

typedef enum
{
    BLE_GATT_CLIENT_PROC_NONE = 0,
    BLE_GATT_CLIENT_PROC_MTU_EXCHANGE,
    BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES,
    BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES_BY_UUID,
    BLE_GATT_CLIENT_PROC_DISCOVER_CHARACTERISTICS,
    BLE_GATT_CLIENT_PROC_DISCOVER_DESCRIPTORS,
    BLE_GATT_CLIENT_PROC_READ,
    BLE_GATT_CLIENT_PROC_WRITE,
    BLE_GATT_CLIENT_PROC_WRITE_CMD,
    BLE_GATT_CLIENT_PROC_WRITE_CCCD
} ble_gatt_client_procedure_t;

typedef enum
{
    BLE_GATT_CLIENT_EVT_MTU_EXCHANGED = 0,
    BLE_GATT_CLIENT_EVT_SERVICE_DISCOVERED,
    BLE_GATT_CLIENT_EVT_CHARACTERISTIC_DISCOVERED,
    BLE_GATT_CLIENT_EVT_DESCRIPTOR_DISCOVERED,
    BLE_GATT_CLIENT_EVT_READ_RSP,
    BLE_GATT_CLIENT_EVT_WRITE_RSP,
    BLE_GATT_CLIENT_EVT_NOTIFICATION,
    BLE_GATT_CLIENT_EVT_INDICATION,
    BLE_GATT_CLIENT_EVT_ERROR_RSP,
    BLE_GATT_CLIENT_EVT_PROCEDURE_COMPLETE
} ble_gatt_client_evt_type_t;

typedef struct
{
    ble_gatt_client_evt_type_t evt_type;
    union
    {
        struct
        {
            uint16_t requested_mtu;
            uint16_t negotiated_mtu;
        } mtu;
        struct
        {
            uint16_t start_handle;
            uint16_t end_handle;
            ble_uuid_t uuid;
        } service;
        struct
        {
            uint16_t declaration_handle;
            uint16_t value_handle;
            uint8_t properties;
            ble_uuid_t uuid;
        } characteristic;
        struct
        {
            uint16_t handle;
            ble_uuid_t uuid;
        } descriptor;
        struct
        {
            uint16_t handle;
            uint16_t len;
            uint8_t data[BLE_ATT_MAX_VALUE_LEN];
        } read;
        struct
        {
            uint16_t handle;
        } write;
        struct
        {
            uint16_t handle;
            uint16_t len;
            uint8_t data[BLE_ATT_MAX_VALUE_LEN];
        } hvx;
        struct
        {
            ble_gatt_client_procedure_t procedure;
            uint8_t request_opcode;
            uint16_t handle;
            uint8_t error_code;
        } error;
        struct
        {
            ble_gatt_client_procedure_t procedure;
        } complete;
    } params;
} ble_gatt_client_evt_t;

typedef void (*ble_gatt_client_evt_handler_t)(const ble_gatt_client_evt_t *p_evt);

void ble_gatt_client_register_evt_handler(ble_gatt_client_evt_handler_t handler);
bool ble_gatt_client_is_busy(void);
bool ble_gatt_client_exchange_mtu(uint16_t requested_mtu);
bool ble_gatt_client_discover_primary_services(void);
bool ble_gatt_client_discover_primary_services_by_uuid(const ble_uuid_t *p_uuid);
bool ble_gatt_client_discover_characteristics(uint16_t start_handle, uint16_t end_handle);
bool ble_gatt_client_discover_descriptors(uint16_t start_handle, uint16_t end_handle);
bool ble_gatt_client_read(uint16_t handle);
bool ble_gatt_client_write(uint16_t handle, const uint8_t *p_data, uint16_t len, bool with_response);
bool ble_gatt_client_write_cccd(uint16_t cccd_handle, bool enable_notifications, bool enable_indications);

#endif /* BLE_GATT_CLIENT_H__ */
