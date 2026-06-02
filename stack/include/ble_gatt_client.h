/* SPDX-License-Identifier: MIT */

#ifndef BLE_GATT_CLIENT_H__
#define BLE_GATT_CLIENT_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble_att.h"
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

/**
 * @brief Register the stack-wide GATT client event callback.
 *
 * @param[in] handler Application callback for client events.
 */
void ble_gatt_client_register_evt_handler(ble_gatt_client_evt_handler_t handler);

/**
 * @brief Return whether a GATT client procedure is already in progress.
 *
 * @retval true A procedure is active.
 * @retval false No procedure is active.
 */
bool ble_gatt_client_is_busy(void);

/**
 * @brief Start an ATT MTU exchange as the GATT client.
 *
 * @param[in] requested_mtu MTU to request from the peer.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_exchange_mtu(uint16_t requested_mtu);

/**
 * @brief Discover all primary services on the connected peer.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_discover_primary_services(void);

/**
 * @brief Discover primary services that match the supplied UUID.
 *
 * @param[in] p_uuid UUID used to filter primary services.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_discover_primary_services_by_uuid(const ble_uuid_t *p_uuid);

/**
 * @brief Discover characteristic declarations within the supplied handle range.
 *
 * @param[in] start_handle First handle in the discovery range.
 * @param[in] end_handle Last handle in the discovery range.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_discover_characteristics(uint16_t start_handle, uint16_t end_handle);

/**
 * @brief Discover descriptors within the supplied handle range.
 *
 * @param[in] start_handle First handle in the discovery range.
 * @param[in] end_handle Last handle in the discovery range.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_discover_descriptors(uint16_t start_handle, uint16_t end_handle);

/**
 * @brief Read the value at the supplied ATT handle.
 *
 * @param[in] handle ATT handle to read.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_read(uint16_t handle);

/**
 * @brief Write a characteristic or descriptor value, with or without response.
 *
 * @param[in] handle ATT handle to write.
 * @param[in] p_data Pointer to the value bytes to send.
 * @param[in] len Number of bytes in @p p_data.
 * @param[in] with_response True to request a write response, false for write command.
 *
 * @retval true The procedure was started or queued.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_write(uint16_t handle, const uint8_t *p_data, uint16_t len, bool with_response);

/**
 * @brief Write the CCCD value for notification and indication subscription control.
 *
 * @param[in] cccd_handle Handle of the CCCD descriptor.
 * @param[in] enable_notifications True to enable notifications.
 * @param[in] enable_indications True to enable indications.
 *
 * @retval true The procedure was started.
 * @retval false The procedure could not be started.
 */
bool ble_gatt_client_write_cccd(uint16_t cccd_handle, bool enable_notifications, bool enable_indications);

#endif /* BLE_GATT_CLIENT_H__ */
