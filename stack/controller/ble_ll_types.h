/**
 * @file ble_ll_types.h
 * @author Surya Poudel
 * @brief Shared internal link-layer packet types for nRF BLE stack
 * @version 0.1
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_LL_TYPES_H__
#define BLE_LL_TYPES_H__

#include <stdint.h>

#define BLE_LL_ADV_DATA_MAX_LEN 31U
#define BLE_ADV_ADVERTISER_ADDRESS_LEN 6U
#define BLE_LL_ADV_RX_PAYLOAD_MAX_LEN 128U
#define BLE_LL_DATA_PAYLOAD_MAX_LEN 251U

typedef enum
{
    LL_ADV_IND = 0x00,
    LL_ADV_DIRECT_IND = 0x01,
    LL_ADV_NONCONN_IND = 0x02,
    LL_ADV_SCAN_IND = 0x06,
    LL_SCAN_REQ = 0x03,
    LL_SCAN_RSP = 0x04,
    LL_CONNECT_REQ = 0x05
} ble_adv_pdu_type_t;

typedef enum
{
    BLE_LLID_CONTINUATION = 0x01,
    BLE_LLID_START_L2CAP = 0x02,
    BLE_LLID_CONTROL_PDU = 0x03
} ble_llid_t;

typedef struct
{
    uint8_t pdu_type : 4;
    uint8_t rfu : 2;
    uint8_t txadd : 1;
    uint8_t rxadd : 1;
} __attribute__((packed)) ble_ll_adv_header_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t mac_address[6];
    uint8_t payload[BLE_LL_ADV_RX_PAYLOAD_MAX_LEN];
} __attribute__((packed)) ble_ll_adv_pdu_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t scanner_address[6];
    uint8_t advertiser_address[6];
} __attribute__((packed)) ble_scan_req_pdu_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t advertiser_address[6];
    uint8_t payload[BLE_LL_ADV_DATA_MAX_LEN];
} __attribute__((packed)) ble_scan_rsp_pdu_t;

typedef struct
{
    uint8_t access_address[4];
    uint8_t crc_init[3];
    uint8_t win_size;
    uint16_t win_offset;
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
    uint8_t channel_map[5];
    uint8_t hop_increment : 5;
    uint8_t sca : 3;
} __attribute__((packed)) ble_ll_connect_ind_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t initiator_address[6];
    uint8_t advertiser_address[6];
    ble_ll_connect_ind_t ll_data;
} __attribute__((packed)) ble_connect_req_pdu_t;

typedef union
{
    ble_ll_adv_pdu_t adv;
    ble_scan_req_pdu_t scan_req;
    ble_connect_req_pdu_t connect_req;
} ble_adv_rx_pdu_t;

typedef struct
{
    uint8_t llid : 2;
    uint8_t nesn : 1;
    uint8_t sn : 1;
    uint8_t md : 1;
    uint8_t rfu : 3;
} __attribute__((packed)) ble_ll_data_header_t;

typedef struct
{
    ble_ll_data_header_t header;
    uint8_t length;
    uint8_t payload[BLE_LL_DATA_PAYLOAD_MAX_LEN];
} __attribute__((packed)) ble_ll_data_raw_pdu_t;

#endif /* BLE_LL_TYPES_H__ */
