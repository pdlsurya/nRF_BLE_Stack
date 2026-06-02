/* SPDX-License-Identifier: MIT */

#ifndef BLE_LL_INTERNAL_H__
#define BLE_LL_INTERNAL_H__

#include <stdint.h>

#define BLE_LL_ADV_DATA_MAX_LEN 31U
#define BLE_ADV_ADVERTISER_ADDRESS_LEN 6U
#define BLE_LL_ADV_RX_PAYLOAD_MAX_LEN 128U
#define BLE_LL_DATA_PAYLOAD_MAX_LEN 251U
#define BLE_LL_DATA_HEADER_BITS 16U
#define BLE_SCAN_REQ_PAYLOAD_LEN 12U
#define BLE_LL_DATA_LEN_DEFAULT_OCTETS 27U
#define BLE_LL_DATA_LEN_MAX_OCTETS 251U
#define BLE_LL_DATA_LEN_DEFAULT_TIME 328U
#define BLE_LL_DATA_LEN_MAX_TIME 2120U
#define BLE_LL_CTRL_CONN_UPDATE_WIN_SIZE_UNITS 1U
#define BLE_LL_CTRL_INSTANT_OFFSET_EVENTS 6U

#define BLE_LL_CTRL_CONN_UPDATE_IND 0x00U
#define BLE_LL_CTRL_CHANNEL_MAP_IND 0x01U
#define BLE_LL_CTRL_TERMINATE_IND 0x02U
#define BLE_LL_CTRL_UNKNOWN_RSP 0x07U
#define BLE_LL_CTRL_FEATURE_REQ 0x08U
#define BLE_LL_CTRL_FEATURE_RSP 0x09U
#define BLE_LL_CTRL_VERSION_IND 0x0CU
#define BLE_LL_CTRL_SLV_FEATURE_REQ 0x0EU
#define BLE_LL_CTRL_LENGTH_REQ 0x14U
#define BLE_LL_CTRL_LENGTH_RSP 0x15U
#define BLE_LL_CTRL_PHY_REQ 0x16U
#define BLE_LL_CTRL_PHY_RSP 0x17U
#define BLE_LL_CTRL_PHY_UPDATE_IND 0x18U

#define BLE_LL_VERSION_4_2 0x08U
#define BLE_LL_COMPANY_ID_NORDIC 0x0059U
#define BLE_LL_SUBVERSION 0x0000U
#define BLE_LL_FEATURE_DATA_LENGTH_EXTENSION 0x20U
#define BLE_LL_FEATURE_2M_PHY 0x01U
#define BLE_LL_PHY_1M 0x01U
#define BLE_LL_PHY_2M 0x02U

typedef enum
{
    LL_ADV_IND = 0x00,
    LL_ADV_DIRECT_IND = 0x01,
    LL_ADV_NONCONN_IND = 0x02,
    LL_ADV_SCAN_IND = 0x06,
    LL_SCAN_REQ = 0x03,
    LL_SCAN_RSP = 0x04,
    LL_CONNECT_REQ = 0x05
} ble_ll_adv_pdu_type_t;

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
    uint8_t advertiser_address[6];
    uint8_t payload[BLE_LL_ADV_RX_PAYLOAD_MAX_LEN];
} __attribute__((packed)) ble_ll_adv_pdu_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t scanner_address[6];
    uint8_t advertiser_address[6];
} __attribute__((packed)) ble_ll_scan_req_pdu_t;

typedef struct
{
    ble_ll_adv_header_t header;
    uint8_t payload_length;
    uint8_t advertiser_address[6];
    uint8_t payload[BLE_LL_ADV_DATA_MAX_LEN];
} __attribute__((packed)) ble_ll_scan_rsp_pdu_t;

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
} __attribute__((packed)) ble_ll_connect_req_pdu_t;

typedef union
{
    ble_ll_scan_req_pdu_t scan_req;
    ble_ll_connect_req_pdu_t connect_req;
} ble_ll_adv_req_pdu_t;

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

#endif /* BLE_LL_INTERNAL_H__ */
