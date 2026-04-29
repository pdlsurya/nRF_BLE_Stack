/**
 * @file ble_controller_shared.h
 * @author Surya Poudel
 * @brief Shared internal definitions for split BLE controller implementation
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BLE_CONTROLLER_SHARED_H__
#define BLE_CONTROLLER_SHARED_H__

#include "ble_runtime_internal.h"

#include "app_error.h"
#include "nrf_delay.h"

#define BLE_LL_DATA_HEADER_BITS 16U
#define BLE_SCAN_REQ_PAYLOAD_LEN 12U
#define BLE_PRIMARY_ADV_PDU_TYPE LL_ADV_IND
#define BLE_INITIATOR_CONN_INTERVAL_UNITS_DEFAULT 24U
#define BLE_INITIATOR_SUPERVISION_TIMEOUT_UNITS_DEFAULT 400U
#define BLE_CONN_TIMER_COMPARE_CC_INDEX 0U
#define BLE_CONN_TIMER_CAPTURE_CC_INDEX 1U
#define BLE_CONN_TIMER_PPI_CH_RADIO_ADDRESS_CAPTURE 26U
#define BLE_RADIO_IRQ_MASK_ADV (RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_RADIO_IRQ_MASK_SCAN (RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_RADIO_IRQ_MASK_CONN (RADIO_INTENSET_BCMATCH_Msk | RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_AD_TYPE_INCOMPLETE_UUID16_LIST 0x02U
#define BLE_AD_TYPE_COMPLETE_UUID16_LIST 0x03U
#define BLE_AD_TYPE_INCOMPLETE_UUID128_LIST 0x06U
#define BLE_AD_TYPE_COMPLETE_UUID128_LIST 0x07U
#define BLE_AD_TYPE_SHORT_LOCAL_NAME 0x08U
#define BLE_AD_TYPE_COMPLETE_LOCAL_NAME 0x09U

void controller_set_mode_with_phy(radio_mode_t mode, uint8_t phy);
void controller_conn_timer_schedule_compare(void);
uint32_t controller_conn_next_event_tick_from_anchor(uint32_t current_event_tick_us, uint16_t current_event_counter);
void controller_prepare_radio_common(uint8_t max_payload_size, const uint8_t *p_address, uint32_t crc_init, uint32_t packet_ptr);
void controller_apply_channel_map(const uint8_t *p_channel_map);
void controller_hop_data_channel(void);
ble_ll_data_header_t controller_conn_header(uint8_t llid);
ble_ll_data_header_t controller_conn_header_for_state(uint8_t llid, uint8_t next_expected_rx_sn, uint8_t tx_sn);
void controller_adv_timer_init(void);
void controller_adv_timer_start(uint32_t interval_ms);
void controller_adv_timer_stop(void);
void controller_scan_timers_init(void);
void controller_scan_timer_start(uint32_t interval_ms);
void controller_scan_timer_stop(void);
void controller_scan_window_timer_start(uint32_t window_ms);
void controller_scan_window_timer_stop(void);
bool controller_adv_type_is_connectable(uint8_t adv_type);
bool controller_adv_type_is_scannable(uint8_t adv_type);
bool controller_adv_type_is_reportable(uint8_t adv_type);
bool controller_radio_has_pending_completion(void);
bool controller_radio_is_rx_window_active(void);
void controller_reset_scan_radio_state(void);
void controller_reset_conn_bcmatch_state(void);
void controller_reset_adv_radio_state(void);
void controller_stage_conn_response(bool new_tx_pdu);
void controller_process_received_conn_pdu(void);
void controller_prestage_conn_response_from_header(void);
void controller_connected_timer_start(uint32_t first_event_delay_us);
void controller_prepare_connected_link(const ble_connect_req_pdu_t *p_req,
                                       ble_gap_role_t role,
                                       const ble_gap_addr_t *p_peer_addr);

bool controller_scan_request_targets_us(const ble_scan_req_pdu_t *p_req);
bool controller_connect_request_targets_us(const ble_connect_req_pdu_t *p_req);
bool controller_scan_filter_matches(const ble_gap_addr_t *p_addr, const ble_adv_rx_pdu_t *p_rx);
void controller_build_connect_request(const ble_gap_addr_t *p_peer_addr);
void controller_publish_scan_report(const ble_adv_rx_pdu_t *p_rx);
void controller_apply_peripheral_connect_request(const ble_connect_req_pdu_t *p_req);
void controller_apply_central_connect_request(void);
void controller_start_connection_event_peripheral(void);
void controller_start_connection_event_central(void);
void radio_handle_connected_packet_peripheral(void);
void radio_handle_connected_crc_error_peripheral(void);
void radio_handle_connected_packet_central(void);
void radio_handle_connected_crc_error_central(void);
void radio_handle_connected_bcmatch_peripheral(void);
void controller_handle_connected_disabled_peripheral(void);
void controller_handle_connected_disabled_central(void);
void controller_handle_advertising_disabled(void);
void controller_handle_scanning_disabled(void);

#endif
