/* SPDX-License-Identifier: MIT */

#ifndef BLE_CONTROLLER_SHARED_H__
#define BLE_CONTROLLER_SHARED_H__

#include "ble_runtime_internal.h"

#include "app_error.h"
#include "nrf_delay.h"

#define BLE_RADIO_IRQ_MASK_ADV (RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_RADIO_IRQ_MASK_SCAN (RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_RADIO_IRQ_MASK_CONN_CENTRAL (RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk | RADIO_INTENSET_DISABLED_Msk)
#define BLE_RADIO_IRQ_MASK_CONN_PERIPHERAL (RADIO_INTENSET_BCMATCH_Msk | BLE_RADIO_IRQ_MASK_CONN_CENTRAL)
#define BLE_CONN_TIMER_COMPARE_CC_INDEX 0U
#define BLE_CONN_TIMER_CAPTURE_CC_INDEX 1U
#define BLE_CONN_TIMER_PPI_CH_RADIO_ADDRESS_CAPTURE 26U

void controller_set_mode_with_phy(radio_mode_t mode, uint8_t phy);
void controller_conn_timer_schedule_compare(void);
uint32_t controller_conn_next_event_tick_from_anchor(uint32_t current_event_tick_us, uint16_t current_event_counter);
void controller_prepare_radio_common(uint8_t max_payload_size, const uint8_t *p_address, uint32_t crc_init, uint32_t packet_ptr);
void controller_hop_data_channel(void);
ble_ll_data_header_t controller_conn_header(uint8_t llid);
ble_ll_data_header_t controller_conn_header_for_state(uint8_t llid, uint8_t next_expected_rx_sn, uint8_t tx_sn);
void controller_peripheral_adv_timer_init(void);
void controller_central_scan_timers_init(void);
void controller_central_scan_timer_stop(void);
void controller_central_scan_window_timer_stop(void);
void controller_reset_conn_tx_selection_state(void);
bool controller_queue_ll_control_payload(const uint8_t *p_payload, uint8_t payload_len);
void controller_schedule_conn_update(const uint8_t *p_payload, uint8_t len);
bool controller_load_pending_conn_tx_pdu_for_state(uint8_t next_expected_rx_sn, uint8_t tx_sn);
void controller_stage_conn_response(bool new_tx_pdu);
void controller_process_received_conn_pdu(void);
void controller_connected_timer_start(uint32_t first_event_delay_us);
void controller_prepare_connected_link(const ble_ll_connect_req_pdu_t *p_req,
                                       ble_gap_role_t role,
                                       const ble_gap_addr_t *p_peer_addr);

void controller_central_state_reset(void);
void controller_central_handle_feature_exchange_complete(void);
void controller_central_handle_data_length_update_complete(void);
void controller_central_handle_unknown_rsp(uint8_t unsupported_opcode);
void controller_central_handle_conn_update_instant_complete(void);
void controller_central_handle_phy_update_instant_complete(void);
void controller_central_auto_ctrl_start(void);
uint16_t controller_central_process_phy_rsp(const uint8_t *p_payload, uint8_t len, uint8_t *p_rsp);
void controller_peripheral_state_reset(void);
void controller_peripheral_start_connection_event(void);
void controller_central_start_connection_event(void);
void controller_central_handle_radio_event(radio_event_t evt, const ble_ll_adv_pdu_t *p_adv_rx);
void controller_peripheral_handle_radio_event(radio_event_t evt, const ble_ll_adv_req_pdu_t *p_adv_rx);

#endif
