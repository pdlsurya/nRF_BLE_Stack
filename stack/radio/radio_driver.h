/**
 * @file radio_driver.h
 * @author Surya Poudel
 * @brief Radio peripheral driver interface for nRF
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef _NRF_RADIO_DRIVER_H
#define _NRF_RADIO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"

typedef enum
{
	RADIO_MODE_TX,
	RADIO_MODE_RX
} radio_mode_t;

typedef enum
{
	RADIO_EVENT_READY,
	RADIO_EVENT_BCMATCH,
	RADIO_EVENT_CRC_OK,
	RADIO_EVENT_CRC_ERROR,
	RADIO_EVENT_DISABLED
} radio_event_t;

typedef void (*radio_event_handler_t)(radio_event_t evt);

typedef enum
{
	BLE_1MBPS = 3,
	BLE_2MBPS
} radio_data_rate_t;

typedef enum
{
	RADIO_LITTLE_ENDIAN,
	RADIO_BIG_ENDIAN
} radio_endian_t;

typedef enum
{
	RADIO_PREAMBLE_8BIT,
	RADIO_PREAMBLE_16BIT
} radio_preamble_length_t;

typedef enum
{
	DISABLED,
	RX_RU,
	RX_IDLE,
	RX,
	RX_DISABLE,
	TX_RU = 9,
	TX_IDLE,
	TX,
	TX_DISABLE
} radio_state_t;

typedef enum
{
	RADIO_RAMP_UP_DEFAULT,
	RADIO_RAMP_UP_FAST
} radio_ramp_up_t;

typedef enum
{
	RADIO_DEFAULT_TX_B1 = 0,
	RADIO_DEFAULT_TX_B0 = 1,
	RADIO_DEFAULT_TX_CENTER = 2
} radio_default_tx_t;

void radio_set_event_handler(radio_event_handler_t handler);

void radio_enable_interrupt_mask(uint32_t interrupt_mask);

void radio_cfg_drate_plen_and_enable_mode(radio_mode_t mode,
										  radio_data_rate_t data_rate,
										  radio_preamble_length_t preamble_length);

void radio_tx_rx();
void radio_tx_then_rx(uint32_t tx_packet_ptr, uint32_t rx_packet_ptr);

void radio_set_address(const uint8_t *address, uint8_t length, uint8_t logical_address);

static inline radio_state_t radio_get_state()
{
	return NRF_RADIO->STATE;
}

static inline void radio_disable()
{
	if (radio_get_state() == DISABLED)
	{
		NRF_RADIO->EVENTS_DISABLED = 0U;
		return;
	}

	NRF_RADIO->EVENTS_DISABLED = 0U;
	NRF_RADIO->TASKS_DISABLE = 1U;
	while (NRF_RADIO->EVENTS_DISABLED == 0)
		;

	NRF_RADIO->EVENTS_DISABLED = 0;
}

static inline void radio_set_frequency(uint32_t frequency)
{
	NRF_RADIO->FREQUENCY = frequency;
}

static inline void radio_set_tx_power(uint32_t tx_power)
{
	NRF_RADIO->TXPOWER = tx_power;
}

static inline void radio_set_data_rate(radio_data_rate_t data_rate)
{
	NRF_RADIO->MODE = (uint32_t)data_rate;
}

static inline void radio_reset_packet_config(void)
{
	NRF_RADIO->PCNF0 = 0U;
	NRF_RADIO->PCNF1 = 0U;
	NRF_RADIO->CRCCNF = 0U;
}

static inline void radio_set_len_field_size(uint8_t bits)
{
	NRF_RADIO->PCNF0 |= (uint32_t)bits << 0;
}

static inline void radio_set_s0_field_size(uint8_t bytes)
{
	NRF_RADIO->PCNF0 |= (uint32_t)bytes << 8;
}

static inline void radio_set_preamble_length(radio_preamble_length_t preamble_length)
{
	NRF_RADIO->PCNF0 &= ~((uint32_t)0x3U << 24);
	NRF_RADIO->PCNF0 |= (uint32_t)preamble_length << 24;
}

static inline void radio_set_max_payload_size(uint32_t max_pl_size)
{
	NRF_RADIO->PCNF1 |= (uint32_t)max_pl_size << 0; // max payload length
}

static inline void radio_set_address_width(uint8_t add_width)
{
	NRF_RADIO->PCNF1 |= ((uint32_t)(add_width - 1)) << 16;
}

static inline void radio_set_payload_endian(radio_endian_t endian)
{
	NRF_RADIO->PCNF1 &= ~(((uint32_t)1) << 24);
	if (endian == RADIO_BIG_ENDIAN)
		NRF_RADIO->PCNF1 |= ((uint32_t)1) << 24;
}

static inline void radio_enable_whitening(bool en)
{
	NRF_RADIO->PCNF1 &= ~(((uint32_t)1U) << 25);
	if (en)
	{
		NRF_RADIO->PCNF1 |= ((uint32_t)1U) << 25;
	}
}

static inline void radio_set_tx_logical_address(uint8_t logical_address)
{
	NRF_RADIO->TXADDRESS = (uint32_t)logical_address;
}

static inline void radio_set_rx_logical_address(uint8_t logical_address)
{
	NRF_RADIO->RXADDRESSES = 1UL << logical_address;
}

static inline void radio_configure_crc(uint8_t crc_len, uint8_t crc_add, uint32_t crc_poly, uint32_t crc_init_val)
{
	NRF_RADIO->CRCCNF = (((uint32_t)crc_len) << 0) |
						(((uint32_t)crc_add) << 8);

	NRF_RADIO->CRCPOLY = crc_poly;
	NRF_RADIO->CRCINIT = crc_init_val;
}

static inline void radio_set_whiteiv(uint8_t init_value)
{
	NRF_RADIO->DATAWHITEIV = (uint32_t)(init_value);
}

static inline void radio_set_packet_ptr(uint32_t ptr)
{
	NRF_RADIO->PACKETPTR = ptr;
}

static inline void radio_set_shorts(uint32_t shorts)
{
	NRF_RADIO->SHORTS = shorts;
}

static inline void radio_clear_ready_event(void)
{
	NRF_RADIO->EVENTS_READY = 0U;
}

static inline void radio_clear_end_event(void)
{
	NRF_RADIO->EVENTS_END = 0U;
}

static inline void radio_clear_disabled_event(void)
{
	NRF_RADIO->EVENTS_DISABLED = 0U;
}

static inline void radio_clear_crc_events(void)
{
	NRF_RADIO->EVENTS_CRCOK = 0U;
	NRF_RADIO->EVENTS_CRCERROR = 0U;
}

static inline void radio_clear_address_event(void)
{
	NRF_RADIO->EVENTS_ADDRESS = 0U;
}

static inline void radio_clear_bcmatch_event(void)
{
	NRF_RADIO->EVENTS_BCMATCH = 0U;
}

static inline void radio_set_bcc(uint32_t bit_count)
{
	NRF_RADIO->BCC = bit_count;
}

static inline void radio_wait_ready(void)
{
	while (NRF_RADIO->EVENTS_READY == 0U)
		;
}

static inline void radio_tx_enable(void)
{
	NRF_RADIO->TASKS_TXEN = 1U;
}

static inline void radio_rx_enable(void)
{
	NRF_RADIO->TASKS_RXEN = 1U;
}

static inline void radio_start(void)
{
	NRF_RADIO->TASKS_START = 1U;
}

static inline void radio_power_on(void)
{
	NRF_RADIO->POWER = 1U;
}

static inline void radio_set_tifs(uint16_t tifs_us)
{
	NRF_RADIO->TIFS = tifs_us;
}

static inline void radio_configure_modecnf0(radio_ramp_up_t ramp_up,
											radio_default_tx_t default_tx)
{
	NRF_RADIO->MODECNF0 = ((uint32_t)ramp_up << 0) |
						  ((uint32_t)default_tx << 8);
}
#endif
