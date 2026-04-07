/**
 * @file radio_driver.c
 * @author Surya Poudel
 * @brief Radio peripheral driver for nRF
 * @version 0.1
 * @date 2026-03-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <stdint.h>
#include "boards.h"
#include "radio_driver.h"

#define RADIO_IRQ_PRIORITY 1

static radio_event_handler_t radio_event_handler;

static void radio_clear_events(void)
{
    radio_clear_ready_event();
    radio_clear_end_event();
    radio_clear_disabled_event();
    NRF_RADIO->EVENTS_ADDRESS = 0U;
    radio_clear_bcmatch_event();
    radio_clear_crc_events();
}

void radio_set_event_handler(radio_event_handler_t evt_handler)
{
    radio_event_handler = evt_handler;
}

void radio_enable_interrupt_mask(uint32_t interrupt_mask)
{
    NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;
    radio_clear_bcmatch_event();
    radio_clear_crc_events();
    radio_clear_disabled_event();
    NRF_RADIO->INTENSET = interrupt_mask;

    NVIC_SetPriority(RADIO_IRQn, RADIO_IRQ_PRIORITY);
    NVIC_EnableIRQ(RADIO_IRQn);
}

void radio_cfg_drate_plen_and_enable_mode(radio_mode_t mode,
                                          radio_data_rate_t data_rate,
                                          radio_preamble_length_t preamble_length)
{
    if (radio_get_state() != DISABLED)
    {
        radio_disable();
    }

    radio_set_data_rate(data_rate);
    radio_set_preamble_length(preamble_length);
    radio_clear_events();

    switch (mode)
    {
    case RADIO_MODE_TX:
        radio_tx_enable();
        break;
    case RADIO_MODE_RX:
        radio_rx_enable();
        break;
    default:
        break;
    }

    radio_wait_ready();
    radio_clear_ready_event();
}

static void radio_set_mode(radio_mode_t mode)
{
    if (!(radio_get_state() == DISABLED))
    {
        radio_disable();
    }

    radio_clear_events();

    switch (mode)
    {
    case RADIO_MODE_TX:
        radio_tx_enable();
        break;
    case RADIO_MODE_RX:
        radio_rx_enable();
        break;
    default:
        break;
    }

    radio_wait_ready();
    radio_clear_ready_event();
}

void radio_tx_rx()
{
    // Reset shorcuts between tasks and events
    radio_set_shorts(0U);
    if (radio_get_state() != TX_IDLE)
    {
        radio_set_mode(RADIO_MODE_TX);
    }
    radio_clear_crc_events();
    radio_clear_end_event();
    radio_clear_disabled_event();
    radio_set_shorts(RADIO_SHORTS_END_DISABLE_Msk |
                     RADIO_SHORTS_DISABLED_RXEN_Msk |
                     RADIO_SHORTS_READY_START_Msk);
    radio_start();
}

void radio_set_address(const uint8_t *address, uint8_t length, uint8_t logical_address)
{

    uint32_t prefix_mask = 0x000000FF << (logical_address * 8);
    if (logical_address < 4)
    {
        NRF_RADIO->PREFIX0 &= ~prefix_mask;
        NRF_RADIO->PREFIX0 |= (uint32_t)(address[length - 1] << (logical_address * 8));
    }
    else
    {
        NRF_RADIO->PREFIX1 &= ~prefix_mask;
        NRF_RADIO->PREFIX1 |= (uint32_t)(address[length - 1] << (logical_address * 8));
    }
    uint8_t shift_pos = 24;
    if (logical_address == 0)
    {
        NRF_RADIO->BASE0 = 0UL;

        for (int i = length - 2; i >= 0; i--)
        {
            NRF_RADIO->BASE0 |= ((uint32_t)address[i]) << shift_pos;
            shift_pos -= 8;
        }
    }
    else if (logical_address > 0 && logical_address <= 7)
    {
        NRF_RADIO->BASE1 = 0UL;
        for (int i = length - 2; i >= 0; i--)
        {
            NRF_RADIO->BASE1 |= ((uint32_t)address[i]) << shift_pos;
            shift_pos -= 8;
        }
    }
}

void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_BCMATCH)
    {
        NRF_RADIO->EVENTS_BCMATCH = 0U;

        if (radio_event_handler != 0)
        {
            radio_event_handler(RADIO_EVENT_BCMATCH);
        }
    }

    if (NRF_RADIO->EVENTS_CRCERROR)
    {
        NRF_RADIO->EVENTS_CRCERROR = 0U;

        if (radio_event_handler != 0)
        {
            radio_event_handler(RADIO_EVENT_CRC_ERROR);
        }
    }
    else if (NRF_RADIO->EVENTS_CRCOK)
    {
        NRF_RADIO->EVENTS_CRCOK = 0U;

        if (radio_event_handler != 0)
        {
            radio_event_handler(RADIO_EVENT_CRC_OK);
        }
    }

    if (NRF_RADIO->EVENTS_DISABLED)
    {
        NRF_RADIO->EVENTS_DISABLED = 0U;

        if (radio_event_handler != 0)
        {
            radio_event_handler(RADIO_EVENT_DISABLED);
        }
    }

}
