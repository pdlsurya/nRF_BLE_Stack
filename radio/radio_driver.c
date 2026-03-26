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
#include <string.h>
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
    radio_clear_crc_events();
}

void radio_set_event_handler(radio_event_handler_t evt_handler)
{
    radio_event_handler = evt_handler;
}

void radio_set_mode(radio_mode_t mode)
{
    if (!(radio_get_state() == DISABLED))
    {
        radio_disable();
    }

    radio_clear_events();

    switch (mode)
    {
    case MODE_TX:
        radio_tx_enable();
        break;
    case MODE_RX:
        radio_rx_enable();
        break;
    default:
        break;
    }

    radio_wait_ready();
    radio_clear_ready_event();
}

void radio_tx()
{
    if (radio_get_state() != TX_IDLE)
    {
        radio_set_mode(MODE_TX);
    }

    radio_start();

    radio_wait_end();
    radio_clear_end_event();

    radio_disable();
}

void radio_rx()
{
    if (radio_get_state() != RX_IDLE)
    {
        radio_set_mode(MODE_RX);
    }

    radio_clear_crc_events();
    radio_clear_end_event();
    radio_start();
}

void radio_tx_rx()
{
    // Reset shorcuts between tasks and events
    radio_set_shorts(0U);
    if (radio_get_state() != TX_IDLE)
    {
        radio_set_mode(MODE_TX);
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

void radio_enable_interrupts()
{
    NRF_RADIO->INTENCLR = 0xFFFFFFFFUL;
    radio_clear_crc_events();
    NRF_RADIO->INTENSET = RADIO_INTENSET_CRCOK_Msk | RADIO_INTENSET_CRCERROR_Msk;

    NVIC_SetPriority(RADIO_IRQn, RADIO_IRQ_PRIORITY);
    NVIC_EnableIRQ(RADIO_IRQn);
}

void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_CRCERROR)
    {
        NRF_RADIO->EVENTS_CRCERROR = 0U;
        return;
    }

    if (NRF_RADIO->EVENTS_CRCOK)
    {
        NRF_RADIO->EVENTS_CRCOK = 0;

        if (radio_event_handler != 0)
        {
            radio_event_handler();
        }
    }
}
