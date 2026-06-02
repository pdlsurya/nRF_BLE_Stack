/* SPDX-License-Identifier: MIT */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "bsp.h"
#include "stdarg.h"

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

#define MAX_LOGS_TO_PRINT 64
#define MAX_LOG_SIZE (NRF_DRV_USBD_EPSIZE - 1)

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static volatile bool log_in_progress;
static volatile uint8_t logs_to_print;
static char log_buffer_queue[MAX_LOG_SIZE * MAX_LOGS_TO_PRINT];
static volatile uint8_t print_next_indx;
static volatile uint8_t load_next_indx;
static uint8_t log_size[MAX_LOGS_TO_PRINT];
static volatile bool port_open;

static void log_tx_pump(void);

static void log_queue_pop_sent(void)
{
  CRITICAL_REGION_ENTER();
  if (logs_to_print > 0)
  {
    print_next_indx++;
    if (print_next_indx == MAX_LOGS_TO_PRINT)
    {
      print_next_indx = 0;
    }
    logs_to_print--;
  }
  log_in_progress = false;
  CRITICAL_REGION_EXIT();
}

static void log_tx_pump(void)
{
  ret_code_t ret;
  uint8_t read_idx;
  uint8_t size;

  CRITICAL_REGION_ENTER();
  if (log_in_progress || !port_open || (logs_to_print == 0))
  {
    CRITICAL_REGION_EXIT();
    return;
  }

  read_idx = print_next_indx;
  size = log_size[read_idx];
  log_in_progress = true;
  CRITICAL_REGION_EXIT();

  ret = app_usbd_cdc_acm_write(&m_app_cdc_acm,
                               log_buffer_queue + read_idx * MAX_LOG_SIZE,
                               size);
  if (ret != NRF_SUCCESS)
  {
    CRITICAL_REGION_ENTER();
    log_in_progress = false;
    CRITICAL_REGION_EXIT();
  }
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
  UNUSED_PARAMETER(p_inst);

  switch (event)
  {
  case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
  {
    port_open = true;
    ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
    UNUSED_VARIABLE(ret);
    log_tx_pump();
    break;
  }

  case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
    port_open = false;
    break;

  case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
    log_queue_pop_sent();
    log_tx_pump();
    break;

  case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
  {
    ret_code_t ret;
    do
    {
      ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
    } while (ret == NRF_SUCCESS);
    break;
  }

  default:
    break;
  }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
  switch (event)
  {
  case APP_USBD_EVT_DRV_SUSPEND:
  case APP_USBD_EVT_DRV_RESUME:
  case APP_USBD_EVT_STARTED:
    break;

  case APP_USBD_EVT_STOPPED:
    app_usbd_disable();
    bsp_board_leds_off();
    break;

  case APP_USBD_EVT_POWER_DETECTED:
    if (!nrf_drv_usbd_is_enabled())
    {
      app_usbd_enable();
    }
    break;

  case APP_USBD_EVT_POWER_REMOVED:
    app_usbd_stop();
    break;

  case APP_USBD_EVT_POWER_READY:
    app_usbd_start();
    break;

  default:
    break;
  }
}

void log_init(void)
{
  ret_code_t ret;
  static const app_usbd_config_t usbd_config = {
      .ev_state_proc = usbd_user_ev_handler,
  };

  print_next_indx = 0;
  load_next_indx = 0;
  logs_to_print = 0;
  log_in_progress = false;
  port_open = false;

  if (!nrf_drv_clock_init_check())
  {
    ret = nrf_drv_clock_init();
    if ((ret != NRF_SUCCESS) && (ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED))
    {
      APP_ERROR_CHECK(ret);
    }
  }

  if (!nrf_drv_clock_lfclk_is_running())
  {
    nrf_drv_clock_lfclk_request(NULL);
    while (!nrf_drv_clock_lfclk_is_running())
    {
    }
  }

  app_usbd_serial_num_generate();

  ret = app_usbd_init(&usbd_config);
  APP_ERROR_CHECK(ret);

  app_usbd_class_inst_t const *class_cdc_acm =
      app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
  ret = app_usbd_class_append(class_cdc_acm);
  APP_ERROR_CHECK(ret);

  if (USBD_POWER_DETECTION)
  {
    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);
  }
  else
  {
    app_usbd_enable();
    app_usbd_start();
  }
}

void log_printf(const char *format, ...)
{
  char buffer[MAX_LOG_SIZE + 1];
  int written;
  size_t size;
  uint8_t write_idx;
  va_list args;

  va_start(args, format);
  written = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (written < 0)
  {
    return;
  }

  size = (size_t)written;
  if (size >= sizeof(buffer))
  {
    size = sizeof(buffer) - 1U;
  }

  CRITICAL_REGION_ENTER();
  if (logs_to_print == MAX_LOGS_TO_PRINT)
  {
    CRITICAL_REGION_EXIT();
    return;
  }

  write_idx = load_next_indx;
  log_size[write_idx] = (uint8_t)size;
  memcpy(log_buffer_queue + write_idx * MAX_LOG_SIZE, buffer, size);

  load_next_indx++;
  if (load_next_indx == MAX_LOGS_TO_PRINT)
  {
    load_next_indx = 0;
  }
  logs_to_print++;
  CRITICAL_REGION_EXIT();

  log_tx_pump();
}
