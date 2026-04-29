#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
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
/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

static void
cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                        app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

#define MAX_logs_to_print 64
#define MAX_LOG_SIZE 96

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm, cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE, CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN, CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool log_in_progress;
static uint8_t logs_to_print;
static char log_buffer_queue[MAX_LOG_SIZE * MAX_logs_to_print];
static uint8_t print_next_indx;
static uint8_t load_next_indx;
static uint8_t log_size[MAX_logs_to_print];

static bool port_open = false;
/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void
cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                        app_usbd_cdc_acm_user_event_t event)
{

  switch (event)
  {
  case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
  {
    port_open = true;
    /*Setup first transfer*/
    ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer,
                                           READ_SIZE);
    UNUSED_VARIABLE(ret);
    break;
  }
  case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
    port_open = false;
    break;
  case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
  {
    print_next_indx++;
    if (print_next_indx == MAX_logs_to_print)
      print_next_indx = 0;
    logs_to_print--;
    log_in_progress = false;
    break;
  }
  case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
  {
    ret_code_t ret;
    do
    {
      /* Fetch data until internal buffer is empty */
      ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer,
                                  READ_SIZE);
    } while (ret == NRF_SUCCESS);

    break;
  }
  default:
    break;
  }
}

static void
usbd_user_ev_handler(app_usbd_event_type_t event)
{
  switch (event)
  {
  case APP_USBD_EVT_DRV_SUSPEND:

    break;
  case APP_USBD_EVT_DRV_RESUME:

    break;
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
  print_next_indx = 0;
  load_next_indx = 0;
  logs_to_print = 0;
  log_in_progress = false;

  ret_code_t ret;
  static const app_usbd_config_t usbd_config =
      {.ev_state_proc = usbd_user_ev_handler};

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
      /* Just waiting */
    }
  }

  app_usbd_serial_num_generate();

  APP_ERROR_CHECK(app_usbd_init(&usbd_config));

  app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(
      &m_app_cdc_acm);
  APP_ERROR_CHECK(app_usbd_class_append(class_cdc_acm));

  if (USBD_POWER_DETECTION)
  {
    APP_ERROR_CHECK(app_usbd_power_events_enable());
  }
  else
  {
    app_usbd_enable();
    app_usbd_start();
  }
}
void log_printf(const char *format, ...)
{
  char buffer[96];
  int written;
  size_t size;
  va_list args;

  va_start(args, format);
  written = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (written < 0)
    return;

  size = (size_t)written;
  if (size >= sizeof(buffer))
    size = sizeof(buffer) - 1U;
  if (size >= sizeof(tx_buffer))
    size = sizeof(tx_buffer) - 1U;

  memcpy(tx_buffer, buffer, size);
  tx_buffer[size] = '\0';

  if (size > MAX_LOG_SIZE)
    size = MAX_LOG_SIZE;
  log_size[load_next_indx] = size;
  memcpy(log_buffer_queue + load_next_indx * MAX_LOG_SIZE, tx_buffer, size);
  logs_to_print++;
  if (logs_to_print == MAX_logs_to_print) // needs to change this
    logs_to_print = 0;

  load_next_indx++;

  if (load_next_indx == MAX_logs_to_print)
    load_next_indx = 0;
}

void log_idle(void)
{
  while (app_usbd_event_queue_process())
    ;

  if (!log_in_progress && port_open && logs_to_print > 0)
  {
    app_usbd_cdc_acm_write(&m_app_cdc_acm,
                           log_buffer_queue + print_next_indx * MAX_LOG_SIZE,
                           log_size[print_next_indx]);
    log_in_progress = true;
  }
}
