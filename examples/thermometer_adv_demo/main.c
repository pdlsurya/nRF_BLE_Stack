/* SPDX-License-Identifier: MIT */

#include <stdint.h>

#include "app_error.h"
#include "app_timer.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_ble.h"
#include "nrf_drv_clock.h"
#include "usb_log.h"

APP_TIMER_DEF(m_temperature_timer_id);

#define BLE_ADV_LED_IDX BSP_BOARD_LED_0
#define HEALTH_THERMOMETER_SERVICE_UUID 0x1809U

static void clock_init(void);
static void start_advertising(void);
static void temperature_timer_handler(void *p_context);
static uint32_t timer_ticks_clamped(uint32_t ms);

static const char m_dev_name[] = "BLE-Therm";
static uint8_t m_temperature_c = 24U;
static const int8_t m_adv_tx_power = 0x08;
static const ble_uuid_t m_health_thermometer_service_uuid =
    BLE_UUID_SIG16_INIT(HEALTH_THERMOMETER_SERVICE_UUID);
static const ble_gap_adv_service_uuid_list_config_t m_adv_service_uuid_lists[] = {
    {
        .type = BLE_GAP_ADV_SERVICE_UUID_LIST_COMPLETE_16,
        .p_uuids = &m_health_thermometer_service_uuid,
        .uuid_count = 1U,
    },
};
static const ble_gap_adv_name_config_t m_adv_name = {
    .name_type = BLE_GAP_ADV_NAME_SHORT,
    .short_name_length = 8U,
};
static const ble_gap_adv_service_data_t m_temperature_service_data = {
    .uuid = BLE_UUID_SIG16_INIT(HEALTH_THERMOMETER_SERVICE_UUID),
    .p_data = &m_temperature_c,
    .data_len = sizeof(m_temperature_c),
};
static const ble_gap_adv_config_t m_adv_config = {
    .flags = (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                       BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED),
    .interval_ms = 100U,
    .adv_type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED,
    .adv_data = {
        .p_name = &m_adv_name,
        .p_tx_power = &m_adv_tx_power,
        .p_service_uuid_lists = m_adv_service_uuid_lists,
        .service_uuid_list_count = 1U,
        .p_service_data = &m_temperature_service_data,
    },
};

static void clock_init(void)
{
  ret_code_t err;

  if (!nrf_drv_clock_init_check())
  {
    err = nrf_drv_clock_init();
    if ((err != NRF_SUCCESS) && (err != NRF_ERROR_MODULE_ALREADY_INITIALIZED))
    {
      APP_ERROR_CHECK(err);
    }
  }

  nrf_drv_clock_lfclk_request(NULL);
  while (!nrf_drv_clock_lfclk_is_running())
  {
  }

  nrf_drv_clock_hfclk_request(NULL);
  while (!nrf_drv_clock_hfclk_is_running())
  {
  }
}

static void start_advertising(void)
{
  bsp_board_led_on(BLE_ADV_LED_IDX);
  ble_gap_start_advertising();
  log_printf("BLE thermometer advertising started\n");
}

static uint32_t timer_ticks_clamped(uint32_t ms)
{
  uint32_t ticks = APP_TIMER_TICKS(ms);

  return (ticks < APP_TIMER_MIN_TIMEOUT_TICKS) ? APP_TIMER_MIN_TIMEOUT_TICKS : ticks;
}

static void temperature_timer_handler(void *p_context)
{
  (void)p_context;

  m_temperature_c++;
  if (m_temperature_c > 34U)
  {
    m_temperature_c = 24U;
  }

  log_printf("BLE thermometer service data temp=%u C\n", (unsigned int)m_temperature_c);
}

int main(void)
{
  bsp_board_init(BSP_INIT_LEDS);
  clock_init();
  log_init();

  ble_stack_init(BLE_GAP_ROLE_PERIPHERAL);
  ble_gap_set_device_name(m_dev_name);
  APP_ERROR_CHECK_BOOL(ble_gap_adv_init(&m_adv_config));

  APP_ERROR_CHECK(app_timer_create(&m_temperature_timer_id,
                                   APP_TIMER_MODE_REPEATED,
                                   temperature_timer_handler));
  APP_ERROR_CHECK(app_timer_start(m_temperature_timer_id, timer_ticks_clamped(1000U), NULL));

  start_advertising();

  while (1)
  {
    __WFE();
  }
}
