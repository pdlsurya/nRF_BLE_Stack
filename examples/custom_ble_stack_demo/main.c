#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "boards.h"
#include "debug_log.h"
#include "nrf_ble.h"
#include "nrf_delay.h"
#include "nrf_power.h"

APP_TIMER_DEF(m_measurement_timer_id);

#define BLE_ADV_LED_IDX       BSP_BOARD_LED_0
#if (LEDS_NUMBER > 1)
#define BLE_CONNECTED_LED_IDX BSP_BOARD_LED_3
#else
#define BLE_CONNECTED_LED_IDX BSP_BOARD_LED_0
#endif

static void counter_char_evt_handler(const ble_gatt_evt_t *p_evt);
static void text_char_evt_handler(const ble_gatt_evt_t *p_evt);
static void ble_evt_handler(const ble_evt_t *p_evt);
static void ble_state_set(bool connected);
static void start_advertising(void);
static uint32_t timer_ticks_clamped(uint32_t ms);
static void timer_stop_if_started(app_timer_id_t timer_id);

static uint8_t m_counter_char_value; 
static uint16_t m_counter_char_value_len = 1U;
static char m_text_char_value[BLE_GATT_MAX_VALUE_LEN] = "";
static uint16_t m_text_char_value_len = 0U;
static const ble_adv_config_t m_adv_config = {
  .p_adv_name = "nrf52-ble",
  .flags = 0x06,
  .tx_power = 0x08,
  .interval_ms = 100U,
  .included_service_uuid = NRF_CUSTOM_SERVICE_UUID,
  .p_service_data = NULL,
};
static const ble_gap_conn_params_t m_gap_conn_params = {
  .min_conn_interval_units = BLE_MS_TO_1P25MS_UNITS(20U),
  .max_conn_interval_units = BLE_MS_TO_1P25MS_UNITS(75U),
  .slave_latency = 0U,
  .supervision_timeout_units = BLE_MS_TO_10MS_UNITS(4000U),
};

static ble_gatt_characteristic_t m_custom_characteristics[] = {
    {
        .uuid = NRF_CUSTOM_COUNTER_CHAR_UUID,
        .properties = (uint8_t)(BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_NOTIFY),
        .p_value = &m_counter_char_value,
        .p_value_len = &m_counter_char_value_len,
        .max_len = sizeof(m_counter_char_value),
        .evt_handler = counter_char_evt_handler,
        .value_handle = 0U,
        .cccd_handle = 0U,
    },
    {
        .uuid = NRF_CUSTOM_TEXT_CHAR_UUID,
        .properties = (uint8_t)(BLE_GATT_CHAR_PROP_READ |
                                BLE_GATT_CHAR_PROP_WRITE |
                                BLE_GATT_CHAR_PROP_WRITE_NO_RESP),
        .p_value = (uint8_t *)m_text_char_value,
        .p_value_len = &m_text_char_value_len,
        .max_len = sizeof(m_text_char_value),
        .evt_handler = text_char_evt_handler,
        .value_handle = 0U,
        .cccd_handle = 0U,
    },
};

static ble_gatt_service_t m_custom_services[] = {
    {
        .uuid = NRF_CUSTOM_SERVICE_UUID,
        .p_characteristics = m_custom_characteristics,
        .characteristic_count = (uint8_t)(sizeof(m_custom_characteristics) / sizeof(m_custom_characteristics[0])),
        .service_handle = 0U,
    },
};

static int nrf52840dongle_regout0_config(void)
{
  if ((nrf_power_mainregstatus_get() == NRF_POWER_MAINREGSTATUS_HIGH) &&
      ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
       (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)))
  {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

    NRF_UICR->REGOUT0 =
        (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
        (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

    NVIC_SystemReset();
  }

  return 0;
}

static void lf_clock_start(void)
{
  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;

  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
  {
  }
}

static void hf_clock_start(void)
{
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
  {
  }
}

static void ble_state_set(bool connected)
{
  if (connected)
  {
    bsp_board_led_off(BLE_ADV_LED_IDX);
    bsp_board_led_on(BLE_CONNECTED_LED_IDX);
    return;
  }

  bsp_board_led_on(BLE_ADV_LED_IDX);
  bsp_board_led_off(BLE_CONNECTED_LED_IDX);
}

static void start_advertising(void)
{
  ble_state_set(false);
  ble_start_advertising();
  debug_log_print("BLE advertising started\n");
}

static uint32_t timer_ticks_clamped(uint32_t ms)
{
  uint32_t ticks = APP_TIMER_TICKS(ms);

  return (ticks < APP_TIMER_MIN_TIMEOUT_TICKS) ? APP_TIMER_MIN_TIMEOUT_TICKS : ticks;
}

static void timer_stop_if_started(app_timer_id_t timer_id)
{
  ret_code_t err = app_timer_stop(timer_id);

  if ((err != NRF_SUCCESS) && (err != NRF_ERROR_INVALID_STATE))
  {
    APP_ERROR_CHECK(err);
  }
}

static void measurement_timer_handler(void *p_context)
{
  (void)p_context;

  if (!ble_is_connected())
  {
    return;
  }

  m_counter_char_value++;
  if (ble_notify_characteristic(&m_custom_characteristics[0]))
  {
    debug_log_print("BLE GATT: notified counter=%u\n",
                    (unsigned int)m_counter_char_value);
  }
}

static void ble_evt_handler(const ble_evt_t *p_evt)
{
  ret_code_t err;

  if (p_evt == NULL)
  {
    return;
  }

  switch (p_evt->evt_type)
  {
  case BLE_GAP_EVT_SUPERVISION_TIMEOUT:
    debug_log_print("BLE LINK: supervision timeout\n");
    return;

  case BLE_GAP_EVT_CONN_UPDATE_IND:
    debug_log_print("BLE LINK: connection update indicated (not yet applied)\n");
    return;

  case BLE_GAP_EVT_TERMINATE_IND:
    debug_log_print("BLE LINK: terminate indication received\n");
    return;

  case BLE_GATT_EVT_MTU_EXCHANGE:
    debug_log_print("BLE ATT: MTU exchange req=%u rsp=%u effective=%u\n",
                    (unsigned int)p_evt->requested_mtu,
                    (unsigned int)p_evt->response_mtu,
                    (unsigned int)p_evt->effective_mtu);
    return;

  case BLE_GAP_EVT_CONNECTED:
    m_counter_char_value = 0U;
    m_counter_char_value_len = 1U;
    ble_state_set(true);
    err = app_timer_start(m_measurement_timer_id, timer_ticks_clamped(1000U), NULL);
    APP_ERROR_CHECK(err);
    debug_log_print("BLE GAP: connected, interval=%d ms timeout=%d ms\n",
                    (int)p_evt->conn_interval_ms, (int)p_evt->supervision_timeout_ms);
    return;

  case BLE_GAP_EVT_DISCONNECTED:
    timer_stop_if_started(m_measurement_timer_id);
    debug_log_print("BLE GAP: disconnected\n");
    start_advertising();
    return;

  default:
    return;
  }
}

static void counter_char_evt_handler(const ble_gatt_evt_t *p_evt)
{
  if (p_evt == NULL)
  {
    return;
  }

  if ((p_evt->evt_type == BLE_GATT_EVT_NOTIFY_ENABLED) ||
      (p_evt->evt_type == BLE_GATT_EVT_NOTIFY_DISABLED))
  {
    debug_log_print("BLE GATT: counter notifications %s\n",
                    p_evt->notifications_enabled ? "enabled" : "disabled");
  }
}

static void text_char_evt_handler(const ble_gatt_evt_t *p_evt)
{
  char written_text[BLE_GATT_MAX_VALUE_LEN + 1U];
  uint16_t copy_len;

  if ((p_evt == NULL) || (p_evt->evt_type != BLE_GATT_EVT_WRITE))
  {
    return;
  }

  copy_len = p_evt->len;
  if (copy_len > BLE_GATT_MAX_VALUE_LEN)
  {
    copy_len = BLE_GATT_MAX_VALUE_LEN;
  }

  if ((copy_len > 0U) && (p_evt->p_data != NULL))
  {
    (void)memcpy(written_text, p_evt->p_data, copy_len);
  }
  written_text[copy_len] = '\0';

  debug_log_print("BLE GATT: write text=\"%s\"\n", written_text);
}

int main(void)
{
  ret_code_t err;
  nrf52840dongle_regout0_config();
  hf_clock_start();
  lf_clock_start();
  debug_log_init();
  bsp_board_init(BSP_INIT_LEDS);

  ble_stack_init();
  ble_register_evt_handler(ble_evt_handler);
  ble_adv_init(&m_adv_config);
  ble_gap_init(&m_gap_conn_params);
  APP_ERROR_CHECK_BOOL(ble_gatt_server_init(m_custom_services,
                                            (uint8_t)(sizeof(m_custom_services) / sizeof(m_custom_services[0]))));

  err = app_timer_create(&m_measurement_timer_id,
                         APP_TIMER_MODE_REPEATED,
                         measurement_timer_handler);
  APP_ERROR_CHECK(err);

  start_advertising();

  while (1)
  {
    debug_log_process();
  }
}
