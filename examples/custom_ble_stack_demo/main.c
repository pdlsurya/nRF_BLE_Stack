#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "boards.h"
#include "usb_log.h"
#include "nrf_ble.h"
#include "nrf_drv_clock.h"

APP_TIMER_DEF(m_measurement_timer_id);

#define BLE_ADV_LED_IDX BSP_BOARD_LED_0
#if (LEDS_NUMBER > 1)
#define BLE_CONNECTED_LED_IDX BSP_BOARD_LED_3
#else
#define BLE_CONNECTED_LED_IDX BSP_BOARD_LED_0
#endif

static void counter_char_evt_handler(const ble_gatt_char_evt_t *p_evt);
static void text_char_evt_handler(const ble_gatt_char_evt_t *p_evt);
static void ble_evt_handler(const ble_evt_t *p_evt);
static void ble_state_set(bool connected);
static void start_advertising(void);
static void clock_init(void);
static uint32_t timer_ticks_clamped(uint32_t ms);
static void timer_stop_if_started(app_timer_id_t timer_id);
static const char *ble_phy_name(uint8_t phy);

static const char m_dev_name[] = "nRF-BLE-Custom-Stack";
static uint8_t m_counter_char_value;
static char m_text_char_value[BLE_GATT_MAX_VALUE_LEN] = "";
static const uint8_t m_custom_uuid_base[BLE_UUID128_LEN] = {
    0x52U,
    0xD0U,
    0x4FU,
    0x36U,
    0x7EU,
    0x85U,
    0x74U,
    0x1CU,
    0xA6U,
    0x8FU,
    0x4EU,
    0x7AU,
    0x00U,
    0x00U,
    0x00U,
    0x00U,
};
static const ble_uuid_t m_custom_service_uuid = BLE_UUID_VENDOR16_INIT(0xFFF0U);
static const ble_gap_conn_params_t m_gap_conn_params = {
    .min_conn_interval_1p25ms = MS_TO_1P25MS_UNITS(30U),
    .max_conn_interval_1p25ms = MS_TO_1P25MS_UNITS(30U),
    .slave_latency = 0U,
    .supervision_timeout_10ms = MS_TO_10MS_UNITS(1500U),
};
static const ble_adv_config_t m_adv_config = {
    .flags = (uint8_t)(BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE |
                       BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED),
    .tx_power = 0x08,
    .interval_ms = 100U,
    .p_included_service_uuid = &m_custom_service_uuid,
    .name_type = BLE_GAP_ADV_NAME_SHORT,
    .short_name_length = 7U,
};

static ble_gatt_characteristic_t m_custom_characteristics[] = {
    {
        .uuid = BLE_UUID_VENDOR16_INIT(0xFFF1U),
        .properties = (uint8_t)(BLE_GATT_CHAR_PROP_READ | BLE_GATT_CHAR_PROP_NOTIFY | BLE_GATT_CHAR_PROP_INDICATE),
        .p_value = &m_counter_char_value,
        .value_len = sizeof(m_counter_char_value),
        .max_len = sizeof(m_counter_char_value),
        .evt_handler = counter_char_evt_handler,
        .value_handle = 0U,
        .cccd_handle = 0U,
    },
    {
        .uuid = BLE_UUID_VENDOR16_INIT(0xFFF2U),
        .properties = (uint8_t)(BLE_GATT_CHAR_PROP_READ |
                                BLE_GATT_CHAR_PROP_WRITE |
                                BLE_GATT_CHAR_PROP_WRITE_NO_RESP),
        .p_value = (uint8_t *)m_text_char_value,
        .value_len = 0U,
        .max_len = sizeof(m_text_char_value),
        .evt_handler = text_char_evt_handler,
        .value_handle = 0U,
        .cccd_handle = 0U,
    },
};

static ble_gatt_service_t m_custom_services[] = {
    {
        .uuid = BLE_UUID_VENDOR16_INIT(0xFFF0U),
        .p_characteristics = m_custom_characteristics,
        .characteristic_count = (uint8_t)(sizeof(m_custom_characteristics) / sizeof(m_custom_characteristics[0])),
        .service_handle = 0U,
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
  log_printf("BLE advertising started\n");
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

static const char *ble_phy_name(uint8_t phy)
{
  switch (phy)
  {
  case BLE_GAP_PHY_1MBPS:
    return "1M";

  case BLE_GAP_PHY_2MBPS:
    return "2M";

  default:
    return "unknown";
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
  if (ble_indicate_characteristic(&m_custom_characteristics[0]))
  {
    log_printf("BLE GATT: indicated counter=%u\n", (unsigned int)m_counter_char_value);
    return;
  }

  if (ble_notify_characteristic(&m_custom_characteristics[0]))
  {
    log_printf("BLE GATT: notified counter=%u\n", (unsigned int)m_counter_char_value);
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
    log_printf("BLE LINK: supervision timeout\n");
    return;

  case BLE_GAP_EVT_CONN_UPDATE_IND:
    log_printf("BLE GAP: connection updated, interval=%d ms latency=%d timeout=%d ms\n",
               (int)p_evt->params.gap.conn_interval_ms,
               (int)p_evt->params.gap.slave_latency,
               (int)p_evt->params.gap.supervision_timeout_ms);
    return;

  case BLE_GAP_EVT_PHY_UPDATE_IND:
    log_printf("BLE GAP: PHY updated, tx=%s rx=%s\n",
               ble_phy_name(p_evt->params.gap.tx_phy),
               ble_phy_name(p_evt->params.gap.rx_phy));
    return;

  case BLE_GAP_EVT_TERMINATE_IND:
    log_printf("BLE LINK: terminate indication received\n");
    return;

  case BLE_GATT_EVT_MTU_EXCHANGE:
    log_printf("BLE ATT: MTU exchange req=%u rsp=%u effective=%u\n",
               (unsigned int)p_evt->params.gatt.requested_mtu,
               (unsigned int)p_evt->params.gatt.response_mtu,
               (unsigned int)p_evt->params.gatt.effective_mtu);
    return;

  case BLE_GAP_EVT_CONNECTED:
    m_counter_char_value = 0U;
    m_custom_characteristics[0].value_len = sizeof(m_counter_char_value);
    ble_state_set(true);
    err = app_timer_start(m_measurement_timer_id, timer_ticks_clamped(1000U), NULL);
    APP_ERROR_CHECK(err);
    log_printf("BLE GAP: connected, interval=%d ms timeout=%d ms\n",
               (int)p_evt->params.gap.conn_interval_ms,
               (int)p_evt->params.gap.supervision_timeout_ms);
    return;

  case BLE_GAP_EVT_DISCONNECTED:
    timer_stop_if_started(m_measurement_timer_id);
    log_printf("BLE GAP: disconnected\n");
    start_advertising();
    return;

  default:
    return;
  }
}

static void counter_char_evt_handler(const ble_gatt_char_evt_t *p_evt)
{
  if (p_evt == NULL)
  {
    return;
  }

  if ((p_evt->evt_type == BLE_GATT_CHAR_EVT_NOTIFY_ENABLED) ||
      (p_evt->evt_type == BLE_GATT_CHAR_EVT_NOTIFY_DISABLED))
  {
    log_printf("BLE GATT: counter notifications %s\n", p_evt->notifications_enabled ? "enabled" : "disabled");
    return;
  }

  if ((p_evt->evt_type == BLE_GATT_CHAR_EVT_INDICATE_ENABLED) ||
      (p_evt->evt_type == BLE_GATT_CHAR_EVT_INDICATE_DISABLED))
  {
    log_printf("BLE GATT: counter indications %s\n", p_evt->indications_enabled ? "enabled" : "disabled");
  }
}

static void text_char_evt_handler(const ble_gatt_char_evt_t *p_evt)
{
  char written_text[BLE_GATT_MAX_VALUE_LEN + 1U];
  uint16_t copy_len;

  if ((p_evt == NULL) || (p_evt->evt_type != BLE_GATT_CHAR_EVT_WRITE))
  {
    return;
  }

  copy_len = p_evt->p_characteristic->value_len;
  if (copy_len > BLE_GATT_MAX_VALUE_LEN)
  {
    copy_len = BLE_GATT_MAX_VALUE_LEN;
  }

  if ((copy_len > 0U) && (p_evt->p_characteristic->p_value != NULL))
  {
    (void)memcpy(written_text, p_evt->p_characteristic->p_value, copy_len);
  }
  written_text[copy_len] = '\0';

  log_printf("BLE GATT: write text=\"%s\"\n", written_text);
}

int main(void)
{
  ret_code_t err;
  bsp_board_init(BSP_INIT_LEDS);
  clock_init();
  log_init();

  ble_stack_init();
  ble_register_evt_handler(ble_evt_handler);
  ble_gap_set_device_name(m_dev_name);
  ble_gap_set_conn_params(&m_gap_conn_params);
  ble_uuid_set_vendor_base(m_custom_uuid_base);
  ble_adv_init(&m_adv_config);
  APP_ERROR_CHECK_BOOL(ble_gatt_server_init(m_custom_services,
                                            (uint8_t)(sizeof(m_custom_services) / sizeof(m_custom_services[0]))));

  err = app_timer_create(&m_measurement_timer_id,
                         APP_TIMER_MODE_REPEATED,
                         measurement_timer_handler);
  APP_ERROR_CHECK(err);

  start_advertising();

  while (1)
  {
    log_idle();
  }
}
