#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "boards.h"
#include "nrf_ble.h"
#include "nrf_drv_clock.h"
#include "usb_log.h"

#define BLE_SCANNING_LED_IDX BSP_BOARD_LED_0
#if (LEDS_NUMBER > 1)
#define BLE_CONNECTED_LED_IDX BSP_BOARD_LED_1
#else
#define BLE_CONNECTED_LED_IDX BSP_BOARD_LED_0
#endif

#define UUID_STR_CHARS 40U
#define GATT_DISCOVERY_MAX_CHARACTERISTICS 8U
#define BLE_UUID_CCCD 0x2902U

typedef struct
{
    uint16_t declaration_handle;
    uint16_t value_handle;
} discovered_characteristic_t;

typedef struct
{
    bool service_found;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    discovered_characteristic_t characteristics[GATT_DISCOVERY_MAX_CHARACTERISTICS];
    uint8_t characteristic_count;
    bool target_characteristic_found;
    uint16_t target_characteristic_decl_handle;
    uint16_t target_characteristic_value_handle;
    uint16_t target_cccd_handle;
} gatt_demo_state_t;

static void gap_evt_handler(const ble_gap_evt_t *p_evt);
static void gatt_client_evt_handler(const ble_gatt_client_evt_t *p_evt);
static void ble_state_set(bool connected);
static void ble_addr_format(const ble_gap_addr_t *p_addr, char *p_buffer, size_t buffer_size);
static void gatt_client_uuid_format(const ble_uuid_t *p_uuid, char *p_buffer, size_t buffer_size);
static void gatt_discovery_reset(void);
static void gatt_discovery_start(void);
static bool gatt_discovery_start_characteristics(void);
static bool gatt_discovery_start_descriptors(void);
static bool gatt_subscription_enable_notifications(void);
static void start_target_scan(void);

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

static const ble_scan_config_t m_scan_config = {
    .interval_ms = 100U,
    .window_ms = 50U,
};

static const ble_gap_conn_params_t m_gap_conn_params = {
    .min_conn_interval_1p25ms = MS_TO_1P25MS_UNITS(20U),
    .max_conn_interval_1p25ms = MS_TO_1P25MS_UNITS(30U),
    .slave_latency = 0U,
    .supervision_timeout_10ms = MS_TO_10MS_UNITS(1500U),
};

static const ble_uuid_t m_cccd_uuid = BLE_UUID_SIG16_INIT(BLE_UUID_CCCD);

static ble_gap_scan_filter_t m_target_filter;
static ble_uuid_t m_target_service_uuid;
static ble_uuid_t m_target_notify_char_uuid;
static gatt_demo_state_t m_gatt_demo;

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
        bsp_board_led_off(BLE_SCANNING_LED_IDX);
        bsp_board_led_on(BLE_CONNECTED_LED_IDX);
        return;
    }

    bsp_board_led_on(BLE_SCANNING_LED_IDX);
    bsp_board_led_off(BLE_CONNECTED_LED_IDX);
}

static void ble_addr_format(const ble_gap_addr_t *p_addr, char *p_buffer, size_t buffer_size)
{
    if ((p_addr == NULL) || (p_buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    (void)snprintf(p_buffer,
                   buffer_size,
                   "%02X:%02X:%02X:%02X:%02X:%02X %s",
                   p_addr->addr[5],
                   p_addr->addr[4],
                   p_addr->addr[3],
                   p_addr->addr[2],
                   p_addr->addr[1],
                   p_addr->addr[0],
                   p_addr->addr_is_random ? "random" : "public");
}

static void gatt_client_uuid_format(const ble_uuid_t *p_uuid, char *p_buffer, size_t buffer_size)
{
    if ((p_uuid == NULL) || (p_buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    if ((p_uuid->type == BLE_UUID_TYPE_SIG_16) ||
        (p_uuid->type == BLE_UUID_TYPE_VENDOR_16))
    {
        (void)snprintf(p_buffer, buffer_size, "0x%04X", p_uuid->value.uuid16);
        return;
    }

    if (p_uuid->type == BLE_UUID_TYPE_RAW_128)
    {
        (void)snprintf(p_buffer,
                       buffer_size,
                       "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                       p_uuid->value.uuid128[15],
                       p_uuid->value.uuid128[14],
                       p_uuid->value.uuid128[13],
                       p_uuid->value.uuid128[12],
                       p_uuid->value.uuid128[11],
                       p_uuid->value.uuid128[10],
                       p_uuid->value.uuid128[9],
                       p_uuid->value.uuid128[8],
                       p_uuid->value.uuid128[7],
                       p_uuid->value.uuid128[6],
                       p_uuid->value.uuid128[5],
                       p_uuid->value.uuid128[4],
                       p_uuid->value.uuid128[3],
                       p_uuid->value.uuid128[2],
                       p_uuid->value.uuid128[1],
                       p_uuid->value.uuid128[0]);
        return;
    }

    (void)snprintf(p_buffer, buffer_size, "type=%u", (unsigned int)p_uuid->type);
}

static void demo_vendor_uuid_build(uint16_t short_uuid, uint8_t uuid128[BLE_UUID128_LEN])
{
    (void)memcpy(uuid128, m_custom_uuid_base, BLE_UUID128_LEN);
    uuid128[12] = (uint8_t)(short_uuid & 0xFFU);
    uuid128[13] = (uint8_t)((short_uuid >> 8) & 0xFFU);
}

static void demo_target_init(void)
{
    (void)memset(&m_target_filter, 0, sizeof(m_target_filter));
    m_target_filter.match_service_uuid128 = true;
    demo_vendor_uuid_build(0xFFF0U, m_target_filter.service_uuid128);

    m_target_service_uuid = (ble_uuid_t)BLE_UUID_NONE_INIT;
    m_target_service_uuid.type = BLE_UUID_TYPE_RAW_128;
    demo_vendor_uuid_build(0xFFF0U, m_target_service_uuid.value.uuid128);

    m_target_notify_char_uuid = (ble_uuid_t)BLE_UUID_NONE_INIT;
    m_target_notify_char_uuid.type = BLE_UUID_TYPE_RAW_128;
    demo_vendor_uuid_build(0xFFF1U, m_target_notify_char_uuid.value.uuid128);
}

static bool gatt_client_uuid_matches(const ble_uuid_t *p_lhs, const ble_uuid_t *p_rhs)
{
    if ((p_lhs == NULL) || (p_rhs == NULL))
    {
        return false;
    }

    if ((p_lhs->type == BLE_UUID_TYPE_RAW_128) &&
        (p_rhs->type == BLE_UUID_TYPE_RAW_128))
    {
        return memcmp(p_lhs->value.uuid128, p_rhs->value.uuid128, BLE_UUID128_LEN) == 0;
    }

    if ((p_lhs->type != BLE_UUID_TYPE_NONE) &&
        (p_rhs->type != BLE_UUID_TYPE_NONE) &&
        (p_lhs->type != BLE_UUID_TYPE_RAW_128) &&
        (p_rhs->type != BLE_UUID_TYPE_RAW_128))
    {
        return p_lhs->value.uuid16 == p_rhs->value.uuid16;
    }

    return false;
}

static void gatt_discovery_reset(void)
{
    (void)memset(&m_gatt_demo, 0, sizeof(m_gatt_demo));
}

static uint16_t gatt_target_characteristic_end_handle_get(void)
{
    uint16_t end_handle = m_gatt_demo.service_end_handle;
    uint8_t i;

    for (i = 0U; i < m_gatt_demo.characteristic_count; i++)
    {
        uint16_t next_decl_handle = m_gatt_demo.characteristics[i].declaration_handle;

        if ((next_decl_handle > m_gatt_demo.target_characteristic_decl_handle) &&
            (next_decl_handle <= end_handle))
        {
            end_handle = (uint16_t)(next_decl_handle - 1U);
        }
    }

    return end_handle;
}

static void gatt_discovery_start(void)
{
    char uuid_str[UUID_STR_CHARS];

    gatt_discovery_reset();
    gatt_client_uuid_format(&m_target_service_uuid, uuid_str, sizeof(uuid_str));

    if (ble_gatt_client_discover_primary_services_by_uuid(&m_target_service_uuid))
    {
        log_printf("BLE GATTC svc discover start\n");
        log_printf("svc uuid=%s\n", uuid_str);
        return;
    }

    log_printf("BLE GATTC svc discover failed\n");
}

static bool gatt_discovery_start_characteristics(void)
{
    if (!m_gatt_demo.service_found)
    {
        log_printf("BLE GATTC target svc missing\n");
        return false;
    }

    if (!ble_gatt_client_discover_characteristics((uint16_t)(m_gatt_demo.service_start_handle + 1U),
                                                  m_gatt_demo.service_end_handle))
    {
        log_printf("BLE GATTC char discover failed\n");
        return false;
    }

    log_printf("BLE GATTC char discover start\n");
    log_printf("range=0x%04X-0x%04X\n",
               (unsigned int)(m_gatt_demo.service_start_handle + 1U),
               (unsigned int)m_gatt_demo.service_end_handle);
    return true;
}

static bool gatt_discovery_start_descriptors(void)
{
    uint16_t descriptor_start;
    uint16_t descriptor_end;

    if (!m_gatt_demo.target_characteristic_found)
    {
        log_printf("BLE GATTC target char missing\n");
        return false;
    }

    descriptor_end = gatt_target_characteristic_end_handle_get();
    descriptor_start = (uint16_t)(m_gatt_demo.target_characteristic_value_handle + 1U);
    if (descriptor_start > descriptor_end)
    {
        log_printf("BLE GATTC no char descriptors\n");
        return false;
    }

    if (!ble_gatt_client_discover_descriptors(descriptor_start, descriptor_end))
    {
        log_printf("BLE GATTC desc discover failed\n");
        return false;
    }

    log_printf("BLE GATTC desc discover start\n");
    log_printf("range=0x%04X-0x%04X\n",
               (unsigned int)descriptor_start,
               (unsigned int)descriptor_end);
    return true;
}

static bool gatt_subscription_enable_notifications(void)
{
    if (m_gatt_demo.target_cccd_handle == 0U)
    {
        log_printf("BLE GATTC CCCD missing\n");
        return false;
    }

    if (!ble_gatt_client_write_cccd(m_gatt_demo.target_cccd_handle, true, false))
    {
        log_printf("BLE GATTC CCCD write failed\n");
        return false;
    }

    log_printf("BLE GATTC enable notif\n");
    log_printf("cccd=0x%04X\n", (unsigned int)m_gatt_demo.target_cccd_handle);
    return true;
}

static void start_target_scan(void)
{
    char uuid_str[UUID_STR_CHARS];

    ble_state_set(false);
    gatt_discovery_reset();
    gatt_client_uuid_format(&m_target_service_uuid, uuid_str, sizeof(uuid_str));

    if (ble_gap_set_scan_filter(&m_target_filter))
    {
        ble_start_scanning();
        log_printf("BLE central scan start\n");
        log_printf("svc uuid=%s\n", uuid_str);
        return;
    }

    log_printf("BLE central scan failed\n");
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

static void gatt_client_evt_handler(const ble_gatt_client_evt_t *p_evt)
{
    char uuid_str[UUID_STR_CHARS];

    if (p_evt == NULL)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
    case BLE_GATT_CLIENT_EVT_SERVICE_DISCOVERED:
        gatt_client_uuid_format(&p_evt->params.service.uuid, uuid_str, sizeof(uuid_str));
        log_printf("BLE GATTC svc 0x%04X-0x%04X\n",
                   (unsigned int)p_evt->params.service.start_handle,
                   (unsigned int)p_evt->params.service.end_handle);
        log_printf("uuid=%s\n", uuid_str);

        if (!m_gatt_demo.service_found)
        {
            m_gatt_demo.service_found = true;
            m_gatt_demo.service_start_handle = p_evt->params.service.start_handle;
            m_gatt_demo.service_end_handle = p_evt->params.service.end_handle;
        }
        return;

    case BLE_GATT_CLIENT_EVT_CHARACTERISTIC_DISCOVERED:
        gatt_client_uuid_format(&p_evt->params.characteristic.uuid, uuid_str, sizeof(uuid_str));
        log_printf("BLE GATTC char decl=0x%04X\n",
                   (unsigned int)p_evt->params.characteristic.declaration_handle);
        log_printf("val=0x%04X prop=0x%02X\n",
                   (unsigned int)p_evt->params.characteristic.value_handle,
                   (unsigned int)p_evt->params.characteristic.properties);
        log_printf("uuid=%s\n", uuid_str);

        if (m_gatt_demo.characteristic_count < GATT_DISCOVERY_MAX_CHARACTERISTICS)
        {
            discovered_characteristic_t *p_char =
                &m_gatt_demo.characteristics[m_gatt_demo.characteristic_count];

            p_char->declaration_handle = p_evt->params.characteristic.declaration_handle;
            p_char->value_handle = p_evt->params.characteristic.value_handle;
            m_gatt_demo.characteristic_count++;
        }

        if (!m_gatt_demo.target_characteristic_found &&
            gatt_client_uuid_matches(&p_evt->params.characteristic.uuid, &m_target_notify_char_uuid))
        {
            m_gatt_demo.target_characteristic_found = true;
            m_gatt_demo.target_characteristic_decl_handle = p_evt->params.characteristic.declaration_handle;
            m_gatt_demo.target_characteristic_value_handle = p_evt->params.characteristic.value_handle;
        }
        return;

    case BLE_GATT_CLIENT_EVT_DESCRIPTOR_DISCOVERED:
        gatt_client_uuid_format(&p_evt->params.descriptor.uuid, uuid_str, sizeof(uuid_str));
        log_printf("BLE GATTC desc h=0x%04X\n",
                   (unsigned int)p_evt->params.descriptor.handle);
        log_printf("uuid=%s\n", uuid_str);

        if ((m_gatt_demo.target_cccd_handle == 0U) &&
            gatt_client_uuid_matches(&p_evt->params.descriptor.uuid, &m_cccd_uuid))
        {
            m_gatt_demo.target_cccd_handle = p_evt->params.descriptor.handle;
        }
        return;

    case BLE_GATT_CLIENT_EVT_WRITE_RSP:
        log_printf("BLE GATTC wr rsp h=0x%04X\n",
                   (unsigned int)p_evt->params.write.handle);
        return;

    case BLE_GATT_CLIENT_EVT_NOTIFICATION:
        log_printf("BLE GATTC notif h=0x%04X\n",
                   (unsigned int)p_evt->params.hvx.handle);
        log_printf("len=%u\n", (unsigned int)p_evt->params.hvx.len);
        if ((p_evt->params.hvx.handle == m_gatt_demo.target_characteristic_value_handle) &&
            (p_evt->params.hvx.len > 0U))
        {
            log_printf("counter=%u\n", (unsigned int)p_evt->params.hvx.data[0]);
        }
        return;

    case BLE_GATT_CLIENT_EVT_INDICATION:
        log_printf("BLE GATTC indic h=0x%04X\n",
                   (unsigned int)p_evt->params.hvx.handle);
        log_printf("len=%u\n", (unsigned int)p_evt->params.hvx.len);
        return;

    case BLE_GATT_CLIENT_EVT_ERROR_RSP:
        log_printf("BLE GATTC err proc=%u op=0x%02X\n",
                   (unsigned int)p_evt->params.error.procedure,
                   (unsigned int)p_evt->params.error.request_opcode);
        log_printf("handle=0x%04X err=0x%02X\n",
                   (unsigned int)p_evt->params.error.handle,
                   (unsigned int)p_evt->params.error.error_code);
        return;

    case BLE_GATT_CLIENT_EVT_PROCEDURE_COMPLETE:
        if (p_evt->params.complete.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_PRIMARY_SERVICES_BY_UUID)
        {
            log_printf("BLE GATTC svc discover done\n");
            (void)gatt_discovery_start_characteristics();
            return;
        }

        if (p_evt->params.complete.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_CHARACTERISTICS)
        {
            log_printf("BLE GATTC char discover done\n");
            (void)gatt_discovery_start_descriptors();
            return;
        }

        if (p_evt->params.complete.procedure == BLE_GATT_CLIENT_PROC_DISCOVER_DESCRIPTORS)
        {
            log_printf("BLE GATTC desc discover done\n");
            (void)gatt_subscription_enable_notifications();
            return;
        }

        if (p_evt->params.complete.procedure == BLE_GATT_CLIENT_PROC_WRITE_CCCD)
        {
            log_printf("BLE GATTC notifications on\n");
            return;
        }

        return;

    case BLE_GATT_CLIENT_EVT_MTU_EXCHANGED:
        log_printf("BLE GATTC mtu req=%u neg=%u\n",
                   (unsigned int)p_evt->params.mtu.requested_mtu,
                   (unsigned int)p_evt->params.mtu.negotiated_mtu);
        return;

    case BLE_GATT_CLIENT_EVT_READ_RSP:
    default:
        return;
    }
}

static void gap_evt_handler(const ble_gap_evt_t *p_evt)
{
    char addr_str[32];

    if (p_evt == NULL)
    {
        return;
    }

    switch (p_evt->evt_type)
    {
    case BLE_GAP_EVT_CONNECTED:
        ble_state_set(true);
        ble_addr_format(&p_evt->params.peer_addr, addr_str, sizeof(addr_str));
        log_printf("BLE GAP connected\n");
        log_printf("peer=%s\n", addr_str);
        log_printf("int=%ums lat=%u to=%ums\n",
                   (unsigned int)p_evt->params.conn_interval_ms,
                   (unsigned int)p_evt->params.slave_latency,
                   (unsigned int)p_evt->params.supervision_timeout_ms);
        gatt_discovery_start();
        return;

    case BLE_GAP_EVT_DISCONNECTED:
        log_printf("BLE GAP disconnected\n");
        start_target_scan();
        return;

    case BLE_GAP_EVT_SUPERVISION_TIMEOUT:
        log_printf("BLE LINK sup timeout\n");
        return;

    case BLE_GAP_EVT_CONN_UPDATE_IND:
        log_printf("BLE GAP conn updated\n");
        log_printf("int=%ums lat=%u to=%ums\n",
                   (unsigned int)p_evt->params.conn_interval_ms,
                   (unsigned int)p_evt->params.slave_latency,
                   (unsigned int)p_evt->params.supervision_timeout_ms);
        return;

    case BLE_GAP_EVT_PHY_UPDATE_IND:
        log_printf("BLE GAP PHY tx=%s rx=%s\n",
                   ble_phy_name(p_evt->params.tx_phy),
                   ble_phy_name(p_evt->params.rx_phy));
        return;

    case BLE_GAP_EVT_FEATURE_EXCHANGED:
        log_printf("BLE GAP features %02X %02X\n",
                   (unsigned int)p_evt->params.features[0],
                   (unsigned int)p_evt->params.features[1]);
        return;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATED:
        log_printf("BLE GAP data len tx=%u rx=%u\n",
                   (unsigned int)p_evt->params.max_tx_octets,
                   (unsigned int)p_evt->params.max_rx_octets);
        return;

    case BLE_GAP_EVT_CONTROL_PROCEDURE_UNSUPPORTED:
        log_printf("BLE GAP ctrl unsup proc=%u op=0x%02X\n",
                   (unsigned int)p_evt->params.procedure,
                   (unsigned int)p_evt->params.unsupported_opcode);
        return;

    case BLE_GAP_EVT_TERMINATE_IND:
        log_printf("BLE LINK terminate\n");
        return;

    default:
        return;
    }
}

int main(void)
{
    char uuid_str[UUID_STR_CHARS];

    bsp_board_init(BSP_INIT_LEDS);
    clock_init();
    log_init();
    demo_target_init();

    ble_stack_init(BLE_GAP_ROLE_CENTRAL);
    ble_gap_register_evt_handler(gap_evt_handler);
    ble_gatt_client_register_evt_handler(gatt_client_evt_handler);
    ble_gap_set_conn_params(&m_gap_conn_params);
    ble_uuid_set_vendor_base(m_custom_uuid_base);
    ble_scan_init(&m_scan_config);

    gatt_client_uuid_format(&m_target_service_uuid, uuid_str, sizeof(uuid_str));
    log_printf("BLE central demo\n");
    log_printf("target svc=%s\n", uuid_str);
    start_target_scan();

    while (1)
    {
        log_idle();
    }
}
