# nRF_BLE_Stack

Minimal BLE peripheral stack for nRF SoCs.

This repository contains a compact educational BLE stack focused on clarity,
small code size, and readable control flow. It implements the pieces needed for
an application-defined BLE peripheral: advertising, scan-response handling for
active scanners, connection handling, ATT/GATT services and characteristics,
deferred application callbacks, passive data length extension, and delayed
connection parameter update requests.

The stack is intentionally small enough to read end to end. Public API,
controller logic, ATT/GATT handling, and radio access are kept in separate
layers so packet flow is easy to follow in code.

## Current Scope

- Peripheral role only
- Advertising with configurable name, flags, TX power, interval, and one
  included service UUID
- Minimal legacy `SCAN_RSP` support for active scanners that send `SCAN_REQ`
  before showing or connecting
- Standard 16-bit SIG UUIDs and vendor UUIDs expanded from one registered
  128-bit base UUID
- Runtime registration of custom GATT services and characteristics
- GATT write and notification-state callbacks
- Deferred BLE events through low-priority software interrupt
- Passive data length extension support
- Passive LE 2M PHY update support
- ATT MTU negotiation up to 247 bytes when the peer performs the LL length
  procedure
- Legacy advertising validation of both `SCAN_REQ` and `CONNECT_REQ` against
  the local advertiser address and address type
- Internal delayed connection parameter update request after connect
- One RX and one TX exchange per connection interval
- Single pending connected TX slot shared by notifications, ATT responses, and
  signaling PDUs

## Repository Layout

- `stack/include/nrf_ble.h`
  Public BLE stack API
- `stack/gatt/ble_gatt_server.h`
  Public GATT and UUID types
- `stack/core/`
  Stack entry points, runtime state, controller flow, UUID helpers, and
  deferred event delivery
- `stack/gatt/`
  ATT/GATT database construction and ATT request handling
- `stack/radio/`
  nRF radio peripheral abstraction used by the controller
- `examples/custom_ble_stack_demo/`
  Example peripheral application using the stack
- `external/nrf5-sdk/`
  nRF5 SDK Git submodule used by the example build
- `BLE_STACK_WALKTHROUGH.pdf`
  Architecture and packet-flow walkthrough

## Public API

Main application-facing entry points:

- `ble_stack_init()`
- `ble_register_evt_handler()`
- `ble_adv_init()`
- `ble_gap_set_device_name()`
- `ble_gap_set_conn_params()`
- `ble_gap_update_conn_params()`
- `ble_uuid_set_vendor_base()`
- `ble_gatt_server_init()`
- `ble_start_advertising()`
- `ble_notify_characteristic()`
- `ble_is_connected()`

See [nrf_ble.h](stack/include/nrf_ble.h) and
[ble_gatt_server.h](stack/gatt/ble_gatt_server.h) for the full public
interface.

## Architecture At A Glance

- `ble_stack.c`
  Public API wrapper layer. Stores host configuration, UUID base, delayed
  connection parameter update timer, and notification helpers.
- `ble_runtime.c`
  Shared runtime state, small utilities, identity address generation, and
  deferred event delivery through `SWI1_EGU1`.
- `ble_controller.c`
  Advertising, scan-request/connect-request validation and handling,
  connection-event timing, LL control, retransmission behavior, DLE parameter
  tracking, and ATT/L2CAP packet transport.
- `ble_gatt_server.c`
  ATT database construction, 16-bit and vendor-base UUID expansion for
  discovery responses, ATT request handling, CCCD tracking, MTU negotiation,
  and notification building.
- `radio_driver.c`
  Direct `NRF_RADIO` access hidden behind a small abstraction.

## UUID Model

- Standard Bluetooth SIG UUIDs are represented as plain 16-bit UUIDs.
- Custom UUIDs are represented as vendor 16-bit values plus one stack-wide
  128-bit base UUID set with `ble_uuid_set_vendor_base()`.
- The stack expands vendor UUIDs into the final 128-bit little-endian UUID
  bytes internally when building advertising data, the ATT database, and ATT
  discovery responses.
- This keeps application service and characteristic definitions compact while
  still exposing full 128-bit UUIDs over the air.

## Event Model

- Stack-level BLE events are delivered through one callback registered with
  `ble_register_evt_handler()`.
- `ble_evt_t` groups event payloads under `params.gap` and `params.gatt`.
- Current stack-level events are:
  - `BLE_GAP_EVT_CONNECTED`
  - `BLE_GAP_EVT_DISCONNECTED`
  - `BLE_GAP_EVT_SUPERVISION_TIMEOUT`
  - `BLE_GAP_EVT_CONN_UPDATE_IND`
  - `BLE_GAP_EVT_PHY_UPDATE_IND`
  - `BLE_GAP_EVT_TERMINATE_IND`
  - `BLE_GATT_EVT_MTU_EXCHANGE`
- GAP events also expose the current `tx_phy` and `rx_phy` so applications can
  log or react when a PHY update takes effect.
- Characteristic-specific events are delivered through each characteristic's
  `evt_handler`.
- `ble_gatt_char_evt_t` carries the event type plus `p_characteristic`. For
  write events, applications read the current value from
  `p_evt->p_characteristic->p_value` and `p_evt->p_characteristic->value_len`.
- Both stack-level and characteristic-level callbacks are deferred to
  low-priority software interrupt context instead of being called directly from
  the radio ISR path.

## Example

The repository includes a working example application in
`examples/custom_ble_stack_demo`.

Before building the example, initialize the SDK submodule:

```sh
git submodule update --init --recursive
```

Build the example with:

```sh
make -C examples/custom_ble_stack_demo -j4
```

Notes:

- The bundled example is written for the nRF52840 dongle and uses the included
  `support/usb_log.c` backend over the dongle's built-in USB interface.
- For `BOARD_PCA10059`, the example uses `bsp_board_init(BSP_INIT_LEDS)` so the
  SDK handles the dongle `REGOUT0` LED-voltage setup.
- The example uses the SDK clock driver for LFCLK and HFCLK startup.
- Because the USB CDC logger needs its event queue serviced in thread context,
  the bundled example calls `log_idle()` in the main loop instead of sleeping
  with `__WFE()`.

## Typical Usage

```c
static const uint8_t custom_uuid_base[BLE_UUID128_LEN] = {
    0x52, 0xD0, 0x4F, 0x36, 0x7E, 0x85, 0x74, 0x1C,
    0xA6, 0x8F, 0x4E, 0x7A, 0x00, 0x00, 0x00, 0x00,
};

static void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_GAP_EVT_CONNECTED:
        (void)p_evt->params.gap.conn_interval_ms;
        break;
    case BLE_GAP_EVT_CONN_UPDATE_IND:
        (void)p_evt->params.gap.slave_latency;
        break;
    case BLE_GATT_EVT_MTU_EXCHANGE:
        (void)p_evt->params.gatt.effective_mtu;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        ble_start_advertising();
        break;
    default:
        break;
    }
}

int main(void)
{
    ble_stack_init();
    ble_register_evt_handler(ble_evt_handler);
    ble_gap_set_device_name("nrf-ble");
    ble_gap_set_conn_params(&(ble_gap_conn_params_t){
        .min_conn_interval_1p25ms = MS_TO_1P25MS_UNITS(30U),
        .max_conn_interval_1p25ms = MS_TO_1P25MS_UNITS(30U),
        .slave_latency = 0U,
        .supervision_timeout_10ms = MS_TO_10MS_UNITS(720U),
    });
    ble_uuid_set_vendor_base(custom_uuid_base);
    ble_adv_init(&adv_config);
    APP_ERROR_CHECK_BOOL(ble_gatt_server_init(services, service_count));
    ble_start_advertising();

    for (;;)
    {
        log_idle();
    }
}
```

## Runtime Flow Summary

The normal peripheral flow is:

1. `ble_stack_init()` brings up shared state, deferred events, controller
   runtime, and the delayed connection parameter update timer.
2. `ble_gap_set_device_name()` stores the local name used by both advertising
   and the GAP Device Name attribute.
3. `ble_gap_set_conn_params()` stores preferred connection parameters that can
   later be requested by the stack.
4. `ble_uuid_set_vendor_base()` stores the one custom 128-bit base UUID used by
   vendor 16-bit UUIDs.
5. `ble_adv_init()` stores advertising parameters.
6. `ble_gatt_server_init()` builds the ATT database from the application's
   service table.
7. `ble_start_advertising()` starts repeated advertising events on channels 37,
   38, and 39.
8. After each advertising transmission, the controller opens a short RX window
   and listens for a targeted `SCAN_REQ` or `CONNECT_REQ`.
9. When a valid `SCAN_REQ` is received, the controller sends a minimal
   `SCAN_RSP` that carries the advertiser address so active scanners can keep
   the advertising event visible without adding extra payload turnaround work.
10. When a `CONNECT_REQ` that targets the local advertiser address and address
    type is received, the controller switches to connected mode, starts
    connection-event timing with `TIMER2`, and begins using the data channel
    map from the request.
11. If the peer performs the LL length procedure, the controller updates the
   usable LL payload size and ATT MTU negotiation can grow up to 247 bytes.
12. If the peer performs the LL PHY procedure, the controller advertises
    `1M | 2M` support, schedules the selected PHYs for the requested instant,
    and applies the RX/TX PHY change at the start of the matching connection
    event.
13. About six seconds after connect, the stack sends an L2CAP Connection
    Parameter Update Request if preferred parameters were configured.
14. Each connection interval is handled as one RX and one TX exchange. Any ATT
    response, notification, or signaling PDU generated from the received packet
    is queued for the next connection event.
15. Stack-level BLE events and characteristic callbacks are delivered later
    from `SWI1_EGU1_IRQHandler()`.

## Design Notes

- Services and characteristics are provided by the application instead of being
  hardcoded in the stack.
- BLE stack events are delivered through a unified `ble_evt_t`.
- GATT characteristic events remain per-characteristic callbacks.
- Characteristic values and current lengths live directly in
  `ble_gatt_characteristic_t`.
- `ble_controller.c` owns BLE packet flow, timing, and LL control handling.
- The controller only accepts legacy `SCAN_REQ` and `CONNECT_REQ` packets whose
  advertiser address and `RxAdd` bit match the current advertising identity.
- Scan responses are intentionally minimal so the advertising RX->TX turnaround
  stays simple and reliable across scanners that actively probe advertisements.
- LE PHY updates stay within the same simple event model by configuring the
  event RX PHY before listening and the TX PHY just before responding.
- `radio_driver.c` owns direct `NRF_RADIO` access.
- The connected data path intentionally uses a simple one-RX / one-TX-per-
  interval model.
- Notifications, ATT responses, and signaling PDUs share one pending connected
  TX slot to keep the controller flow small and traceable.

## Limitations

- No central role or scanning support
- No L2CAP fragmentation or reassembly
- No security, pairing, or bonding
- No long writes or prepare/execute write support
- Data length extension is passive only; the stack responds to the peer's LL
  length procedure but does not initiate it
- PHY update is passive only; the stack responds to the peer's LL PHY
  procedure but does not initiate it
- The delayed connection parameter update request is one-shot and uses the same
  single pending TX slot as other outgoing connected traffic

## Documentation

For a detailed code and packet-flow explanation, read:

- `BLE_STACK_WALKTHROUGH.pdf`

## License

MIT. See [LICENSE](LICENSE).
