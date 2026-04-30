# nrf-ble-stack

Minimal BLE stack for nRF SoCs.

This repository contains a compact educational BLE stack focused on clarity,
small code size, and readable control flow. It implements the pieces needed for
an application-defined BLE peripheral or central: advertising, passive
scanning, central connection initiation, connection handling, ATT/GATT
services and characteristics, GATT client procedures, deferred application
callbacks, automatic central link-layer feature exchange, automatic central
data length and PHY updates, and application-driven MTU and connection
parameter procedures.

The stack is intentionally small enough to read end to end. Public API,
controller logic, ATT/GATT handling, and radio access are kept in separate
layers so packet flow is easy to follow in code.

## Current Scope

- Peripheral and central role support
- One role active at a time
- Advertising with configurable name, flags, TX power, interval, and one
  included service UUID
- Passive scanning with scan report callbacks and optional auto-connect filter
- Minimal legacy `SCAN_RSP` support for active scanners that send `SCAN_REQ`
  before showing or connecting
- Standard 16-bit SIG UUIDs and vendor UUIDs expanded from one registered
  128-bit base UUID
- Runtime registration of custom GATT services and characteristics
- GATT client procedures for MTU exchange, discovery, read, write, and CCCD updates
- GATT write and notification-state callbacks
- Deferred BLE events through low-priority software interrupt
- Automatic central feature exchange after connect
- Automatic central data length update after connect
- Automatic central LE 1M/2M PHY update after connect
- ATT MTU negotiation up to 247 bytes
- Legacy advertising validation of both `SCAN_REQ` and `CONNECT_REQ` against
  the local advertiser address and address type
- Application-driven peripheral connection parameter update requests and
  central connection update indications
- Connection-event timing re-anchored from the central packet address using
  `TIMER0` plus fixed PPI capture for better interoperability with active
  central implementations
- One RX and one TX exchange per connection interval
- Single pending connected TX slot shared by notifications, ATT responses, and
  signaling PDUs

## Repository Layout

- `stack/include/nrf_ble.h`
  Public BLE stack API
- `stack/include/ble_gatt_server.h`
  Public GATT server types and APIs
- `stack/include/ble_gatt_client.h`
  Public GATT client types and APIs
- `stack/core/`
  Stack entry points, runtime state, UUID helpers, and
  deferred event delivery
- `stack/controller/`
  Shared, central, and peripheral controller/link-layer implementation
- `stack/include/ble_att_defs.h`
  Shared ATT protocol definitions
- `stack/host/gap/`
  GAP-facing host APIs
- `stack/host/gatt/`
  GATT client/server implementation and public GATT helpers
- `stack/radio/`
  nRF radio peripheral abstraction used by the controller
- `examples/peripheral_demo/`
  Example peripheral application using the stack
- `examples/central_demo/`
  Minimal central application that scans, connects, starts GATT discovery on
  `BLE_GAP_EVT_CONNECTED`, and subscribes while the stack performs automatic
  central LL setup in the background
- `external/nrf5-sdk/`
  nRF5 SDK Git submodule used by the example build
- `README.md`
  Architecture and packet-flow walkthrough

## Public API

Main application-facing entry points:

- `ble_stack_init()`
- `ble_gap_register_evt_handler()`
- `ble_adv_init()`
- `ble_gap_set_device_name()`
- `ble_gap_set_conn_params()`
- `ble_gap_request_conn_params_update()`
- `ble_gap_initiate_conn_update()`
- `ble_uuid_set_vendor_base()`
- `ble_gatt_server_init()`
- `ble_gatt_server_register_evt_handler()`
- `ble_gatt_client_register_evt_handler()`
- `ble_start_advertising()`
- `ble_notify_characteristic()`
- `ble_is_connected()`

See [nrf_ble.h](stack/include/nrf_ble.h) and
[ble_gatt_server.h](stack/include/ble_gatt_server.h) for the full public
interface.

## Architecture At A Glance

- `ble_stack.c`
  Public API wrapper layer. Stores host configuration, UUID base, and
  notification helpers.
- `ble_runtime.c`
  Shared runtime state, small utilities, identity address generation, and
  deferred event delivery through `SWI1_EGU1`.
- `ble_controller_common.c`, `ble_controller_central.c`, and `ble_controller_peripheral.c`
  Shared, central, and peripheral controller flow including advertising,
  scanning, connection-event timing, LL control, retransmission behavior, DLE
  parameter tracking, and ATT/L2CAP packet transport.
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

- GAP events are delivered through one callback registered with
  `ble_gap_register_evt_handler()`.
- Current GAP events are:
  - `BLE_GAP_EVT_CONNECTED`
  - `BLE_GAP_EVT_DISCONNECTED`
  - `BLE_GAP_EVT_SUPERVISION_TIMEOUT`
  - `BLE_GAP_EVT_CONN_UPDATE_IND`
  - `BLE_GAP_EVT_PHY_UPDATE_IND`
  - `BLE_GAP_EVT_TERMINATE_IND`
  - `BLE_GAP_EVT_FEATURE_EXCHANGED`
  - `BLE_GAP_EVT_DATA_LENGTH_UPDATED`
  - `BLE_GAP_EVT_CONTROL_PROCEDURE_UNSUPPORTED`
- GAP events also expose the current `tx_phy` and `rx_phy` so applications can
  log or react when a PHY update takes effect.
- GATT server events are delivered through `ble_gatt_server_register_evt_handler()`.
- The current GATT server event is `BLE_GATT_SERVER_EVT_MTU_EXCHANGE`.
- GATT client procedure events are delivered through
  `ble_gatt_client_register_evt_handler()`.
- Characteristic-specific events are delivered through each characteristic's
  `evt_handler`.
- `ble_gatt_char_evt_t` carries the event type plus `p_characteristic`. For
  write events, applications read the current value from
  `p_evt->p_characteristic->p_value` and `p_evt->p_characteristic->value_len`.
- Both stack-level and characteristic-level callbacks are deferred to
  low-priority software interrupt context instead of being called directly from
  the radio ISR path.

## Example

The repository includes working example applications in
`examples/peripheral_demo` and `examples/central_demo`.

Before building the example, initialize the SDK submodule:

```sh
git submodule update --init --recursive
```

Build the peripheral example with:

```sh
make -C examples/peripheral_demo -j4
```

Build the central example with:

```sh
make -C examples/central_demo -j4
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

static void gap_evt_handler(const ble_gap_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_GAP_EVT_CONNECTED:
        (void)p_evt->params.conn_interval_ms;
        break;
    case BLE_GAP_EVT_CONN_UPDATE_IND:
        (void)p_evt->params.slave_latency;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        ble_start_advertising();
        break;
    default:
        break;
    }
}

static void gatt_server_evt_handler(const ble_gatt_server_evt_t *p_evt)
{
    if (p_evt->evt_type == BLE_GATT_SERVER_EVT_MTU_EXCHANGE)
    {
        (void)p_evt->params.effective_mtu;
    }
}

int main(void)
{
    ble_stack_init(BLE_GAP_ROLE_PERIPHERAL);
    ble_gap_register_evt_handler(gap_evt_handler);
    ble_gatt_server_register_evt_handler(gatt_server_evt_handler);
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

The normal stack flow is:

1. `ble_stack_init()` brings up shared state, deferred events, controller
   runtime, and GATT client state.
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
    connection-event timing with `TIMER0`, and begins using the data channel
    map from the request.
11. When the stack is acting as the central, it automatically sequences LL
    feature exchange, data length update, and a `1M | 2M` PHY request as soon
    as the link is established.
12. If the peer performs the LL length or LL PHY procedures on its own, the
    controller still updates the negotiated packet length or scheduled PHY
    state and reports the resulting GAP events.
13. ATT MTU exchange and connection parameter updates remain application
    driven through the public GATT and GAP APIs.
14. Applications can start GATT discovery immediately after
    `BLE_GAP_EVT_CONNECTED` when acting as the central; the controller keeps
    automatic central LL control traffic ahead of queued ATT/L2CAP payloads.
15. Each connection interval is handled as one RX and one TX exchange. Any ATT
    response, notification, or signaling PDU generated from the received packet
    is queued for the next connection event.
16. Stack-level BLE events and characteristic callbacks are delivered later
    from `SWI1_EGU1_IRQHandler()`.

## Design Notes

- Services and characteristics are provided by the application instead of being
  hardcoded in the stack.
- GAP, GATT server, GATT client, and characteristic events are delivered
  through separate callback registrations.
- GATT characteristic events remain per-characteristic callbacks.
- Characteristic values and current lengths live directly in
  `ble_gatt_characteristic_t`.
- The controller files own BLE packet flow, timing, and LL control handling.
- The controller only accepts legacy `SCAN_REQ` and `CONNECT_REQ` packets whose
  advertiser address and `RxAdd` bit match the current advertising identity.
- Scan responses are intentionally minimal so the advertising RX->TX turnaround
  stays simple and reliable across scanners that actively probe advertisements.
- Connected event timing uses `TIMER0` compare scheduling and the nRF52840
  fixed PPI `RADIO ADDRESS -> TIMER0 CAPTURE[1]` path to re-anchor future
  events from the actual on-air receive timing.
- LE PHY updates stay within the same simple event model by configuring the
  event RX PHY before listening and the TX PHY just before responding.
- `radio_driver.c` owns direct `NRF_RADIO` access.
- The connected data path intentionally uses a simple one-RX / one-TX-per-
  interval model.
- Notifications, ATT responses, and signaling PDUs share one pending connected
  TX slot to keep the controller flow small and traceable.

## Limitations

- No simultaneous multi-role support; the stack runs as either peripheral or
  central at one time
- `TIMER0` is reserved by the controller for connection timing and radio-anchor
  capture
- No L2CAP fragmentation or reassembly
- No security, pairing, or bonding
- No long writes or prepare/execute write support
- Central-side automatic feature exchange, data length update, and PHY update
  are serialized ahead of queued ATT/L2CAP traffic because the controller
  keeps one pending slot per traffic class

## License

MIT. See [LICENSE](LICENSE).
