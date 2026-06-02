<!-- SPDX-License-Identifier: MIT -->

# nrf-ble-stack

Minimal BLE stack for nRF52 series SoCs.

This repository contains a compact educational BLE stack focused on clarity,
small code size, and readable control flow. It implements the pieces needed for
an application-defined BLE peripheral or central: advertising, passive
scanning, central connection initiation, connection handling, ATT/GATT
services and characteristics, GATT client procedures, deferred application
callbacks, automatic central link-layer feature exchange, automatic central
data length and PHY updates, delayed peripheral connection parameter update
requests, and application-driven MTU and connection parameter procedures.

The current implementation targets nRF52-series RADIO behavior and timing.
nRF51 compatibility is not implemented yet.

The stack is intentionally small enough to read end to end. Public API,
controller logic, ATT/GATT handling, and radio access are kept in separate
layers so packet flow is easy to follow in code.

## Current Scope

- nRF52 series support only
- Peripheral and central role support
- One role active at a time
- Advertising with configurable name, flags, TX power, interval, service UUID
  lists, service data, and manufacturer-specific data
- Passive and active legacy scanning with scan report callbacks, optional
  auto-connect filter, and `scan_response` reporting
- Legacy `SCAN_RSP` support with separate application-defined advertising and
  scan-response data blocks
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
- Delayed peripheral connection parameter update request when preferred
  parameters are configured
- Application-driven peripheral and central connection parameter update APIs
- Connection-event timing re-anchored from the central packet address using
  `TIMER0` plus fixed PPI capture for better interoperability with active
  central implementations
- One RX and one TX exchange per connection interval
- Bounded connected L2CAP TX queue for notifications, ATT responses, and
  signaling PDUs

## Repository Layout

- `stack/include/nrf_ble.h`
  Umbrella public BLE stack header
- `stack/include/ble_gap.h`
  Public GAP types and APIs
- `stack/include/ble_gatt_server.h`
  Public GATT server types and APIs
- `stack/include/ble_gatt_client.h`
  Public GATT client types and APIs
- `stack/core/`
  Stack entry points, runtime state, UUID helpers, and
  deferred event delivery
- `stack/controller/`
  Shared, central, and peripheral controller/link-layer implementation
- `stack/include/ble_att.h`
  Public ATT size and MTU definitions
- `stack/host/`
  Internal host runtime state and GAP/L2CAP/GATT host submodules
- `stack/host/gap/`
  GAP-facing host APIs
- `stack/host/l2cap/`
  Internal L2CAP definitions and signaling helpers
- `stack/host/gatt/`
  GATT client/server implementation and public GATT helpers
- `stack/radio/`
  nRF radio peripheral abstraction used by the controller
- `examples/peripheral_demo/`
  Example peripheral application using the stack
- `examples/thermometer_adv_demo/`
  Nonconnectable Health Thermometer service-data advertising example
- `examples/central_demo/`
  Minimal central application that scans, connects, starts GATT discovery on
  `BLE_GAP_EVT_CONNECTED`, and subscribes while the stack performs automatic
  central LL setup in the background
- `external/nrf5-sdk/`
  nRF5 SDK Git submodule used by the example build
- `README.md`
  Architecture and packet-flow walkthrough

## Public API

Main application-facing entry points include:

- Core and GAP:
  `ble_stack_init()`, `ble_gap_register_evt_handler()`,
  `ble_gap_register_scan_report_handler()`, `ble_gap_adv_init()`,
  `ble_gap_scan_init()`, `ble_gap_start_advertising()`,
  `ble_gap_start_scanning()`, `ble_gap_stop_scanning()`,
  `ble_gap_set_scan_filter()`, `ble_gap_clear_scan_filter()`,
  `ble_gap_set_device_name()`, `ble_gap_set_conn_params()`,
  `ble_gap_connect()`, `ble_gap_request_conn_params_update()`,
  `ble_gap_initiate_conn_update()`, `ble_gap_disconnect()`,
  `ble_uuid_set_vendor_base()`, and `ble_gap_is_connected()`
- GATT server:
  `ble_gatt_server_init()`, `ble_gatt_server_register_evt_handler()`,
  `ble_gatt_server_notify_characteristic()`, and
  `ble_gatt_server_indicate_characteristic()`
- GATT client:
  `ble_gatt_client_register_evt_handler()`, `ble_gatt_client_is_busy()`,
  `ble_gatt_client_exchange_mtu()`,
  `ble_gatt_client_discover_primary_services()`,
  `ble_gatt_client_discover_primary_services_by_uuid()`,
  `ble_gatt_client_discover_characteristics()`,
  `ble_gatt_client_discover_descriptors()`, `ble_gatt_client_read()`,
  `ble_gatt_client_write()`, and `ble_gatt_client_write_cccd()`

See [nrf_ble.h](stack/include/nrf_ble.h),
[ble_gap.h](stack/include/ble_gap.h),
[ble_uuid.h](stack/include/ble_uuid.h),
[ble_gatt_server.h](stack/include/ble_gatt_server.h), and
[ble_gatt_client.h](stack/include/ble_gatt_client.h) for the full public
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
- `ble_l2cap.c`
  Internal L2CAP connection-data dispatch, ATT PDU routing, signaling PDU
  handling, and connection-parameter update request formatting.
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

The repository includes working example applications:

- `examples/peripheral_demo`
  Connectable GATT peripheral demo
- `examples/thermometer_adv_demo`
  Nonconnectable Health Thermometer service-data advertiser
- `examples/central_demo`
  Minimal central scanner and GATT client demo

Before building the example, initialize the SDK submodule:

```sh
git submodule update --init --recursive
```

Then configure the ARM GCC toolchain path in the SDK makefile:

```sh
external/nrf5-sdk/components/toolchain/gcc/Makefile.posix
```

Set `GNU_INSTALL_ROOT` to the directory that contains `arm-none-eabi-gcc`,
and set `GNU_VERSION` and `GNU_PREFIX` for your installed toolchain.

Build the peripheral example with:

```sh
make -C examples/peripheral_demo -j4
```

Build the thermometer advertising example with:

```sh
make -C examples/thermometer_adv_demo -j4
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
- The USB CDC logger is self-pumping with interrupt-driven USBD events, so the
  bundled examples can sleep with `__WFE()` instead of polling a log idle hook.

## Runtime Flow Summary

The stack has a shared initialization path and then diverges into peripheral or
central runtime flow depending on the configured role.

For detailed role-specific internal diagrams and state machines, see
[`BLE_STACK_FLOWCHARTS.md`](BLE_STACK_FLOWCHARTS.md).

Common setup:

1. `ble_stack_init()` brings up shared state, deferred events, controller
   runtime, and GATT client state.
2. `ble_gap_set_device_name()` stores the local name used by both advertising
   and the GAP Device Name attribute.
3. `ble_gap_set_conn_params()` stores preferred connection parameters that can
   later be requested by the stack.
4. `ble_uuid_set_vendor_base()` stores the one custom 128-bit base UUID used by
   vendor 16-bit UUIDs.
5. Each connection interval is handled as one RX and one TX exchange. Any ATT
   response, notification, or signaling PDU generated from the received packet
   is queued for the next connection event.
6. Stack-level BLE events and characteristic callbacks are delivered later
   from `SWI1_EGU1_IRQHandler()`.
7. Connected L2CAP data PDUs are decoded by `ble_l2cap_process_conn_data_pdu()`,
   which routes ATT traffic into `ble_gatt_client_process_att_pdu()` or
   `ble_gatt_server_process_att_pdu()` depending on role, and routes L2CAP
   signaling traffic into the internal signaling-PDU handler.

Peripheral flow:

1. `ble_gap_adv_init()` stores advertising parameters and copies configured
   advertising and scan-response metadata. Service data and manufacturer-data
   payload pointers are retained so applications can update those buffers
   between advertising events.
2. `ble_gatt_server_init()` builds the ATT database from the application's
   service table.
3. `ble_gap_start_advertising()` starts repeated advertising events on channels
   37, 38, and 39.
4. After each advertising transmission, the controller opens a short RX window
   and listens for a targeted `SCAN_REQ` or `CONNECT_REQ`.
5. When a valid `SCAN_REQ` is received, the controller sends a `SCAN_RSP` that
   carries the advertiser address and any configured scan-response AD
   structures.
6. When a `CONNECT_REQ` that targets the local advertiser address and address
   type is received, the controller switches to connected mode, starts
   connection-event timing with `TIMER0`, and begins using the data channel
   map from the request.
7. When preferred peripheral connection parameters are configured, the stack
   starts a one-shot delayed L2CAP Connection Parameter Update Request after
   connect.
8. ATT MTU exchange, notifications, indications, and explicit peripheral
   connection parameter update APIs remain available through the public GATT
   and GAP interfaces.

Central flow:

1. `ble_gap_scan_init()` stores scan interval and window parameters.
2. Optional `ble_gap_set_scan_filter()` configuration tells the controller
   which peer address, name, or service UUID should trigger auto-connect.
3. `ble_gap_start_scanning()` starts passive or active scanning on channels
   37, 38, and 39 and reports advertisements through the registered
   scan-report callback.
4. If active scanning is enabled and a scannable advertisement is received,
   the controller can send `SCAN_REQ`, report the matching `SCAN_RSP`, and
   remember that peer for a later connectable advertisement if the filter only
   matches in scan-response data.
5. If the app calls `ble_gap_connect()` or a scan filter matches a connectable
   advertisement directly, the controller builds and transmits a legacy
   connect request and then switches to connected mode.
6. Once connected, the central automatically sequences LL feature exchange,
   data length update, and a `1M | 2M` PHY request.
7. If the peer performs LL feature exchange, LL length update, connection
   update, or LL PHY update procedures on its own, shared controller handling
   updates the negotiated link state and reports the resulting GAP events
   without entering central-only procedure state from peripheral mode.
8. Applications can start ATT MTU exchange and GATT discovery immediately
   after `BLE_GAP_EVT_CONNECTED`; automatic central LL control traffic stays
   ahead of queued ATT/L2CAP payloads.
9. Central-side GATT client procedures then drive service discovery,
   characteristic discovery, descriptor discovery, reads, writes, and CCCD
   updates.

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
- Optional name, TX power, service UUID lists, service data, and
  manufacturer-specific data fields can be configured separately for the primary
  advertising packet and scan response. Fields that do not fit in the selected
  packet are omitted by the packet builder.
- If a configured complete service UUID list does not fully fit, the packet
  builder includes the UUIDs that fit and advertises that AD structure as an
  incomplete list.
- Connected event timing uses `TIMER0` compare scheduling and the nRF52840
  fixed PPI `RADIO ADDRESS -> TIMER0 CAPTURE[1]` path to re-anchor future
  events from the actual on-air receive timing.
- LE PHY updates stay within the same simple event model by configuring the
  event RX PHY before listening and the TX PHY just before responding.
- `radio_driver.c` owns direct `NRF_RADIO` access.
- The connected data path intentionally uses a simple one-RX / one-TX-per-
  interval model.
- Notifications, ATT responses, and signaling PDUs are buffered through a
  small connected L2CAP TX queue, while LL control response/control traffic
  keeps dedicated pending slots.

## Limitations

- nRF51 support is not implemented yet
- No simultaneous multi-role support; the stack runs as either peripheral or
  central at one time
- `TIMER0` is reserved by the controller for connection timing and radio-anchor
  capture
- No L2CAP fragmentation or reassembly
- No security, pairing, or bonding
- No long writes or prepare/execute write support
- Central-side automatic feature exchange, data length update, and PHY update
  are serialized ahead of queued ATT/L2CAP traffic because the controller
  keeps dedicated LL control slots ahead of a small L2CAP TX queue

## License

MIT. See [LICENSE](LICENSE).
