# nRF_BLE_Stack

Minimal BLE peripheral stack for nRF SoCs.

This repository contains a compact educational BLE stack focused on clarity and
small code size. It implements the pieces needed for a custom BLE peripheral:
advertising, connection handling, ATT/GATT services and characteristics, and
deferred application callbacks.

The stack is meant to be small enough to read end to end. Public API, controller
logic, ATT/GATT handling, and radio access are kept in separate layers so the
packet flow is easy to follow in code.

## Current Scope

- Peripheral role only
- Advertising with configurable name, flags, TX power, interval, and service UUID
- Runtime registration of custom GATT services and characteristics
- GATT write and notification-state callbacks
- Deferred BLE events through low-priority software interrupt
- Fixed ATT MTU of 23
- One RX and one TX exchange per connection interval
- Single pending ATT TX slot for notifications and ATT responses

## Repository Layout

- `stack/include/nrf_ble.h`
  Public BLE stack API
- `stack/core/`
  Stack entry points, runtime state, controller flow, and deferred event delivery
- `stack/gatt/`
  ATT/GATT database build and request handling
- `stack/radio/`
  nRF radio peripheral abstraction used by the controller
- `examples/custom_ble_stack_demo/`
  Example peripheral application using the stack
- `external/nrf5-sdk/`
  nRF5 SDK Git submodule used by the example build
- `BLE_STACK_WALKTHROUGH.pdf`
  Detailed architecture and packet-flow walkthrough

## Public API

Main application-facing entry points:

- `ble_stack_init()`
- `ble_register_evt_handler()`
- `ble_adv_init()`
- `ble_gap_set_device_name()`
- `ble_gatt_server_init()`
- `ble_start_advertising()`
- `ble_notify_characteristic()`
- `ble_is_connected()`

See [nrf_ble.h](stack/include/nrf_ble.h) for the full public interface.

## Architecture At A Glance

- `ble_stack.c`
  Small public API wrapper layer. Stores configuration and hands work to the
  controller and GATT server.
- `ble_runtime.c`
  Shared runtime state, utility helpers, and deferred event delivery through
  `SWI1_EGU1`.
- `ble_controller.c`
  Advertising, connect-request handling, connection-event timing, LL control,
  retransmission behavior, and ATT packet transport.
- `ble_gatt_server.c`
  ATT database construction, ATT request handling, CCCD tracking, and
  notification building.
- `radio_driver.c`
  Direct `NRF_RADIO` access hidden behind a small abstraction.

## Event Model

- Stack-level BLE events are delivered through one callback registered with
  `ble_register_evt_handler()`.
- Current stack-level events are:
  - `BLE_GAP_EVT_CONNECTED`
  - `BLE_GAP_EVT_DISCONNECTED`
  - `BLE_GAP_EVT_SUPERVISION_TIMEOUT`
  - `BLE_GAP_EVT_CONN_UPDATE_IND`
  - `BLE_GAP_EVT_TERMINATE_IND`
  - `BLE_GATT_EVT_MTU_EXCHANGE`
- Characteristic-specific events are delivered through each characteristic's
  `evt_handler`.
- Both stack-level and characteristic-level callbacks are deferred to
  low-priority software interrupt context instead of being called directly from
  the radio ISR path, so application callbacks do not run from the radio ISR.

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

Note: the bundled example is currently written for the nRF52840 dongle and uses
the included `debug_log` backend over the dongle's built-in USB interface. Other
nRF devices may not provide the same USB logging path, so their applications
should replace that example logging backend with whatever output is available on
the target, such as UART, RTT, or another board-specific logger.

## Typical Usage

```c
static void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_GAP_EVT_CONNECTED:
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
    ble_adv_init(&adv_config);
    APP_ERROR_CHECK_BOOL(ble_gatt_server_init(services, service_count));
    ble_start_advertising();

    for (;;)
    {
        __WFE();
    }
}
```

## Runtime Flow Summary

The normal peripheral flow is:

1. `ble_stack_init()` brings up shared state, deferred events, and controller runtime.
2. `ble_gap_set_device_name()` stores the local name used by both advertising and the GAP Device Name attribute.
3. `ble_adv_init()` stores advertising parameters.
4. `ble_gatt_server_init()` builds the ATT database from the application's service table.
5. `ble_start_advertising()` starts repeated advertising events on channels 37, 38, and 39.
6. When a `CONNECT_REQ` is received, the controller switches to connected mode and starts connection-event timing with `TIMER2`.
7. Each connection interval is handled as one RX and one TX exchange. Any ATT response or notification generated from the received packet is queued for the next connection event.
8. Stack-level BLE events and characteristic callbacks are delivered later from `SWI1_EGU1_IRQHandler()`.

## Design Notes

- Services and characteristics are provided by the application instead of being hardcoded in the stack.
- BLE stack events are delivered through a unified `ble_evt_t`.
- GATT events remain per-characteristic callbacks.
- `ble_controller.c` owns BLE packet flow and timing.
- `radio_driver.c` owns direct `NRF_RADIO` access.
- The stack keeps ATT MTU fixed at 23 to avoid adding L2CAP fragmentation and reassembly logic.
- The connected data path intentionally uses a simple one-RX / one-TX-per-interval model.
- Notifications and ATT responses share a single pending connected TX slot, which keeps the controller flow simple and easy to trace.

## Limitations

- No central role or scanning support
- No L2CAP fragmentation or reassembly
- No security, pairing, or bonding
- No long writes or prepare/execute write support
- No active connection parameter update procedure

## Documentation

For a detailed code and packet-flow explanation, read:

- `BLE_STACK_WALKTHROUGH.pdf`

## License

MIT. See [LICENSE](LICENSE).
