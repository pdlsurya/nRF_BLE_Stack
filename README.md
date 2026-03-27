# nRF_BLE_Stack

Minimal BLE peripheral stack for nRF SoCs.

This repository contains a compact educational BLE stack focused on clarity and
small code size. It implements the pieces needed for a custom BLE peripheral:
advertising, connection handling, ATT/GATT services and characteristics, and
deferred application callbacks.

## Current Scope

- Peripheral role only
- Advertising with configurable name, flags, TX power, interval, and service UUID
- Runtime registration of custom GATT services and characteristics
- GATT write and notification-state callbacks
- Deferred BLE events through low-priority software interrupt
- Fixed ATT MTU of 23
- Single pending notification TX slot

## Repository Layout

- `include/nrf_ble.h`
  Public BLE stack API
- `core/`
  Stack entry points, runtime state, controller flow, and deferred event delivery
- `gatt/`
  ATT/GATT database build and request handling
- `radio/`
  nRF radio peripheral abstraction used by the controller
- `BLE_STACK_WALKTHROUGH.pdf`
  Detailed architecture and packet-flow walkthrough

## Public API

Main application-facing entry points:

- `ble_stack_init()`
- `ble_register_evt_handler()`
- `ble_adv_init()`
- `ble_gap_init()`
- `ble_gatt_server_init()`
- `ble_start_advertising()`
- `ble_stop_advertising()`
- `ble_notify_characteristic()`
- `ble_disconnect()`

See [nrf_ble.h](include/nrf_ble.h) for the full public interface.

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
    ble_gap_init(&gap_params);
    ble_adv_init(&adv_config);
    APP_ERROR_CHECK_BOOL(ble_gatt_server_init(services, service_count));
    ble_start_advertising();

    for (;;)
    {
        __WFE();
    }
}
```

## Design Notes

- Services and characteristics are provided by the application instead of being hardcoded in the stack.
- BLE stack events are delivered through a unified `ble_evt_t`.
- GATT events remain per-characteristic callbacks.
- `ble_controller.c` owns BLE packet flow and timing.
- `radio_driver.c` owns direct `NRF_RADIO` access.

## Limitations

- No central role or scanning support
- No L2CAP fragmentation or reassembly
- No security, pairing, or bonding
- No long writes or prepare/execute write support
- Preferred GAP connection parameters are stored but not actively negotiated on-air

## Documentation

For a detailed code and packet-flow explanation, read:

- `BLE_STACK_WALKTHROUGH.pdf`

## License

MIT. See [LICENSE](LICENSE).
