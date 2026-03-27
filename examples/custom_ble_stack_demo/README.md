# custom_ble_stack_demo

Example BLE peripheral application for `nRF_BLE_Stack`.

This example:

- initializes the stack
- configures GAP advertising and preferred connection parameters
- registers a custom GATT service with a counter characteristic and a text characteristic
- sends periodic notifications while connected
- logs GAP and GATT activity over USB CDC ACM

## Requirements

- `external/nrf5-sdk` submodule initialized
- GNU Arm Embedded toolchain
- `make`

## Build

```sh
git submodule update --init --recursive
make -C examples/custom_ble_stack_demo -j4
```

## Notes

- The example uses the stack directly from the repository root through `BLE_STACK_DIR := ../..`.
- The small `support/debug_log` module is included locally in this example so the build does not depend on machine-specific paths.
