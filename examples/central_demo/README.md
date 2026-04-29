# central_demo

Minimal BLE central example for `nrf-ble-stack`.

This example:

- initializes the stack in central role
- scans for the demo custom service used by `examples/peripheral_demo`
- connects automatically when the service UUID is found
- discovers the notify characteristic and its CCCD
- enables notifications and logs received counter updates over USB CDC ACM

## Requirements

- `external/nrf5-sdk` submodule initialized
- GNU Arm Embedded toolchain
- `make`

## Build

```sh
git submodule update --init --recursive
make -C examples/central_demo -j4
```

## Notes

- The central demo is meant to pair with `examples/peripheral_demo`.
- The example uses the stack directly from the repository root through `BLE_STACK_DIR := ../..`.
- The small `support/usb_log` module is included locally in this example so the build does not depend on machine-specific paths.
