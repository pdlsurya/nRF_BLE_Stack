<!-- SPDX-License-Identifier: MIT -->

# central_demo

Minimal BLE central example for `nrf-ble-stack`.

This example:

- initializes the stack in central role
- actively scans for the demo custom service used by `examples/peripheral_demo`
- logs scan-report hits, including whether the report came from advertising
  data or a scan response
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
- The example registers `ble_gap_register_scan_report_handler()` so active-scan
  `SCAN_RSP` data is visible in logs before connection.
