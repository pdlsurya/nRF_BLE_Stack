<!-- SPDX-License-Identifier: MIT -->

# thermometer_adv_demo

Nonconnectable BLE advertising example for `nrf-ble-stack`.

This example:

- initializes the stack in peripheral role
- starts nonconnectable legacy advertising
- advertises the Health Thermometer service UUID `0x1809` from a complete
  16-bit service UUID list
- includes service data with a single `uint8_t` temperature value
- updates the application-owned service-data byte once per second
- logs updates over USB CDC ACM

## Requirements

- `external/nrf5-sdk` submodule initialized
- GNU Arm Embedded toolchain
- `make`

## Build

```sh
git submodule update --init --recursive
make -C examples/thermometer_adv_demo -j4
```

## Notes

- The example uses `BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED`.
- The service-data payload points directly at the mutable temperature byte, so
  the next advertising event uses the latest value after the timer updates it.
