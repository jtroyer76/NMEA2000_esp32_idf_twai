# NMEA2000_esp32_idf_twai

CAN driver for Timo Lappalainen's [NMEA2000](https://github.com/ttlappalainen/NMEA2000) library on ESP32, built on Espressif's TWAI driver (`driver/twai.h`).

Status: **v0.1.0, host-tested, not yet run on hardware.** See [docs/SPEC.md](docs/SPEC.md).

## Use

```cpp
#include <NMEA2000_esp32_idf_twai.h>
tNMEA2000_esp32_idf_twai NMEA2000(GPIO_NUM_22, GPIO_NUM_19);  // TX, RX
```

Then use `NMEA2000` as with any other driver. Call `ParseMessages()` often: supervision (bus-off recovery, restart) runs inside it. `NMEA2000.GetStatus()` returns state and counters. Built against Arduino-ESP32 2.0.17, whose prebuilt sdkconfig enables the TWAI errata workarounds (2.0.4 does not).

## Development

- `pio test -e native` — host tests against a fake `driver/twai.h` (`test/fakes`). Needs a host `gcc`/`g++`.
- `pio run -e m5stack-atom` — compiles `examples/BuildCheck` for the M5Stack Atom Lite.

Why: the TWAI driver includes Espressif's ESP32 CAN errata workarounds and reports bus errors. This driver adds bus-off recovery and status counters, without extra tasks or callbacks from other tasks.

First user: [N2k.AIS_Gateway](https://github.com/jtroyer76/N2k.AIS_Gateway).

## License

MIT. See [LICENSE](LICENSE).
