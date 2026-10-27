# NMEA2000_esp32_idf_twai — Spec

Status: **draft for review**, 2026-10-27. v0.1.0 implemented the same day: 22 host tests pass (`pio test -e native`), compiles for the M5Stack Atom Lite (`pio run -e m5stack-atom`). Not yet run on hardware.

## 1. Purpose

A CAN driver (`tNMEA2000` subclass) for the NMEA2000 library on ESP32, using Espressif's TWAI driver. It must keep a device on the bus unattended: it recovers from bus-off by itself, never blocks, and reports what happened.

### Why not the existing drivers
Reviewed 2026-10-27 (details in N2k.AIS_Gateway `docs/SPEC.md` §5.3):

| Driver | Problem |
|---|---|
| `ttlappalainen/NMEA2000_esp32` | Writes the CAN registers directly. Its error-interrupt handler is empty: no bus-off recovery, and no ESP32 errata workarounds. Original ESP32 only. Its strength: Timo reports it passed NMEA 2000 certification. |
| `sergei/NMEA2000_esp32_twai` | Two paths leave CAN stuck off: `CANOpen()` retried after a failed `twai_start()` fails forever at `twai_driver_install()`; a failed `twai_start()` after recovery leaves the driver stopped with nothing watching it. An alert task wakes on every frame. It carries a UDP side interface and frame listeners. It ignores the library's buffer-size settings. Timo declined to support it ("does not use properly my library", NMEA2000_esp32 #25). |
| `skarlsson/NMEA2000_twai` | Needs ESP-IDF ≥ 5.2 (`twai_handle_t`); doesn't build on Arduino-ESP32 2.0.x. |

### Goals
- Works with the NMEA2000 library as its other drivers do: buffer sizes from the library's settings, no API the library doesn't expect.
- Bus-off recovery and restart after any failure, from the polled controller state.
- **No extra tasks and no callbacks from other tasks.** All work happens inside the library's calls into the driver, on the caller's task.
- Status and counters for the application (status pages, logs).
- Host-side unit tests of the recovery logic against a fake TWAI layer.

### Non-goals
- CAN FD, bit rates other than 250 kbit/s, multiple controllers.
- ESP-IDF 5.x `esp_twai` node API. The legacy `driver/twai.h` API is used.
- Logging. The driver counts; the application decides what to log.

## 2. Target

| | |
|---|---|
| Framework | Arduino-ESP32 2.0.17 (ESP-IDF 4.4), PlatformIO `espressif32 @ 6.8.1` |
| Chips | ESP32 first (M5Stack Atom Lite). ESP32-S3 later; same API, untested until then. |
| Errata | Arduino 2.0.17's prebuilt sdkconfig enables `CONFIG_TWAI_ERRATA_FIX_BUS_OFF_REC`, `_TX_INTR_LOST`, `_RX_FRAME_INVALID`, `_RX_FIFO_CORRUPT` (verified 2026-10-27). Arduino 2.0.4 had them off — README must state the minimum core version. |
| Known limit | `CONFIG_TWAI_ISR_IN_IRAM` is off in Arduino's sdkconfig, so the TWAI interrupt is held off during flash writes (NVS, OTA). Frames can be lost then. Not fixable without a custom sdkconfig; documented, and visible in the missed/overrun counters. |
| Licence | MIT |

## 3. Interface

```cpp
class tNMEA2000_esp32_idf_twai : public tNMEA2000 {
public:
  tNMEA2000_esp32_idf_twai(gpio_num_t txPin, gpio_num_t rxPin,
                           twai_mode_t mode = TWAI_MODE_NORMAL);

  struct Status {
    twai_state_t state;            // from twai_get_status_info()
    uint32_t txErrorCounter, rxErrorCounter;
    uint32_t txFailed, rxMissed, rxOverrun, arbLost, busErrors;  // driver totals
    uint32_t busOffCount;          // bus-off entries seen
    uint32_t recoveryCount;        // completed recoveries (back to running)
    uint32_t restartFailures;      // twai_start() failures
    uint32_t errorWarningCount, errorPassiveCount;  // alert counts
    uint32_t droppedFrames;        // non-extended or RTR frames discarded
  };
  Status GetStatus() const;        // cheap; safe to call from the main loop

protected:  // tNMEA2000 driver interface
  void InitCANFrameBuffers() override;
  bool CANOpen() override;
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) override;
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;
};
```

Exact field names may change during implementation; the set of information may not shrink.

## 4. Behaviour

### 4.1 Buffers
Follow `NMEA2000_esp32`: in `InitCANFrameBuffers()`, size the TWAI RX and TX queues from the library's `MaxCANReceiveFrames` / `MaxCANSendFrames` (set by `SetN2kCANReceiveFrameBufSize()` / `SetN2kCANSendFrameBufSize()`), with sensible minimums, and leave a small library-side send buffer. No extra constructor parameters for queue sizes.

### 4.2 Open
`CANOpen()` is idempotent and recovers from partial failure:
- Install the driver only if it isn't installed (250 kbit/s, accept-all filter, the alerts in §4.4).
- If installed and stopped, call `twai_start()`.
- Return true only when the state is RUNNING.

The library retries `CANOpen()` every second until it succeeds, so any failure here is retried rather than permanent.

### 4.3 Send and receive
- `CANSendFrame()`: `twai_transmit(…, 0)` of an extended frame. Returns false if not RUNNING or the queue is full, so the library keeps the frame in its buffer and retries in order. `wait_sent` needs no special handling: the controller has a single transmit buffer and the TWAI TX queue is FIFO, so frame order is kept. This matches `NMEA2000_esp32`, which also ignores `wait_sent`.
- `CANGetFrame()`: `twai_receive(…, 0)`. Non-extended (11-bit) and RTR frames are discarded and counted; the next frame is tried.
- Neither call ever blocks.

### 4.4 Supervision (bus-off recovery and restart)
Runs from `CANGetFrame()` (the library calls it on every `ParseMessages()`), at most once per supervision interval (proposed 10 ms), using `twai_get_status_info()`:

| State | Action |
|---|---|
| RUNNING | Nothing. |
| BUS_OFF | `twai_initiate_recovery()`, count a bus-off. |
| RECOVERING | Wait. The controller needs 128 occurrences of the bus-free signal (IDF docs). |
| STOPPED (after recovery, or a failed start) | `twai_start()`. On success count a recovery; on failure count it and retry at the next interval (proposed back-off: 1 s). |

Alerts are read with `twai_read_alerts(…, 0)` at the same point, only to count error-warning and error-passive entries. Recovery is driven by the **state**, not by alerts, so a missed alert can't leave the driver stuck. Only error alerts are enabled; RX/TX data alerts are not.

Both `twai_start()` and `twai_initiate_recovery()` clear the TWAI TX queue (IDF docs). A fast packet in flight at bus-off is cut short; receivers discard incomplete fast packets. The library-side buffer is not affected.

### 4.5 Threading
Everything runs on the task that calls the library (`ParseMessages()` / `SendMsg()`). The driver creates no tasks, registers no callbacks, and holds no locks of its own. The TWAI driver's own ISR is the only other context, and the IDF API is safe to call against it.

## 5. Testing

### 5.1 Host tests (`pio test -e native`)
A fake `driver/twai.h` layer with scripted states and return codes. Cases:
- Open: success; install OK + start fails, then later succeeds (the case that breaks the Sergei driver).
- Bus-off → recovery → stopped → started; counters correct.
- Start fails after recovery, then succeeds on retry.
- Supervision is rate-limited.
- Non-extended and RTR frames are dropped and counted.
- Send returns false when not running or the queue is full.
- Buffer sizes follow the library's settings.

### 5.2 On target
- Builds in N2k.AIS_Gateway with no warnings.
- Desk check over USB (one device, no bus): `TWAI_MODE_NO_ACK` with self-reception — frames sent are received back.
- In the AIS gateway trial: address claim, AIS PGNs on the plotter, counters on the status page. Bus-off is not forced on the live bus (AIS gateway spec §7); a natural one shows in the counters.

## 6. Open questions
- **Q1.** Timo's view on what "use the library properly" means for a TWAI driver, and whether his unpublished restart solution (NMEA2000 #468) affects §4.4. Asked in [ttlappalainen/NMEA2000_esp32#39](https://github.com/ttlappalainen/NMEA2000_esp32/issues/39), 2026-10-27 (pending).
- **Q2.** Supervision interval and restart back-off values (10 ms / 1 s proposed).
- **Q3.** Should the driver also offer an optional "stuck in RECOVERING too long → stop/uninstall/reinstall" escalation, or is that over-engineering?
