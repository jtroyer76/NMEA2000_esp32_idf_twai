/*
  NMEA2000_esp32_idf_twai - CAN driver for the NMEA2000 library on ESP32, using Espressif's TWAI
  driver (driver/twai.h, ESP-IDF 4.4 / Arduino-ESP32 2.0.x).

  - Bus-off recovery and restart after any failure, driven from the polled controller state.
  - No extra tasks and no callbacks from other tasks: everything runs inside the library's calls
    into the driver (ParseMessages / SendMsg), on the caller's task.
  - Queue sizes follow the library's SetN2kCANReceiveFrameBufSize / SetN2kCANSendFrameBufSize.
  - GetStatus() for status pages and logs.

  Built against Arduino-ESP32 2.0.17: its prebuilt sdkconfig enables the ESP32 TWAI
  errata workarounds (2.0.4 does not).

  Copyright (c) 2026 Jess Troyer. MIT License, see LICENSE.
*/
#pragma once

#include <stdint.h>
#include "driver/twai.h"
#include "NMEA2000.h"

class tNMEA2000_esp32_idf_twai : public tNMEA2000 {
public:
  // How often the controller state is checked (bus-off recovery, restart), and how long to wait
  // after a failed twai_start() before trying again.
  static const uint32_t SuperviseIntervalMs = 10;
  static const uint32_t RestartBackoffMs = 1000;

  // In TWAI_MODE_NO_ACK, frames are sent as self-reception requests, so a lone device on the
  // desk receives its own frames (bench check only).
  tNMEA2000_esp32_idf_twai(gpio_num_t txPin, gpio_num_t rxPin, twai_mode_t mode = TWAI_MODE_NORMAL);

  struct Status {
    twai_state_t state;           // controller state (STOPPED if the driver is not installed)
    uint32_t txErrorCounter;      // current TEC / REC
    uint32_t rxErrorCounter;
    uint32_t txFailed;            // TWAI driver totals
    uint32_t rxMissed;            // RX queue full (or errata workaround)
    uint32_t rxOverrun;           // RX FIFO overrun
    uint32_t arbLost;
    uint32_t busErrors;
    uint32_t busOffCount;         // bus-off entries (recovery initiated)
    uint32_t recoveryCount;       // recoveries completed (running again)
    uint32_t restartFailures;     // failed twai_start() calls
    uint32_t errorWarningCount;   // error counter crossed the warning limit
    uint32_t errorPassiveCount;   // entered error passive
    uint32_t droppedFrames;       // received non-extended or remote frames, discarded
  };
  Status GetStatus() const;

protected:  // tNMEA2000 driver interface
  void InitCANFrameBuffers() override;
  bool CANOpen() override;
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) override;
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;

  // Checks the controller state and drives recovery/restart. Rate-limited to SuperviseIntervalMs
  // unless forced. Called from CANGetFrame() and CANOpen().
  void Supervise(bool force = false);

private:
  gpio_num_t txPin;
  gpio_num_t rxPin;
  twai_mode_t mode;
  uint32_t txQueueLen = 0;
  uint32_t rxQueueLen = 0;
  bool installed = false;
  bool recovering = false;        // bus-off seen; counts a recovery when running again
  bool startFailed = false;       // last twai_start() failed; wait RestartBackoffMs
  uint32_t lastSupervise = 0;
  uint32_t lastStartAttempt = 0;
  bool supervisedOnce = false;

  uint32_t busOffCount = 0;
  uint32_t recoveryCount = 0;
  uint32_t restartFailures = 0;
  uint32_t errorWarningCount = 0;
  uint32_t errorPassiveCount = 0;
  uint32_t droppedFrames = 0;

  bool TryStart(uint32_t now);
};
