// Copyright (c) 2026 Jess Troyer. MIT License, see LICENSE.
#include "NMEA2000_esp32_idf_twai.h"

#include <string.h>
#include "N2kTimer.h"

// Only error alerts: they are read without blocking during supervision, just for counting.
// Recovery is driven by the controller state, so a missed alert can't leave the driver stuck.
// RX/TX data alerts stay off so nothing happens per frame.
static const uint32_t kAlerts = TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;

tNMEA2000_esp32_idf_twai::tNMEA2000_esp32_idf_twai(gpio_num_t txPin, gpio_num_t rxPin, twai_mode_t mode)
  : tNMEA2000(), txPin(txPin), rxPin(rxPin), mode(mode) {}

//*****************************************************************************
// Same sizing as NMEA2000_esp32: the TWAI queues take the library's settings and the library
// keeps only a small send buffer of its own.
void tNMEA2000_esp32_idf_twai::InitCANFrameBuffers() {
  if (MaxCANReceiveFrames < 10) MaxCANReceiveFrames = 50;
  if (MaxCANSendFrames < 10) MaxCANSendFrames = 40;
  rxQueueLen = MaxCANReceiveFrames;
  txQueueLen = MaxCANSendFrames - 4;
  MaxCANSendFrames = 4;

  tNMEA2000::InitCANFrameBuffers();
}

//*****************************************************************************
// Idempotent: the library retries CANOpen() every second until it returns true, so a partial
// failure (installed but not started) must not turn into a permanent one.
bool tNMEA2000_esp32_idf_twai::CANOpen() {
  if (!installed) {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, mode);
    g.tx_queue_len = txQueueLen;
    g.rx_queue_len = rxQueueLen;
    g.alerts_enabled = kAlerts;
    const twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();  // NMEA 2000 is fixed at 250 kbit/s
    const twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    installed = true;
  }

  Supervise(true);

  twai_status_info_t s;
  return twai_get_status_info(&s) == ESP_OK && s.state == TWAI_STATE_RUNNING;
}

//*****************************************************************************
// wait_sent needs no handling: the controller has one transmit buffer and the TWAI TX queue is
// FIFO, so frames go out in order (NMEA2000_esp32 ignores it too). Returning false keeps the
// frame in the library's buffer for a retry.
bool tNMEA2000_esp32_idf_twai::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf,
                                            bool /*wait_sent*/) {
  twai_message_t m;
  memset(&m, 0, sizeof(m));
  m.extd = 1;
  m.self = (mode == TWAI_MODE_NO_ACK);
  m.identifier = id;
  m.data_length_code = len > 8 ? 8 : len;
  memcpy(m.data, buf, m.data_length_code);
  return twai_transmit(&m, 0) == ESP_OK;
}

//*****************************************************************************
bool tNMEA2000_esp32_idf_twai::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  Supervise();

  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if (!m.extd || m.rtr) {  // NMEA 2000 uses only extended data frames
      droppedFrames++;
      continue;
    }
    id = m.identifier;
    len = m.data_length_code > 8 ? 8 : m.data_length_code;
    memcpy(buf, m.data, len);
    return true;
  }
  return false;
}

//*****************************************************************************
void tNMEA2000_esp32_idf_twai::Supervise(bool force) {
  if (!installed) return;
  const uint32_t now = N2kMillis();
  if (!force && supervisedOnce && now - lastSupervise < SuperviseIntervalMs) return;
  supervisedOnce = true;
  lastSupervise = now;

  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) errorWarningCount++;
    if (alerts & TWAI_ALERT_ERR_PASS) errorPassiveCount++;
  }

  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return;

  switch (s.state) {
    case TWAI_STATE_RUNNING:
      break;
    case TWAI_STATE_BUS_OFF:
      if (twai_initiate_recovery() == ESP_OK) {
        busOffCount++;
        recovering = true;
      }
      break;
    case TWAI_STATE_RECOVERING:
      break;  // waiting for 128 occurrences of the bus-free signal
    case TWAI_STATE_STOPPED:
      if (force || !startFailed || now - lastStartAttempt >= RestartBackoffMs) TryStart(now);
      break;
  }
}

//*****************************************************************************
bool tNMEA2000_esp32_idf_twai::TryStart(uint32_t now) {
  lastStartAttempt = now;
  if (twai_start() != ESP_OK) {
    restartFailures++;
    startFailed = true;
    return false;
  }
  startFailed = false;
  if (recovering) {
    recoveryCount++;
    recovering = false;
  }
  return true;
}

//*****************************************************************************
tNMEA2000_esp32_idf_twai::Status tNMEA2000_esp32_idf_twai::GetStatus() const {
  Status st;
  memset(&st, 0, sizeof(st));
  st.state = TWAI_STATE_STOPPED;

  twai_status_info_t s;
  if (installed && twai_get_status_info(&s) == ESP_OK) {
    st.state = s.state;
    st.txErrorCounter = s.tx_error_counter;
    st.rxErrorCounter = s.rx_error_counter;
    st.txFailed = s.tx_failed_count;
    st.rxMissed = s.rx_missed_count;
    st.rxOverrun = s.rx_overrun_count;
    st.arbLost = s.arb_lost_count;
    st.busErrors = s.bus_error_count;
  }
  st.busOffCount = busOffCount;
  st.recoveryCount = recoveryCount;
  st.restartFailures = restartFailures;
  st.errorWarningCount = errorWarningCount;
  st.errorPassiveCount = errorPassiveCount;
  st.droppedFrames = droppedFrames;
  return st;
}
