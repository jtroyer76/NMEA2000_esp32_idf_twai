// Host tests for tNMEA2000_esp32_idf_twai against the fake driver/twai.h in test/fakes.
// Run with `pio test -e native`.

#include <unity.h>
#include "NMEA2000_esp32_idf_twai.h"

// The NMEA2000 library needs the application to provide millis() off Arduino; the tests drive it.
static uint32_t nowMs = 1000;
extern "C" uint32_t millis() { return nowMs; }

// Exposes the protected driver interface the library normally calls.
class TestDriver : public tNMEA2000_esp32_idf_twai {
public:
  TestDriver(twai_mode_t mode = TWAI_MODE_NORMAL) : tNMEA2000_esp32_idf_twai(GPIO_NUM_22, GPIO_NUM_19, mode) {}
  using tNMEA2000_esp32_idf_twai::InitCANFrameBuffers;
  using tNMEA2000_esp32_idf_twai::CANOpen;
  using tNMEA2000_esp32_idf_twai::CANSendFrame;
  using tNMEA2000_esp32_idf_twai::CANGetFrame;
  uint16_t librarySendFrames() const { return MaxCANSendFrames; }
};

static TestDriver *drv = nullptr;

void setUp() {
  fakeTwai.reset();
  nowMs = 1000;
  delete drv;
  drv = new TestDriver();
}

void tearDown() {}

static void openRunning() {
  drv->InitCANFrameBuffers();
  TEST_ASSERT_TRUE(drv->CANOpen());
  TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, fakeTwai.state);
}

static twai_message_t frame(uint32_t id, bool extd, bool rtr = false) {
  twai_message_t m = {};
  m.extd = extd;
  m.rtr = rtr;
  m.identifier = id;
  m.data_length_code = 3;
  m.data[0] = 0xA1; m.data[1] = 0xB2; m.data[2] = 0xC3;
  return m;
}

// Calls CANGetFrame after the supervision interval has passed.
static bool getFrameLater(unsigned long &id, unsigned char &len, unsigned char *buf, uint32_t ms = 10) {
  nowMs += ms;
  return drv->CANGetFrame(id, len, buf);
}

static void superviseLater(uint32_t ms = 10) {
  unsigned long id; unsigned char len; unsigned char buf[8];
  getFrameLater(id, len, buf, ms);
}

//*****************************************************************************
// Buffers and open

void test_open_installs_250k_accept_all_and_starts() {
  openRunning();
  TEST_ASSERT_EQUAL(1, fakeTwai.installCalls);
  TEST_ASSERT_EQUAL(TWAI_MODE_NORMAL, fakeTwai.gconfig.mode);
  TEST_ASSERT_EQUAL(GPIO_NUM_22, fakeTwai.gconfig.tx_io);
  TEST_ASSERT_EQUAL(GPIO_NUM_19, fakeTwai.gconfig.rx_io);
  TEST_ASSERT_EQUAL_UINT32(16, fakeTwai.tconfig.brp);  // TWAI_TIMING_CONFIG_250KBITS
  TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFF, fakeTwai.fconfig.acceptance_mask);
}

// Defaults as NMEA2000_esp32: library send 40 / receive 0 -> RX 50, TX 36, library keeps 4.
void test_default_buffer_sizes_follow_timo_driver() {
  openRunning();
  TEST_ASSERT_EQUAL_UINT32(50, fakeTwai.gconfig.rx_queue_len);
  TEST_ASSERT_EQUAL_UINT32(36, fakeTwai.gconfig.tx_queue_len);
  TEST_ASSERT_EQUAL_UINT16(4, drv->librarySendFrames());
}

void test_buffer_sizes_follow_library_settings() {
  drv->SetN2kCANReceiveFrameBufSize(80);
  drv->SetN2kCANSendFrameBufSize(64);
  openRunning();
  TEST_ASSERT_EQUAL_UINT32(80, fakeTwai.gconfig.rx_queue_len);
  TEST_ASSERT_EQUAL_UINT32(60, fakeTwai.gconfig.tx_queue_len);
  TEST_ASSERT_EQUAL_UINT16(4, drv->librarySendFrames());
}

void test_only_error_alerts_enabled() {
  openRunning();
  uint32_t a = fakeTwai.gconfig.alerts_enabled;
  TEST_ASSERT_TRUE(a & TWAI_ALERT_ABOVE_ERR_WARN);
  TEST_ASSERT_TRUE(a & TWAI_ALERT_ERR_PASS);
  TEST_ASSERT_FALSE_MESSAGE(a & (TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_IDLE),
                            "data alerts must stay off (no per-frame work)");
}

// The case that wedges NMEA2000_esp32_twai: install succeeds, start fails, the library retries.
void test_open_retry_after_failed_start_does_not_reinstall() {
  fakeTwai.startFailures = 1;
  drv->InitCANFrameBuffers();
  TEST_ASSERT_FALSE(drv->CANOpen());
  TEST_ASSERT_EQUAL_UINT32(1, drv->GetStatus().restartFailures);
  nowMs += 1000;  // the library retries CANOpen() every second
  TEST_ASSERT_TRUE(drv->CANOpen());
  TEST_ASSERT_EQUAL(1, fakeTwai.installCalls);
  TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, fakeTwai.state);
}

void test_open_fails_when_install_fails_and_retries() {
  fakeTwai.installResult = ESP_FAIL;
  drv->InitCANFrameBuffers();
  TEST_ASSERT_FALSE(drv->CANOpen());
  fakeTwai.installResult = ESP_OK;
  nowMs += 1000;
  TEST_ASSERT_TRUE(drv->CANOpen());
}

void test_open_is_idempotent() {
  openRunning();
  TEST_ASSERT_TRUE(drv->CANOpen());
  TEST_ASSERT_EQUAL(1, fakeTwai.installCalls);
}

//*****************************************************************************
// Send and receive

void test_send_queues_extended_frame_without_blocking() {
  openRunning();
  const unsigned char data[3] = {1, 2, 3};
  TEST_ASSERT_TRUE(drv->CANSendFrame(0x09F80123, 3, data, true));
  TEST_ASSERT_EQUAL(1, fakeTwai.tx.size());
  const twai_message_t &m = fakeTwai.tx[0];
  TEST_ASSERT_EQUAL(1, m.extd);
  TEST_ASSERT_EQUAL(0, m.rtr);
  TEST_ASSERT_EQUAL(0, m.self);
  TEST_ASSERT_EQUAL_HEX32(0x09F80123, m.identifier);
  TEST_ASSERT_EQUAL(3, m.data_length_code);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(data, m.data, 3);
  TEST_ASSERT_EQUAL_UINT32(0, fakeTwai.maxTicks);
}

void test_send_false_when_not_running() {
  openRunning();
  fakeTwai.state = TWAI_STATE_BUS_OFF;
  const unsigned char data[1] = {0};
  TEST_ASSERT_FALSE(drv->CANSendFrame(0x123, 1, data, false));
}

void test_send_false_when_queue_full() {
  openRunning();
  fakeTwai.transmitResult = ESP_ERR_TIMEOUT;
  const unsigned char data[1] = {0};
  TEST_ASSERT_FALSE(drv->CANSendFrame(0x123, 1, data, false));
}

void test_no_ack_mode_sends_self_reception() {
  delete drv;
  drv = new TestDriver(TWAI_MODE_NO_ACK);
  openRunning();
  const unsigned char data[1] = {0};
  TEST_ASSERT_TRUE(drv->CANSendFrame(0x123, 1, data, false));
  TEST_ASSERT_EQUAL(1, fakeTwai.tx[0].self);
}

void test_receive_extended_frame() {
  openRunning();
  fakeTwai.rx.push_back(frame(0x09F80123, true));
  unsigned long id; unsigned char len; unsigned char buf[8];
  TEST_ASSERT_TRUE(drv->CANGetFrame(id, len, buf));
  TEST_ASSERT_EQUAL_HEX32(0x09F80123, id);
  TEST_ASSERT_EQUAL(3, len);
  TEST_ASSERT_EQUAL_HEX8(0xC3, buf[2]);
  TEST_ASSERT_FALSE(drv->CANGetFrame(id, len, buf));
  TEST_ASSERT_EQUAL_UINT32(0, fakeTwai.maxTicks);
}

void test_receive_drops_standard_and_remote_frames() {
  openRunning();
  fakeTwai.rx.push_back(frame(0x123, false));
  fakeTwai.rx.push_back(frame(0x09F80123, true, true));
  fakeTwai.rx.push_back(frame(0x0DF80456, true));
  unsigned long id; unsigned char len; unsigned char buf[8];
  TEST_ASSERT_TRUE(drv->CANGetFrame(id, len, buf));
  TEST_ASSERT_EQUAL_HEX32(0x0DF80456, id);
  TEST_ASSERT_EQUAL_UINT32(2, drv->GetStatus().droppedFrames);
}

void test_receive_clamps_length_to_8() {
  openRunning();
  twai_message_t m = frame(0x09F80123, true);
  m.data_length_code = 15;  // DLC > 8 is legal on the wire, data is still 8 bytes
  fakeTwai.rx.push_back(m);
  unsigned long id; unsigned char len; unsigned char buf[8];
  TEST_ASSERT_TRUE(drv->CANGetFrame(id, len, buf));
  TEST_ASSERT_EQUAL(8, len);
}

//*****************************************************************************
// Supervision

void test_bus_off_recovery_and_restart() {
  openRunning();
  fakeTwai.state = TWAI_STATE_BUS_OFF;
  superviseLater();
  TEST_ASSERT_EQUAL(1, fakeTwai.recoveryCalls);
  TEST_ASSERT_EQUAL(TWAI_STATE_RECOVERING, fakeTwai.state);
  TEST_ASSERT_EQUAL_UINT32(1, drv->GetStatus().busOffCount);

  superviseLater();  // still recovering: wait
  TEST_ASSERT_EQUAL(TWAI_STATE_RECOVERING, fakeTwai.state);

  fakeTwai.state = TWAI_STATE_STOPPED;  // recovery complete
  superviseLater();
  TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, fakeTwai.state);
  TEST_ASSERT_EQUAL_UINT32(1, drv->GetStatus().recoveryCount);
  TEST_ASSERT_EQUAL_UINT32(1, drv->GetStatus().busOffCount);
}

// The second case that wedges NMEA2000_esp32_twai.
void test_failed_start_after_recovery_retries_with_backoff() {
  openRunning();
  fakeTwai.state = TWAI_STATE_BUS_OFF;
  superviseLater();
  fakeTwai.state = TWAI_STATE_STOPPED;
  fakeTwai.startFailures = 1;
  superviseLater();
  TEST_ASSERT_EQUAL(TWAI_STATE_STOPPED, fakeTwai.state);
  TEST_ASSERT_EQUAL_UINT32(1, drv->GetStatus().restartFailures);

  int starts = fakeTwai.startCalls;
  superviseLater(10);  // inside the back-off: no attempt
  TEST_ASSERT_EQUAL(starts, fakeTwai.startCalls);

  superviseLater(tNMEA2000_esp32_idf_twai::RestartBackoffMs);
  TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, fakeTwai.state);
  TEST_ASSERT_EQUAL_UINT32(1, drv->GetStatus().recoveryCount);
}

void test_unexpected_stop_is_restarted() {
  openRunning();
  fakeTwai.state = TWAI_STATE_STOPPED;
  superviseLater();
  TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, fakeTwai.state);
  TEST_ASSERT_EQUAL_UINT32(0, drv->GetStatus().recoveryCount);  // not a bus-off recovery
}

void test_supervision_is_rate_limited() {
  openRunning();
  unsigned long id; unsigned char len; unsigned char buf[8];
  nowMs += 10;
  drv->CANGetFrame(id, len, buf);
  int calls = fakeTwai.statusCalls;
  drv->CANGetFrame(id, len, buf);
  drv->CANGetFrame(id, len, buf);
  TEST_ASSERT_EQUAL(calls, fakeTwai.statusCalls);
  nowMs += tNMEA2000_esp32_idf_twai::SuperviseIntervalMs;
  drv->CANGetFrame(id, len, buf);
  TEST_ASSERT_EQUAL(calls + 1, fakeTwai.statusCalls);
}

void test_supervision_survives_millis_wraparound() {
  nowMs = 0xFFFFFFF0;
  openRunning();
  fakeTwai.state = TWAI_STATE_BUS_OFF;
  superviseLater(0x20);  // wraps past zero
  TEST_ASSERT_EQUAL(TWAI_STATE_RECOVERING, fakeTwai.state);
}

void test_error_alerts_are_counted() {
  openRunning();
  fakeTwai.pendingAlerts = TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS;
  superviseLater();
  tNMEA2000_esp32_idf_twai::Status s = drv->GetStatus();
  TEST_ASSERT_EQUAL_UINT32(1, s.errorWarningCount);
  TEST_ASSERT_EQUAL_UINT32(1, s.errorPassiveCount);
  TEST_ASSERT_EQUAL_UINT32(0, fakeTwai.maxTicks);
}

//*****************************************************************************
// Status

void test_status_reports_driver_counters() {
  openRunning();
  fakeTwai.counters.tx_error_counter = 7;
  fakeTwai.counters.rx_error_counter = 9;
  fakeTwai.counters.tx_failed_count = 2;
  fakeTwai.counters.rx_missed_count = 3;
  fakeTwai.counters.rx_overrun_count = 4;
  fakeTwai.counters.arb_lost_count = 5;
  fakeTwai.counters.bus_error_count = 6;
  tNMEA2000_esp32_idf_twai::Status s = drv->GetStatus();
  TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, s.state);
  TEST_ASSERT_EQUAL_UINT32(7, s.txErrorCounter);
  TEST_ASSERT_EQUAL_UINT32(9, s.rxErrorCounter);
  TEST_ASSERT_EQUAL_UINT32(2, s.txFailed);
  TEST_ASSERT_EQUAL_UINT32(3, s.rxMissed);
  TEST_ASSERT_EQUAL_UINT32(4, s.rxOverrun);
  TEST_ASSERT_EQUAL_UINT32(5, s.arbLost);
  TEST_ASSERT_EQUAL_UINT32(6, s.busErrors);
}

void test_status_before_open() {
  tNMEA2000_esp32_idf_twai::Status s = drv->GetStatus();
  TEST_ASSERT_EQUAL(TWAI_STATE_STOPPED, s.state);
  TEST_ASSERT_EQUAL_UINT32(0, s.txErrorCounter);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_open_installs_250k_accept_all_and_starts);
  RUN_TEST(test_default_buffer_sizes_follow_timo_driver);
  RUN_TEST(test_buffer_sizes_follow_library_settings);
  RUN_TEST(test_only_error_alerts_enabled);
  RUN_TEST(test_open_retry_after_failed_start_does_not_reinstall);
  RUN_TEST(test_open_fails_when_install_fails_and_retries);
  RUN_TEST(test_open_is_idempotent);
  RUN_TEST(test_send_queues_extended_frame_without_blocking);
  RUN_TEST(test_send_false_when_not_running);
  RUN_TEST(test_send_false_when_queue_full);
  RUN_TEST(test_no_ack_mode_sends_self_reception);
  RUN_TEST(test_receive_extended_frame);
  RUN_TEST(test_receive_drops_standard_and_remote_frames);
  RUN_TEST(test_receive_clamps_length_to_8);
  RUN_TEST(test_bus_off_recovery_and_restart);
  RUN_TEST(test_failed_start_after_recovery_retries_with_backoff);
  RUN_TEST(test_unexpected_stop_is_restarted);
  RUN_TEST(test_supervision_is_rate_limited);
  RUN_TEST(test_supervision_survives_millis_wraparound);
  RUN_TEST(test_error_alerts_are_counted);
  RUN_TEST(test_status_reports_driver_counters);
  RUN_TEST(test_status_before_open);
  return UNITY_END();
}
