// Minimal node: claims an address, sends a heartbeat, prints driver status every 10 s.
// Also the compile check for `pio run -e m5stack-atom`.
#include <Arduino.h>
#include <NMEA2000_esp32_idf_twai.h>

tNMEA2000_esp32_idf_twai NMEA2000(GPIO_NUM_22, GPIO_NUM_19);  // M5Stack Atom Lite + CAN Unit

void setup() {
  Serial.begin(115200);
  NMEA2000.SetProductInformation("1", 1, "TWAI driver check", "0.1.0", "M5 Atom");
  NMEA2000.SetDeviceInformation(1, 130, 25, 2046);
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 32);
  NMEA2000.Open();
}

void loop() {
  NMEA2000.ParseMessages();

  static uint32_t last = 0;
  if (millis() - last >= 10000) {
    last = millis();
    tNMEA2000_esp32_idf_twai::Status s = NMEA2000.GetStatus();
    Serial.printf("state=%d tec=%u rec=%u busOff=%u recovered=%u restartFail=%u missed=%u overrun=%u\n",
                  s.state, s.txErrorCounter, s.rxErrorCounter, s.busOffCount, s.recoveryCount,
                  s.restartFailures, s.rxMissed, s.rxOverrun);
  }
}
