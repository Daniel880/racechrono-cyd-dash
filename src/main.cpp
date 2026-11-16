#include <Arduino.h>

#include "Dashboard.h"
#include "GpsReceiver.h"
#include "GpxLogger.h"
#include "RaceChronoBle.h"

#ifdef SIM_GPS
#include "GpsSim.h"
#endif

#include "LGFX_CYD.h"
static LGFX lcd;

static constexpr const char* kDeviceName = "RC DIY CYD";
static constexpr uint32_t kRefreshMs = 100;  // 10 Hz - tyle, ile wysyla RaceChrono

static void readValues(DashData& d) {
  d.connected = rc::isConnected();
  d.configured = rc::isConfigured();
  d.hasDelta = rc::get(rc::DELTA, d.deltaMs);
  d.hasCurrent = rc::get(rc::LAP_TIME, d.currentMs);
  d.hasLast = rc::get(rc::LAST_LAP, d.lastMs);
  d.hasBest = rc::get(rc::BEST_LAP, d.bestMs);
  d.hasLapNumber = rc::get(rc::LAP_NUMBER, d.lapNumber);
  d.hasSpeed = rc::get(rc::SPEED, d.speedKmhX10);
  d.hasBestLapNumber = rc::get(rc::BEST_LAP_NUMBER, d.bestLapNumber);
  d.hasRef = rc::get(rc::REF_LAP, d.refMs);
  d.hasRefLapNumber = rc::get(rc::REF_LAP_NUMBER, d.refLapNumber);
  d.hasLastLapNumber = rc::get(rc::LAST_LAP_NUMBER, d.lastLapNumber);
  d.hasDeltaSpeed = rc::get(rc::DELTA_SPEED, d.deltaSpeedKmhX10);
  d.gpsState = (int8_t)gpsrx::state();
  d.sdState = (int8_t)gpx::state();
}

// Pozycja z modulu GPS (lub symulatora): do RaceChrono i do kolejki zapisu na SD.
static void handleFix(const rc::GpsFix& fix) {
  rc::sendGps(fix);
  gpx::addFix(fix);
}

static void gpsTask(void*) {
  for (;;) {
    rc::GpsFix fix;
    if (gpsrx::update(fix)) handleFix(fix);
#ifdef SIM_GPS
    if (gpssim::update(fix)) handleFix(fix);
#endif
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup() {
  Serial.begin(115200);

  // Dioda RGB na CYD jest aktywna stanem niskim - wylaczamy ja.
  for (int pin : {4, 16, 17}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  lcd.init();
  lcd.setRotation(2);  // pionowo, 240x320 (obrot o 90 st. w prawo wzgledem dawnego 1)
  lcd.setBrightness(255);
  dash::begin(lcd);

  rc::begin(kDeviceName);
  gpx::begin();    // backup na SD we wlasnym zadaniu (tylko z -DGPX_LOG)
  gpsrx::begin();  // modul GPS na UART (tylko z -DGPS_UART_RX)

#ifdef SIM_GPS
  gpssim::begin();
#endif

  // GPS w osobnym zadaniu o wyzszym priorytecie niz loop(): rysowanie ekranu
  // i zapis na SD nie opozniaja wysylki pozycji do RaceChrono.
  xTaskCreatePinnedToCore(gpsTask, "gps", 8192, nullptr, 2, nullptr, ARDUINO_RUNNING_CORE);
  Serial.printf("[SYS] po starcie: wolny RAM %u B, najwiekszy blok %u B\n", (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap());
}

void loop() {
  rc::loop();

  static uint32_t lastDraw = 0;
  if (millis() - lastDraw < kRefreshMs) {
    delay(1);  // oddaj procesor (GPS, BLE) zamiast krecic sie w petli
    return;
  }
  lastDraw = millis();

  DashData d{};
  readValues(d);
  dash::update(d);
}
