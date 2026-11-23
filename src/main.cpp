#include <Arduino.h>

#include "Dashboard.h"
#include "GpsReceiver.h"
#include "GpxLogger.h"
#include "OtaUpdate.h"
#include "RaceChronoBle.h"
#include "Settings.h"
#include "SysLog.h"

#ifdef SIM_GPS
#include "GpsSim.h"
#endif

#if defined(SIM_GPS) && defined(GPS_UART_RX)
#error "SIM_GPS razem z GPS_UART_RX: RaceChrono dostaloby pozycje z dwoch zrodel naraz - zostaw jedno"
#endif

#include "LGFX_CYD.h"
static LGFX lcd;

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
  gpsrx::Quality q = gpsrx::quality();
  d.gpsSats = q.used;
  d.gpsHdopX10 = (int16_t)lroundf(min(q.hdop, 99.9f) * 10.0f);
  d.gpsAccX10 = (int16_t)lroundf(min(q.accM, 99.9f) * 10.0f);
  d.sdState = (int8_t)gpx::state();
  d.otaState = (int8_t)ota::state();
}

// Pozycja z modulu GPS (lub symulatora): do RaceChrono i do kolejki zapisu na SD.
static void handleFix(const rc::GpsFix& fix) {
  rc::sendGps(fix);
  gpx::addFix(fix);
}

static TaskHandle_t gpsTaskHandle = nullptr;
static volatile bool gpsStopReq = false;  // tryb wgrywania: zadanie GPS konczy sie samo
static volatile bool gpsStopped = false;

static void gpsTask(void*) {
  for (;;) {
    if (gpsStopReq) {
      gpsStopped = true;
      vTaskDelete(nullptr);
    }
    rc::GpsFix fix;
    if (gpsrx::update(fix)) handleFix(fix);
#ifdef SIM_GPS
    if (gpssim::update(fix)) handleFix(fix);
#endif
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void logHeap(const char* after);

// Tryb wgrywania programu przez WiFi: zwalnia RAM (~27 kB), po wgraniu i tak jest restart.
// Najpierw GPS (doklada do kolejki SD), potem karta. Bluetooth zostaje: wylaczanie NimBLE
// (deinit) w trakcie pracy jest ryzykowne, a bez niego RAM-u i tak wystarcza.
static void enterUpdateMode() {
  syslog::logf("[OTA] tryb wgrywania: zatrzymuje GPS i zapis SD\n");
  gpsStopReq = true;
  for (int i = 0; i < 50 && gpsTaskHandle && !gpsStopped; i++) delay(10);
  gpsrx::stop();
  syslog::logf("[OTA] GPS zatrzymany%s\n", gpsStopped || !gpsTaskHandle ? "" : " (zadanie nie odpowiedzialo)");
  gpx::shutdown();
  logHeap("wejsciu w tryb wgrywania");
}

static void logHeap(const char* after) {
  syslog::logf("[SYS] po %s: wolny RAM %u B, najwiekszy blok %u B\n", after, (unsigned)ESP.getFreeHeap(),
               (unsigned)ESP.getMaxAllocHeap());
}

void setup() {
  Serial.begin(115200);
  syslog::begin();  // przyczyna restartu, licznik awarii (tryb awaryjny bez SD), log w pamieci RTC

  // Dioda RGB na CYD jest aktywna stanem niskim - wylaczamy ja.
  for (int pin : {4, 16, 17}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  lcd.init();
  lcd.setRotation(2);  // pionowo, 240x320 (obrot o 90 st. w prawo wzgledem dawnego 1)
  lcd.setBrightness(255);
  dash::begin(lcd);
  logHeap("ekranie");

  // nazwa Bluetooth i tryb GPS - ustawiane na stronie WiFi, zapisane w pamieci NVS
  const settings::Settings cfg = settings::load();
  rc::begin(cfg.btName);
  logHeap("Bluetooth");
  // WiFi do wgrywania tylko przez chwile po starcie (tylko z -DOTA_WIFI), potem wylaczone.
  // Przed karta SD: przy braku RAM-u start WiFi konczy sie bledem programu, a karta SD
  // tylko stanem Error (i probuje znowu, gdy WiFi sie wylaczy i zwolni pamiec).
  ota::begin(dash::showOtaProgress, enterUpdateMode);
  logHeap("WiFi");
  gpx::begin(!syslog::safeMode());  // backup na SD we wlasnym zadaniu (tylko z -DGPX_LOG)
  logHeap("karcie SD");
  gpsrx::begin(cfg.gpsMode);  // modul GPS na UART (tylko z -DGPS_UART_RX)

#ifdef SIM_GPS
  gpssim::begin();
#endif

  // GPS w osobnym zadaniu o wyzszym priorytecie niz loop(): rysowanie ekranu
  // i zapis na SD nie opozniaja wysylki pozycji do RaceChrono.
  xTaskCreatePinnedToCore(gpsTask, "gps", 8192, nullptr, 2, &gpsTaskHandle, ARDUINO_RUNNING_CORE);
  logHeap("GPS (koniec startu)");
}

void loop() {
  syslog::loop();
  // raz, po 30 s: zapas stosow (do strojenia pamieci)
  static bool stacksLogged = false;
  if (!stacksLogged && millis() > 30000) {
    stacksLogged = true;
    syslog::logf("[SYS] wolny stos: gps %u B, loop %u B; wolny RAM %u B (minimum %u B)\n",
                 gpsTaskHandle ? (unsigned)uxTaskGetStackHighWaterMark(gpsTaskHandle) : 0u,
                 (unsigned)uxTaskGetStackHighWaterMark(nullptr), (unsigned)ESP.getFreeHeap(),
                 (unsigned)ESP.getMinFreeHeap());
  }
  ota::loop();
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
