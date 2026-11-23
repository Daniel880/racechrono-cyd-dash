#pragma once
// Urzadzenie RaceChrono DIY BLE - tylko Monitor API (RaceChrono -> ESP32).
// Specyfikacja: https://github.com/aollin/racechrono-ble-diy-device

#include <Arduino.h>

namespace rc {

// Kolejnosc = Monitor ID wysylane do RaceChrono. Wzory w RaceChronoBle.cpp.
enum MonitorId : uint8_t {
  DELTA = 0,   // delta do okrazenia porownawczego (REF) [ms]
  LAP_TIME,    // aktualne okrazenie [ms]
  LAST_LAP,    // poprzednie okrazenie [ms]
  BEST_LAP,    // najlepsze okrazenie [ms]
  LAP_NUMBER,  // numer okrazenia
  SPEED,       // predkosc GPS [km/h * 10]
  BEST_LAP_NUMBER,  // numer najlepszego okrazenia
  REF_LAP,          // okrazenie porownawcze, do ktorego liczona jest delta [ms]
  REF_LAP_NUMBER,   // numer okrazenia porownawczego
  LAST_LAP_NUMBER,  // numer poprzedniego okrazenia
  DELTA_SPEED,      // delta predkosci do okrazenia porownawczego [km/h * 10], + = szybciej
  MONITOR_COUNT
};

void begin(const char* deviceName);
void loop();  // wywolywac w loop(): wysyla konfiguracje monitorow po polaczeniu

bool isConnected();
bool isConfigured();
bool get(MonitorId id, int32_t& out);  // false = brak waznej wartosci

// GPS -> RaceChrono (charakterystyki 0x0003 / 0x0004). Wolac tylko z jednego zadania.
struct GpsFix {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  uint16_t millis;
  double lat, lon;  // stopnie
  float altM;
  float speedKmh;
  float bearingDeg;
  uint8_t satellites;
  uint8_t fixQuality;  // 0 = brak, 1 = GPS, 2 = DGPS
  float hdop, vdop;
};
void sendGps(const GpsFix& fix);
// Pozycje wyslane do RaceChrono od startu (tylko przy polaczeniu) i ile z nich sie nie udalo
// (brak miejsca w buforach Bluetooth) - diagnostyka na stronie /gps.
struct GpsTx {
  uint32_t sent, failed;
};
GpsTx gpsTx();

}  // namespace rc
