#pragma once
#include <stdint.h>

class LGFX;

struct DashData {
  bool connected;
  bool configured;
  bool hasDelta;
  int32_t deltaMs;
  bool hasCurrent;
  int32_t currentMs;
  bool hasLast;
  int32_t lastMs;
  bool hasBest;
  int32_t bestMs;
  bool hasLapNumber;
  int32_t lapNumber;
  bool hasSpeed;
  int32_t speedKmhX10;
  bool hasBestLapNumber;
  int32_t bestLapNumber;
  bool hasRef;  // okrazenie porownawcze (do niego liczona jest delta)
  int32_t refMs;
  bool hasRefLapNumber;
  int32_t refLapNumber;
  bool hasLastLapNumber;
  int32_t lastLapNumber;
  bool hasDeltaSpeed;
  int32_t deltaSpeedKmhX10;  // + = szybciej niz okrazenie porownawcze
  int8_t gpsState;  // gpsrx::State: 0 brak modulu, 1 brak danych, 2 brak fixa, 3 fix
  int8_t sdState;   // gpx::State: 0 wylaczone, -1 blad, 1 czeka na fix, 2 zapis
};

namespace dash {
void begin(LGFX& lcd);
void update(const DashData& d);  // przerysowuje tylko zmienione pola
}  // namespace dash
