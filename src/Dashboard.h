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
  uint8_t gpsSats;      // satelity uzyte w rozwiazaniu (przed fixem zwykle 0-3)
  int16_t gpsAccX10;    // szacowany blad pozycji [m * 10] (NAV-PVT hAcc), 0 = brak
  uint8_t gpsCno;       // sila sygnalu 4 najmocniejszych satelitow [dBHz], 0 = brak
  int8_t sdState;   // gpx::State: 0 wylaczone, -1 blad, 1 czeka na fix, 2 zapis
  int8_t otaState;  // ota::State: 0 WiFi wylaczone, 1 czeka na podlaczenie, 2 ktos podlaczony
  bool safeMode;    // tryb awaryjny: tylko ekran i WiFi (SysLog)
};

namespace dash {
void begin(LGFX& lcd);
void update(const DashData& d);       // przerysowuje tylko zmienione pola
void showOtaProgress(int percent);    // caly ekran: postep wgrywania przez WiFi (0..100)
// Test zaklocen GPS: przez ms ekran bez podswietlenia i bez rysowania (zero transmisji do ekranu).
// Wolac z tego samego zadania co update() (loop).
void blankFor(uint32_t ms);
uint32_t blankLeftMs();
}  // namespace dash
