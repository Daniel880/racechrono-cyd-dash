#pragma once
// Modul GPS u-blox M10 na UART, protokol UBX. Kazda nowa pozycja (UBX-NAV-PVT) trafia jako
// rc::GpsFix, a kazda ramka UBX z modulu do gpx::ubxFrame() (surowy zapis na SD do analizy).

#include <stdint.h>

#include "RaceChronoBle.h"  // rc::GpsFix

namespace gpsrx {

enum class State : int8_t { Off = 0, NoData = 1, NoFix = 2, Fix = 3 };

// Jakosc odbioru do wskaznika na ekranie
struct Quality {
  uint8_t used;  // satelity uzyte w rozwiazaniu (NAV-PVT numSV), 0 = brak danych
  float hdop;    // NAV-DOP, 0 = brak (bez fixa)
  float accM;    // szacowany blad pozycji poziomej [m] (NAV-PVT hAcc), 0 = brak (bez fixa)
};

// ---------- Diagnostyka (strona /gps) ----------
constexpr int kMaxSats = 64;
struct SatInfo {
  uint8_t gnss;   // 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS
  uint8_t sv;
  uint8_t cno;    // sila sygnalu C/N0 [dBHz], 0 = brak
  int8_t elev;    // wysokosc nad horyzontem [stopnie], poza +-90 = nieznana
  uint16_t azim;
  uint8_t flags;  // bit 0 uzyty w pozycji, bity 1-3 jakosc sygnalu (0-7),
                  // bity 4-5 orbita: 0 brak, 1 efemerydy, 2 almanach, 3 inna
};
struct Diag {
  uint8_t hz;
  const char* modeName;
  bool running;          // modul skonfigurowany, normalna praca
  uint8_t rate[60];      // epoki w kolejnych pelnych sekundach (wg czasu z modulu), najstarsza pierwsza
  uint8_t rateLen;
  uint32_t epochs;       // epoki od startu (w normalnej pracy)
  uint32_t missed;       // zgubione epoki: luka w czasie z modulu wieksza niz okres pomiaru
  uint32_t gaps;         // przerwy w danych dluzsze niz 3 s
  uint32_t firstFixMs;   // pierwszy fix: ms od startu ESP, 0 = jeszcze nie bylo
  uint32_t fixLost;      // ile razy fix zniknal
  // Z komunikatow UBX raz na sekunde. Wiek w ms, UINT32_MAX = brak danych.
  uint32_t satAge, rfAge, navAge;
  uint8_t numSats;
  SatInfo sats[kMaxSats];
  uint16_t noise;        // MON-RF noisePerMS: szum mierzony przez odbiornik
  uint16_t agc;          // MON-RF agcCnt (0-8191): wzmocnienie wejscia
  uint8_t cw;            // MON-RF cwSuppression: 0 = brak zaklocen waskopasmowych, 255 = silne
  uint8_t jam;           // MON-RF jammingState: 0 nieznany, 1 OK, 2 ostrzezenie, 3 krytyczny
  uint8_t ant;           // MON-RF antStatus: 0 init, 1 nieznany, 2 OK, 3 zwarcie, 4 rozwarcie
  uint8_t navFix;        // NAV-STATUS gpsFix
  uint32_t navTtff;      // NAV-STATUS: TTFF wg modulu [ms]
  uint32_t navMsss;      // NAV-STATUS: czas od startu modulu [ms]
};
void stop();           // przed wgrywaniem programu: konczy UART (update() juz nie wolac)
void diag(Diag& out);  // kopia z innego zadania (strona WWW)

void begin(uint8_t gpsMode);  // 25 / 20 / 10 - patrz Settings.h
bool update(rc::GpsFix& out);  // true = nowa pozycja (jedna na epoke GPS, tylko z aktualnym fixem)
// Stan i jakosc sa liczone w update(), wiec mozna je czytac z innego zadania (ekran).
State state();
Quality quality();

}  // namespace gpsrx
