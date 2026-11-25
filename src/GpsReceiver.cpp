#include "GpsReceiver.h"
#include "SysLog.h"

#ifdef GPS_UART_RX

#include <Arduino.h>

#include "GpxLogger.h"

#ifndef GPS_UART_TX
#define GPS_UART_TX -1
#endif
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD 115200  // docelowa predkosc UART
#endif
namespace gpsrx {
namespace {

// Tryb GPS = czestotliwosc + systemy satelitarne (limity u-blox M10, 25 i 20 Hz wymagaja trybu
// high performance - GEP-M10 ma go fabrycznie). Wybierany na stronie WiFi (Settings).
struct Mode {
  uint8_t hz;
  uint32_t galileo, beidou;  // 1 = wlaczony (BeiDou: sygnal B1C, domyslny w M10)
  const char* name;
};

Mode modeFor(uint8_t m) {
  switch (m) {
    case 25:
      return {25, 0, 0, "GPS"};  // najwiecej punktow, najmniej satelitow
    case 10:
      return {10, 1, 1, "GPS+Galileo+BeiDou"};  // najwiecej satelitow, najlepszy HDOP
    default:
      return {20, 1, 0, "GPS+Galileo"};  // kompromis
  }
}

Mode mode = modeFor(20);

constexpr uint32_t kNoDataTimeoutMs = 3000;  // tyle bez NAV-PVT = modul zniknal albo zmienil predkosc
constexpr uint32_t kMaxFixAgeMs = 1500;
constexpr uint32_t kDopMaxAgeMs = 1000;
constexpr int32_t kWeekMs = 604800000;  // iTOW liczy ms od poczatku tygodnia GPS

// Ostatnia epoka z NAV-PVT / NAV-DOP. Zmienia i czyta tylko zadanie GPS.
uint32_t pvtCount = 0;   // poprawne NAV-PVT od startu
uint32_t lastPvtMs = 0;  // 0 = jeszcze zadnego
uint32_t lastFixMs = 0;
uint8_t pvtSats = 0;     // numSV: satelity uzyte w rozwiazaniu (bez limitu 12 jak w GGA)
float pvtAccM = 0.0f;    // hAcc: szacowany blad pozycji poziomej - z jakosci pomiarow, nie tylko geometrii
float dopH = 0.0f, dopV = 0.0f;
uint32_t dopMs = 0;
rc::GpsFix pending{};     // pozycja z ostatniej epoki z fixem, czeka na update()
bool hasPending = false;

// Diagnostyka dla strony /gps. Pisze zadanie GPS, czyta strona (diag()) - pod diagMux.
portMUX_TYPE diagMux = portMUX_INITIALIZER_UNLOCKED;
Diag live{};
uint32_t satMs = 0, rfMs = 0, navMs = 0;  // kiedy przyszly ostatnie NAV-SAT / MON-RF / NAV-STATUS
// Epoki liczone z iTOW (czas GPS epoki), nie z chwili odbioru, wiec opoznienia UART i zadan ich nie psuja.
bool haveItow = false;
uint32_t lastItow = 0;
int32_t curSec = -1;  // biezaca sekunda (z iTOW) i liczba epok w niej
uint8_t epochsInSec = 0;
bool skipFirstSec = true;  // pierwsza sekunda po (re)starcie liczenia jest niepelna
bool fixNow = false;
uint32_t fixLostMs = 0;
bool ttffLogPending = false;  // po pierwszym fixie: najblizszy NAV-STATUS idzie do logu
bool commsWarned = false;     // przepelnienie bufora nadawczego modulu juz zalogowane

// Ekran (inne zadanie) dostaje stan i jakosc przez te kopie, odswiezane w kazdym update().
volatile State stateSnap = State::NoData;
volatile uint8_t qUsed = 0;
volatile float qHdop = 0.0f, qAcc = 0.0f;

uint16_t u16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
uint32_t u32(const uint8_t* p) { return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
int32_t i32(const uint8_t* p) { return (int32_t)u32(p); }

// ---------- Data UTC: dni od 1970-01-01 i z powrotem (algorytm H. Hinnanta) ----------
int32_t daysFromCivil(int32_t y, uint32_t m, uint32_t d) {
  y -= m <= 2;
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = (uint32_t)(y - era * 400);
  const uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

void civilFromDays(int32_t z, uint16_t& y, uint8_t& m, uint8_t& d) {
  z += 719468;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const uint32_t doe = (uint32_t)(z - era * 146097);
  const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp = (5 * doy + 2) / 153;
  d = (uint8_t)(doy - (153 * mp + 2) / 5 + 1);
  m = (uint8_t)(mp < 10 ? mp + 3 : mp - 9);
  y = (uint16_t)((int32_t)yoe + era * 400 + (m <= 2));
}

// Czas UTC epoki z NAV-PVT: data, godzina i nano (ulamek sekundy, bywa ujemny - np. 12:00:05
// i -20 us = 12:00:04.99998) zaokraglone do ms, z przeniesieniem az do daty.
void utcFromPvt(const uint8_t* p, rc::GpsFix& f) {
  const int32_t nano = i32(p + 16);
  const int32_t ms = (nano >= 0 ? nano + 500000 : nano - 500000) / 1000000;  // -1000..1000
  const int64_t t =
      ((int64_t)daysFromCivil(u16(p + 4), p[6], p[7]) * 86400 + p[8] * 3600 + p[9] * 60 + p[10]) * 1000 + ms;
  civilFromDays((int32_t)(t / 86400000), f.year, f.month, f.day);
  int32_t rest = (int32_t)(t % 86400000);
  f.hour = rest / 3600000;
  rest %= 3600000;
  f.minute = rest / 60000;
  rest %= 60000;
  f.second = rest / 1000;
  f.millis = rest % 1000;
}

// ---------- Konfiguracja M10 (UBX-CFG-VALSET) ----------
// Stan: szukamy predkosci UART -> konfigurujemy -> przechodzimy na GPS_UART_BAUD -> praca.
// W pracy pilnujemy, czy z modulu dalej plyna NAV-PVT i czy nie wrocilo NMEA (reset modulu do
// ustawien fabrycznych, np. po chwilowym braku zasilania) - wtedy wszystko od nowa.
// Deaf = modul nie odpowiada na komendy: bez konfiguracji nie ma NAV-PVT, czyli pozycji.
enum class Cfg : uint8_t { Detect, Verify, Run, Deaf };
constexpr uint32_t kBauds[] = {115200, 38400, 9600, 57600, 230400};
constexpr size_t kBaudCount = sizeof(kBauds) / sizeof(kBauds[0]);
constexpr uint32_t kDetectWindowMs = 1200;
constexpr uint32_t kVerifyTimeoutMs = 2500;
constexpr uint32_t kRetryCfgMs = 10000;  // modul nie odpowiadal na komendy: kolejna proba po tym czasie

Cfg cfg = Cfg::Detect;
size_t baudIdx = 0;
uint32_t windowStartMs = 0;
uint32_t countAtStart = 0;
bool cfgUnanswered = false;  // modul nie odpowiedzial na ostatnia konfiguracje
uint32_t cfgAttemptMs = 0;
uint32_t nmeaAtRun = 0;

// Odpowiedzi modulu na UBX-CFG-VALSET: UBX-ACK-ACK / UBX-ACK-NAK
uint32_t ackCount = 0;
uint32_t nakCount = 0;

// ---------- NMEA: tylko do wykrycia modulu na ustawieniach fabrycznych ----------
char nmea[96];
uint8_t nmeaLen = 0;
uint32_t nmeaCount = 0;  // zdania z poprawna suma kontrolna

bool checksumOk(const char* s) {
  const char* star = strchr(s, '*');
  if (!star || !star[1] || !star[2]) return false;
  uint8_t ck = 0;
  for (const char* p = s + 1; p < star; p++) ck ^= (uint8_t)*p;
  return ck == (uint8_t)strtoul(star + 1, nullptr, 16);
}

void nmeaByte(char c) {
  if (c == '$') {
    nmea[0] = c;
    nmeaLen = 1;
  } else if (nmeaLen == 0) {
    return;
  } else if (c == '\r' || c == '\n') {
    nmea[nmeaLen] = '\0';
    if (checksumOk(nmea)) nmeaCount++;
    nmeaLen = 0;
  } else if (nmeaLen < sizeof(nmea) - 1) {
    nmea[nmeaLen++] = c;
  } else {
    nmeaLen = 0;  // za dluga linia = smieci (np. zla predkosc UART)
  }
}

// ---------- Ramki UBX ----------
// Cala ramka (naglowek, dane, suma kontrolna) w buforze - w tej postaci idzie na karte (.ubx).
// Najwieksza: NAV-SAT, 8 + 12 B na satelite (MON-SPAN 276 B, MON-COMMS 8 + 40 B na port).
constexpr uint16_t kMaxPayload = 1024;
enum class Ubx : uint8_t { Sync1, Sync2, Cls, Id, Len1, Len2, Payload, CkA, CkB };
Ubx ubxSt = Ubx::Sync1;
uint8_t ubxCkA, ubxCkB;
uint16_t ubxLen, ubxPos;
uint8_t frame[6 + kMaxPayload + 2];
uint32_t ubxCount = 0;  // ramki z poprawna suma kontrolna
bool tooBigLogged = false;

void onFixStatus(bool fix);  // nizej
void countEpoch(uint32_t itow);

void onPvt(const uint8_t* p) {
  const uint32_t now = millis();
  pvtCount++;
  lastPvtMs = now;
  const uint8_t valid = p[11], fixType = p[20], flags = p[21];
  // gnssFixOK = pozycja w granicach DOP i dokladnosci; 2D, 3D albo GNSS + dead reckoning; invalidLlh = 0
  const bool fix = (flags & 0x01) && fixType >= 2 && fixType <= 4 && !(u16(p + 78) & 0x01);
  pvtSats = p[23];
  pvtAccM = u32(p + 40) / 1000.0f;
  onFixStatus(fix);
  if (valid & 0x02) countEpoch(u32(p));  // przed poznaniem czasu iTOW jeszcze skacze
  if (!fix) return;
  lastFixMs = now;
  if ((valid & 0x03) != 0x03) return;  // data albo czas UTC jeszcze nieznane

  rc::GpsFix& f = pending;
  utcFromPvt(p, f);
  f.lat = i32(p + 28) * 1e-7;
  f.lon = i32(p + 24) * 1e-7;
  f.altM = i32(p + 36) / 1000.0f;           // hMSL
  f.speedKmh = i32(p + 60) * 0.0036f;       // gSpeed [mm/s]
  float head = (float)(i32(p + 64) * 1e-5);  // headMot
  if (head < 0.0f) head += 360.0f;
  f.bearingDeg = head;
  f.satellites = pvtSats < 62 ? pvtSats : 62;  // RaceChrono: 6 bitow, 63 = brak
  f.fixQuality = (flags & 0x02) ? 2 : 1;       // RaceChrono: 2 = z korekcja (SBAS/EGNOS)
  const bool dopFresh = dopMs != 0 && now - dopMs <= kDopMaxAgeMs;
  f.hdop = dopFresh ? dopH : 9.9f;
  f.vdop = dopFresh ? dopV : 9.9f;
  hasPending = true;
}

void handleUbx(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len) {
  if (cls == 0x01 && id == 0x07 && len >= 92) {  // NAV-PVT
    onPvt(p);
  } else if (cls == 0x01 && id == 0x04 && len >= 18) {  // NAV-DOP
    dopV = u16(p + 10) / 100.0f;
    dopH = u16(p + 12) / 100.0f;
    dopMs = millis() | 1;
  } else if (cls == 0x05 && len >= 2 && p[0] == 0x06 && p[1] == 0x8A) {  // ACK/NAK na CFG-VALSET
    if (id == 0x01) {
      ackCount++;
      Serial.println("[GPS] modul potwierdzil konfiguracje (UBX-ACK-ACK)");
    } else if (id == 0x00) {
      nakCount++;
      syslog::logf("[GPS] modul ODRZUCIL konfiguracje (UBX-ACK-NAK)\n");
    }
  } else if (cls == 0x01 && id == 0x35 && len >= 8) {  // NAV-SAT
    const uint8_t n = p[5];
    if (len < 8 + 12 * n) return;
    const uint8_t m = n < kMaxSats ? n : kMaxSats;
    portENTER_CRITICAL(&diagMux);
    for (uint8_t i = 0; i < m; i++) {
      const uint8_t* e = p + 8 + 12 * i;
      const uint32_t f = u32(e + 8);
      uint8_t orbit = (f >> 8) & 7;
      if (orbit > 3) orbit = 3;
      SatInfo& si = live.sats[i];
      si.gnss = e[0];
      si.sv = e[1];
      si.cno = e[2];
      si.elev = (int8_t)e[3];
      si.azim = u16(e + 4);
      si.flags = (uint8_t)(((f >> 3) & 1) | ((f & 7) << 1) | (orbit << 4));
    }
    live.numSats = m;
    satMs = millis() | 1;
    portEXIT_CRITICAL(&diagMux);
  } else if (cls == 0x0A && id == 0x38 && len >= 4 + 24 && p[1] >= 1) {  // MON-RF, blok 0 = L1
    // przesuniecia z dokumentacji (4 + n*24 ...) juz zawieraja 4-bajtowy naglowek
    portENTER_CRITICAL(&diagMux);
    live.jam = p[5] & 0x03;
    live.ant = p[6];
    live.noise = u16(p + 16);
    live.agc = u16(p + 18);
    live.cw = p[20];
    rfMs = millis() | 1;
    portEXIT_CRITICAL(&diagMux);
  } else if (cls == 0x01 && id == 0x03 && len >= 16) {  // NAV-STATUS
    portENTER_CRITICAL(&diagMux);
    live.navFix = p[4];
    live.navTtff = u32(p + 8);
    live.navMsss = u32(p + 12);
    navMs = millis() | 1;
    portEXIT_CRITICAL(&diagMux);
    if (ttffLogPending && p[4] >= 2) {
      ttffLogPending = false;
      const uint32_t ttff = u32(p + 8), msss = u32(p + 12);
      syslog::logf("[GPS] wg modulu: pierwszy fix po %lu.%lu s, modul dziala od %lu s%s\n",
                   (unsigned long)(ttff / 1000), (unsigned long)(ttff % 1000 / 100), (unsigned long)(msss / 1000),
                   msss > millis() + 5000 ? " (pracowal juz przed startem ESP - start nie byl zimny)" : "");
    }
  } else if (cls == 0x0A && id == 0x36 && len >= 8) {  // MON-COMMS: txErrors = bufor nadawczy pelny
    if ((p[2] & 0x03) && !commsWarned) {
      commsWarned = true;
      syslog::logf("[GPS] UWAGA: bufor nadawczy modulu sie przepelnia (MON-COMMS) - za duzo danych na UART, "
                   "czesc komunikatow przepada\n");
    }
  }
}

// true = bajt nalezy do ramki UBX (nie przekazywac do NMEA). NMEA to sam ASCII, a ramka UBX
// zaczyna sie od 0xB5 0x62, wiec nie da sie ich pomylic.
bool ubxByte(uint8_t c) {
  switch (ubxSt) {
    case Ubx::Sync1:
      if (c != 0xB5) return false;
      frame[0] = c;
      ubxSt = Ubx::Sync2;
      return true;
    case Ubx::Sync2:
      if (c != 0x62) {
        ubxSt = Ubx::Sync1;
        return false;
      }
      frame[1] = c;
      ubxCkA = ubxCkB = 0;
      ubxSt = Ubx::Cls;
      return true;
    case Ubx::CkA:
      if (ubxLen <= kMaxPayload) frame[6 + ubxLen] = c;
      ubxSt = c == ubxCkA ? Ubx::CkB : Ubx::Sync1;
      return true;
    case Ubx::CkB:
      ubxSt = Ubx::Sync1;
      if (c != ubxCkB) return true;
      if (ubxLen > kMaxPayload) {
        if (!tooBigLogged) {
          tooBigLogged = true;
          syslog::logf("[GPS] ramka UBX 0x%02X 0x%02X ma %u B - wiecej niz bufor, pomijam\n", frame[2], frame[3],
                       ubxLen);
        }
        return true;
      }
      frame[7 + ubxLen] = c;
      ubxCount++;
      handleUbx(frame[2], frame[3], frame + 6, ubxLen);
      gpx::ubxFrame(frame, 8 + ubxLen);
      return true;
    default:
      break;
  }
  ubxCkA += c;
  ubxCkB += ubxCkA;
  switch (ubxSt) {
    case Ubx::Cls:
      frame[2] = c;
      ubxSt = Ubx::Id;
      break;
    case Ubx::Id:
      frame[3] = c;
      ubxSt = Ubx::Len1;
      break;
    case Ubx::Len1:
      frame[4] = c;
      ubxLen = c;
      ubxSt = Ubx::Len2;
      break;
    case Ubx::Len2:
      frame[5] = c;
      ubxLen |= (uint16_t)c << 8;
      ubxPos = 0;
      ubxSt = ubxLen > 4096 ? Ubx::Sync1 : (ubxLen ? Ubx::Payload : Ubx::CkA);  // > 4 kB = smieci
      break;
    case Ubx::Payload:
      if (ubxPos < kMaxPayload) frame[6 + ubxPos] = c;
      if (++ubxPos == ubxLen) ubxSt = Ubx::CkA;
      break;
    default:
      break;
  }
  return true;
}

// Jeden bajt z modulu: ramka UBX albo (poza ramka) zdanie NMEA.
void feed(uint8_t c) {
  if (!ubxByte(c)) nmeaByte((char)c);
}

// Klucze konfiguracji (u-blox M10 SPG 5.10 Interface Description)
constexpr uint32_t CFG_RATE_MEAS = 0x30210001;        // U2, okres pomiaru [ms]
constexpr uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021;  // U1, 4 = automotive
constexpr uint32_t CFG_UART1_BAUDRATE = 0x40520001;   // U4
constexpr uint32_t CFG_UART1OUTPROT_UBX = 0x10740001;
constexpr uint32_t CFG_UART1OUTPROT_NMEA = 0x10740002;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_DOP_UART1 = 0x20910039;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_STATUS_UART1 = 0x2091001b;
constexpr uint32_t CFG_MSGOUT_UBX_MON_RF_UART1 = 0x2091035a;
constexpr uint32_t CFG_MSGOUT_UBX_MON_SPAN_UART1 = 0x2091038c;
constexpr uint32_t CFG_MSGOUT_UBX_MON_COMMS_UART1 = 0x20910350;
constexpr uint32_t CFG_ITFM_ENABLE = 0x1041000d;  // L, detektor zaklocen (domyslnie wylaczony)
// L, wlaczenie systemow satelitarnych
constexpr uint32_t CFG_SIGNAL_GAL_ENA = 0x10310021;
constexpr uint32_t CFG_SIGNAL_BDS_ENA = 0x10310022;
constexpr uint32_t CFG_SIGNAL_QZSS_ENA = 0x10310024;
constexpr uint32_t CFG_SIGNAL_GLO_ENA = 0x10310025;

struct KeyVal {
  uint32_t key;
  uint32_t value;
};

// Rozmiar wartosci wynika z bitow 28..30 klucza.
size_t valueSize(uint32_t key) {
  switch ((key >> 28) & 0x7) {
    case 1:
    case 2:
      return 1;
    case 3:
      return 2;
    case 4:
      return 4;
    default:
      return 8;
  }
}

void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  const uint8_t head[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < 6; i++) {
    ckA += head[i];
    ckB += ckA;
  }
  for (uint16_t i = 0; i < len; i++) {
    ckA += payload[i];
    ckB += ckA;
  }
  const uint8_t ck[2] = {ckA, ckB};
  Serial2.write(head, sizeof(head));
  if (len) Serial2.write(payload, len);
  Serial2.write(ck, sizeof(ck));
}

// UBX-CFG-VALSET (0x06 0x8A), warstwa RAM: po odcieciu zasilania modul wraca do
// ustawien fabrycznych, a my konfigurujemy go od nowa przy kazdym starcie.
void sendValset(const KeyVal* kv, size_t count) {
  uint8_t payload[4 + 12 * 8];
  size_t n = 0;
  payload[n++] = 0x00;  // version
  payload[n++] = 0x01;  // layers: RAM
  payload[n++] = 0x00;
  payload[n++] = 0x00;
  for (size_t i = 0; i < count && n + 12 <= sizeof(payload); i++) {
    for (int b = 0; b < 4; b++) payload[n++] = (kv[i].key >> (8 * b)) & 0xFF;
    size_t vs = valueSize(kv[i].key);
    for (size_t b = 0; b < vs; b++) payload[n++] = (b < 4) ? (kv[i].value >> (8 * b)) & 0xFF : 0;
  }
  sendUbx(0x06, 0x8A, payload, (uint16_t)n);
}

// Czeka ms milisekund, caly czas przetwarzajac dane z modulu: odpowiedzi ACK/NAK sa znane
// przed decyzja o zmianie predkosci, a dane z modulu nie przepadaja.
void pump(uint32_t ms) {
  const uint32_t start = millis();
  do {
    while (Serial2.available()) feed((uint8_t)Serial2.read());
    delay(1);
  } while (millis() - start < ms);
}

// Czeka na odpowiedz ACK/NAK na ostatnia komende (najwyzej ms), przetwarzajac dane z modulu.
bool waitAck(uint32_t ms) {
  const uint32_t before = ackCount + nakCount;
  const uint32_t start = millis();
  do {
    while (Serial2.available()) feed((uint8_t)Serial2.read());
    if (ackCount + nakCount != before) return true;
    delay(1);
  } while (millis() - start < ms);
  return false;
}

void resetParsers() {
  ubxSt = Ubx::Sync1;
  nmeaLen = 0;
}

void startDetect(size_t idx) {
  baudIdx = idx % kBaudCount;
  Serial2.updateBaudRate(kBauds[baudIdx]);
  resetParsers();
  cfg = Cfg::Detect;
  windowStartMs = millis();
  countAtStart = ubxCount + nmeaCount;  // modul fabryczny nadaje NMEA, juz skonfigurowany (restart ESP) - UBX
  hasPending = false;
}

void enterRun() {
  haveItow = false;  // liczenie epok od nowa (inna czestotliwosc albo przerwa)
  curSec = -1;
  skipFirstSec = true;
  nmeaAtRun = nmeaCount;
  commsWarned = false;
  cfg = Cfg::Run;
}

void configureModule() {
  ackCount = 0;
  nakCount = 0;
  cfgAttemptMs = millis();
  // Systemy satelitarne wg trybu - zawsze wysylane w calosci, bo modul pamieta poprzedni
  // tryb do odciecia zasilania (sam reset ESP go nie czysci). GPS i SBAS zostaja wlaczone.
  // QZSS (japonskie, domyslnie wlaczone) i GLONASS zawsze wylaczone: QZSS w Europie nie widac.
  // Zmiana systemow restartuje odbiornik GNSS - dlatego wysylana pierwsza, a potem wg u-blox:
  // czekamy na ACK i jeszcze 0.5 s, zanim pojdzie nastepna komenda.
  const KeyVal signals[] = {
      {CFG_SIGNAL_GAL_ENA, mode.galileo},
      {CFG_SIGNAL_BDS_ENA, mode.beidou},
      {CFG_SIGNAL_QZSS_ENA, 0},
      {CFG_SIGNAL_GLO_ENA, 0},
  };
  sendValset(signals, sizeof(signals) / sizeof(signals[0]));
  Serial2.flush();
  waitAck(1500);
  pump(500);
  // W kazdej epoce: NAV-PVT (czas, pozycja, predkosc, kurs, dokladnosc) i NAV-DOP (HDOP i VDOP
  // dla RaceChrono). Same ramki UBX - NMEA wylaczone.
  const KeyVal settings[] = {
      {CFG_RATE_MEAS, 1000u / mode.hz},
      {CFG_NAVSPG_DYNMODEL, 4},
      {CFG_UART1OUTPROT_UBX, 1},
      {CFG_UART1OUTPROT_NMEA, 0},
      {CFG_MSGOUT_UBX_NAV_PVT_UART1, 1},
      {CFG_MSGOUT_UBX_NAV_DOP_UART1, 1},
  };
  sendValset(settings, sizeof(settings) / sizeof(settings[0]));
  Serial2.flush();
  waitAck(500);
  // Raz na sekunde, do zapisu na karte (analiza po jezdzie) i strony /gps. Wartosc dla NAV = co
  // ile epok, dla MON = co ile sekund (sprawdzone: MON-RF z wartoscia 25 szedl co 25 s). Osobno -
  // gdyby modul czegos nie przyjal, odrzuci tylko to, a nie pozycje powyzej.
  const KeyVal diagMsgs[] = {
      {CFG_MSGOUT_UBX_NAV_SAT_UART1, mode.hz},     // satelity: C/N0, wysokosc, uzycie
      {CFG_MSGOUT_UBX_NAV_STATUS_UART1, mode.hz},  // TTFF, czas pracy modulu, detektor spoofingu
      {CFG_MSGOUT_UBX_MON_RF_UART1, 1},            // szum, AGC, zaklocenia, stan anteny
      {CFG_MSGOUT_UBX_MON_SPAN_UART1, 1},          // widmo pasma L1 (256 punktow)
      {CFG_MSGOUT_UBX_MON_COMMS_UART1, 1},         // zajetosc bufora nadawczego UART modulu
      {CFG_ITFM_ENABLE, 1},                        // detektor zaklocen (jammingState w MON-RF)
  };
  sendValset(diagMsgs, sizeof(diagMsgs) / sizeof(diagMsgs[0]));
  Serial2.flush();
  waitAck(500);

  // Zadnej odpowiedzi = modul nas nie slyszy (przewod IO22 -> RX modulu): bez pozycji,
  // ponawiamy co kRetryCfgMs (w logu tylko raz).
  if (ackCount == 0 && nakCount == 0) {
    if (!cfgUnanswered)
      syslog::logf("[GPS] UWAGA: modul nie odpowiada na komendy - sprawdz przewod IO22 -> RX modulu\n");
    cfgUnanswered = true;
    cfg = Cfg::Deaf;
    return;
  }
  cfgUnanswered = false;

  // Zmiana predkosci na koncu: modul przelacza sie od razu, my za nim - chyba ze ja odrzucil.
  const uint32_t nakBefore = nakCount;
  const KeyVal baud[] = {{CFG_UART1_BAUDRATE, GPS_UART_BAUD}};
  sendValset(baud, 1);
  Serial2.flush();
  pump(100);
  const bool switched = nakCount == nakBefore;
  if (switched) {
    Serial2.updateBaudRate(GPS_UART_BAUD);
    // Resztki odebrane jeszcze na starej predkosci nie moga zaliczyc weryfikacji nowej.
    while (Serial2.available()) Serial2.read();
    resetParsers();
  }

  syslog::logf("[GPS] u-blox M10: %u Hz (%s), automotive, UBX, %lu bps%s\n", mode.hz, mode.name,
               (unsigned long)(switched ? GPS_UART_BAUD : kBauds[baudIdx]),
               switched ? "" : " (modul odrzucil zmiane predkosci)");
  cfg = Cfg::Verify;
  windowStartMs = millis();
  countAtStart = pvtCount;
}

void autoConfigStep() {
  const uint32_t now = millis();
  switch (cfg) {
    case Cfg::Detect:
      if (ubxCount + nmeaCount - countAtStart >= 2) {
        syslog::logf("[GPS] wykryto modul na %lu bps\n", (unsigned long)kBauds[baudIdx]);
        configureModule();
      } else if (now - windowStartMs > kDetectWindowMs) {
        startDetect(baudIdx + 1);
      }
      break;
    case Cfg::Verify:
      if (pvtCount - countAtStart >= 2) {
        enterRun();
        if (nakCount > 0)
          syslog::logf("[GPS] UWAGA: NAK x%lu - modul nie przyjal czesci ustawien\n", (unsigned long)nakCount);
        else
          syslog::logf("[GPS] konfiguracja OK (ACK x%lu)\n", (unsigned long)ackCount);
      } else if (now - windowStartMs > kVerifyTimeoutMs) {
        syslog::logf("[GPS] brak NAV-PVT po konfiguracji, szukam od nowa\n");
        startDetect(0);
      }
      break;
    case Cfg::Deaf:
      if (now - cfgAttemptMs >= kRetryCfgMs) configureModule();
      break;
    case Cfg::Run:
      // Liczy sie tylko poprawna ramka: przy niezgodnej predkosci UART (np. modul po resecie
      // wrocil na fabryczna) bajty dalej przychodza, tylko ze to smieci.
      if (now - lastPvtMs > kNoDataTimeoutMs) {
        syslog::logf("[GPS] brak NAV-PVT z modulu, szukam od nowa\n");
        startDetect(0);
      } else if (nmeaCount - nmeaAtRun >= 2) {
        // NMEA jest wylaczone - wrocilo, wiec modul przeszedl reset (np. chwilowy brak zasilania)
        syslog::logf("[GPS] modul znow nadaje NMEA (reset do ustawien fabrycznych?), konfiguruje od nowa\n");
        startDetect(0);
      }
      break;
  }
}

// Normalna praca z zadana czestotliwoscia (tylko wtedy liczenie zgubionych epok ma sens).
bool running() { return cfg == Cfg::Run; }

void pushRate(uint8_t n) {
  if (live.rateLen < sizeof(live.rate)) {
    live.rate[live.rateLen++] = n;
  } else {
    memmove(live.rate, live.rate + 1, sizeof(live.rate) - 1);
    live.rate[sizeof(live.rate) - 1] = n;
  }
}

void onFixStatus(bool fix) {
  if (fix == fixNow) return;
  fixNow = fix;
  const uint32_t now = millis();
  if (!fix) {
    fixLostMs = now;
    portENTER_CRITICAL(&diagMux);
    live.fixLost++;
    portEXIT_CRITICAL(&diagMux);
    return;
  }
  if (live.firstFixMs == 0) {
    portENTER_CRITICAL(&diagMux);
    live.firstFixMs = now ? now : 1;
    portEXIT_CRITICAL(&diagMux);
    syslog::logf("[GPS] pierwszy fix po %lu.%lu s od wlaczenia (norma M10: zimny start ~30 s przy czystym niebie)\n",
                 (unsigned long)(now / 1000), (unsigned long)(now % 1000 / 100));
    ttffLogPending = true;  // TTFF i czas pracy wg modulu (NAV-STATUS): zimny start czy modul juz dzialal
  } else if (now - fixLostMs >= 2000) {
    syslog::logf("[GPS] fix wrocil po %lu s przerwy\n", (unsigned long)((now - fixLostMs) / 1000));
  }
}

// Kazdy NAV-PVT = jedna epoka pomiarowa modulu; iTOW = jej czas GPS [ms od poczatku tygodnia].
void countEpoch(uint32_t itow) {
  if (!running()) {
    haveItow = false;
    curSec = -1;
    skipFirstSec = true;
    return;
  }
  const int32_t period = 1000 / mode.hz;  // ms: 25 Hz = 40, 20 Hz = 50, 10 Hz = 100

  portENTER_CRITICAL(&diagMux);
  if (haveItow) {
    int32_t dt = (int32_t)itow - (int32_t)lastItow;
    if (dt < 0) dt += kWeekMs;  // nowy tydzien GPS
    if (dt > 3000) {
      live.gaps++;
    } else if (dt > 0) {
      const int32_t n = (dt + period / 2) / period;  // ile okresow minelo
      if (n > 1) live.missed += n - 1;
    }
  }
  haveItow = true;
  lastItow = itow;
  const int32_t sec = (int32_t)(itow / 1000);
  if (sec != curSec) {
    if (curSec >= 0) {
      if (!skipFirstSec) {
        pushRate(epochsInSec);
        int32_t ds = sec - curSec;
        if (ds < 0) ds += kWeekMs / 1000;
        for (int32_t k = 1; k < ds && k <= 5; k++) pushRate(0);  // sekundy bez zadnej epoki
      }
      skipFirstSec = false;
    }
    curSec = sec;
    epochsInSec = 0;
  }
  if (epochsInSec < 255) epochsInSec++;
  live.epochs++;
  portEXIT_CRITICAL(&diagMux);
}

// Kopia stanu i jakosci dla ekranu (patrz stateSnap).
void publish() {
  const uint32_t now = millis();
  State s = State::Fix;
  if (cfg != Cfg::Run || lastPvtMs == 0 || now - lastPvtMs > kNoDataTimeoutMs)
    s = State::NoData;
  else if (!fixNow || now - lastFixMs > kMaxFixAgeMs)
    s = State::NoFix;
  stateSnap = s;

  // Satelity uzyte w rozwiazaniu - takze przed fixem (wtedy zwykle 0-3).
  qUsed = (s == State::NoData) ? 0 : pvtSats;
  const bool fix = s == State::Fix;
  qHdop = (fix && dopMs != 0 && now - dopMs <= kDopMaxAgeMs) ? dopH : 0.0f;
  qAcc = fix ? pvtAccM : 0.0f;
}

}  // namespace

void begin(uint8_t gpsMode) {
  mode = modeFor(gpsMode);
  Serial2.setRxBufferSize(2048);
  Serial2.begin(kBauds[0], SERIAL_8N1, GPS_UART_RX, GPS_UART_TX);
  startDetect(0);
  syslog::logf("[GPS] UART RX=%d TX=%d, szukam modulu u-blox M10\n", GPS_UART_RX, GPS_UART_TX);
}

bool update(rc::GpsFix& out) {
  // Najwyzej jedna pozycja na wywolanie - reszta czeka w buforze UART na nastepne (za 2 ms).
  while (!hasPending && Serial2.available()) feed((uint8_t)Serial2.read());
  autoConfigStep();
  publish();
  if (!hasPending) return false;
  hasPending = false;
  if (cfg != Cfg::Run) return false;
  out = pending;
  return true;
}

State state() { return stateSnap; }

Quality quality() { return Quality{qUsed, qHdop, qAcc}; }

void stop() { Serial2.end(); }  // zwalnia sterownik UART (bufory) - przed wgrywaniem programu

void diag(Diag& out) {
  portENTER_CRITICAL(&diagMux);
  out = live;
  const uint32_t sm = satMs, rm = rfMs, nm = navMs;
  portEXIT_CRITICAL(&diagMux);
  const uint32_t now = millis();
  out.satAge = sm ? now - sm : UINT32_MAX;
  out.rfAge = rm ? now - rm : UINT32_MAX;
  out.navAge = nm ? now - nm : UINT32_MAX;
  out.hz = mode.hz;
  out.modeName = mode.name;
  out.running = running();
}

}  // namespace gpsrx

#else  // bez GPS_UART_RX: brak modulu

namespace gpsrx {
void begin(uint8_t) {}
bool update(rc::GpsFix&) { return false; }
State state() { return State::Off; }
Quality quality() { return {}; }
void stop() {}
void diag(Diag& out) {
  out = Diag{};
  out.satAge = out.rfAge = out.navAge = UINT32_MAX;
  out.modeName = "brak modulu GPS";
}
}  // namespace gpsrx

#endif
