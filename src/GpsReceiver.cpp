#include "GpsReceiver.h"
#include "SysLog.h"

#ifdef GPS_UART_RX

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "GpxLogger.h"

#ifndef GPS_UART_TX
#define GPS_UART_TX -1
#endif
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD 9600  // docelowa predkosc UART
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

constexpr uint32_t kNoDataTimeoutMs = 3000;
constexpr uint32_t kMaxFixAgeMs = 1500;

TinyGPSPlus gps;
char line[128];
size_t lineLen = 0;
uint32_t lastValidMs = 0;         // ostatnie zdanie NMEA z poprawna suma kontrolna (0 = jeszcze zadne)
uint32_t lastEpoch = 0xFFFFFFFF;  // czas ostatnio wyslanej pozycji (hhmmsscc)

// ---------- Wlasne parsowanie: jakosc fixa z GGA, satelity z sygnalem z GSV ----------
constexpr uint32_t kTrackedMaxAgeMs = 3000;
uint8_t ggaQuality = 0;  // pole 6 GGA: 0 brak, 1 GPS, 2 DGPS/SBAS, 4/5 RTK, 6 DR
uint8_t tracked = 0;     // satelity z sygnalem (SNR > 0) z ostatniego kompletu GSV
uint32_t trackedMs = 0;
uint8_t trackedAcc = 0;
bool gsvPending = false;
uint32_t rmcCount = 0;  // licznik pomiarow (rzeczywista czestotliwosc: kontrola modulu i debug)
// Satelity uzyte w fixie z GNS (zakres 0-99, raz na sekunde) - w GGA to pole ma limit 12.
constexpr uint32_t kGnsMaxAgeMs = 2500;
uint8_t gnsSats = 0;
uint32_t gnsMs = 0;
// Szacowany blad pozycji poziomej z GST (raz na sekunde): odbiornik liczy go z jakosci pomiarow,
// wiec - inaczej niz HDOP (sama geometria satelitow) - widac w nim tez slaby sygnal i odbicia.
float gstAccM = 0.0f;
uint32_t gstMs = 0;

// Diagnostyka dla strony /gps. Pisze zadanie GPS, czyta strona (diag()) - pod diagMux.
portMUX_TYPE diagMux = portMUX_INITIALIZER_UNLOCKED;
Diag live{};
uint32_t satMs = 0, rfMs = 0, navMs = 0;  // kiedy przyszly ostatnie NAV-SAT / MON-RF / NAV-STATUS
// Epoki liczone z czasu w RMC (nie z chwili odbioru), wiec opoznienia UART i zadan ich nie psuja.
int32_t lastRmcCs = -1;  // czas ostatniego RMC [setne sekundy od polnocy UTC], -1 = brak
int32_t curSec = -1;     // biezaca sekunda UTC i liczba epok w niej
uint8_t epochsInSec = 0;
bool skipFirstSec = true;  // pierwsza sekunda po (re)starcie liczenia jest niepelna
bool fixNow = false;
uint32_t fixLostMs = 0;
constexpr uint32_t kDiagHoldMs = 30000;  // komunikaty diagnostyczne tyle po ostatnim odpytaniu strony
volatile uint32_t diagReqMs = 0;
bool diagOn = false;       // modul wysyla teraz komunikaty diagnostyczne
bool diagSync = true;      // wyslac stan diagnostyki do modulu przy najblizszej okazji
bool navPollPending = false;  // po pierwszym fixie: odpowiedz NAV-STATUS idzie do logu

void onRmc(const char* s);  // nizej: liczenie epok i stan fixa

// Wszystko powyzej (razem z TinyGPS++) zmienia i czyta tylko zadanie GPS. Ekran (inne zadanie)
// dostaje stan i jakosc przez te kopie, odswiezane w kazdym update().
volatile State stateSnap = State::NoData;
volatile uint8_t qUsed = 0, qTracked = 0, qFix = 0;
volatile float qHdop = 0.0f, qAcc = 0.0f;

bool checksumOk(const char* s) {
  const char* star = strchr(s, '*');
  if (!star || !star[1] || !star[2]) return false;
  uint8_t ck = 0;
  for (const char* p = s + 1; p < star; p++) ck ^= (uint8_t)*p;
  return ck == (uint8_t)strtoul(star + 1, nullptr, 16);
}

// Poczatek pola n (0 = "$GNGGA"); pole konczy sie na ',' albo '*'. nullptr = brak pola.
const char* field(const char* s, int n) {
  for (; n > 0; n--) {
    s = strpbrk(s, ",*");
    if (!s || *s == '*') return nullptr;
    s++;
  }
  return s;
}

// Zwraca true, gdy linia ma trafic do backupu na SD (GSV sluzy tylko wskaznikowi na ekranie).
bool processLine(const char* s) {
  if (s[0] != '$' || strlen(s) < 6) return false;
  const char* type = s + 3;  // po "$GN"
  const bool isGsv = strncmp(type, "GSV", 3) == 0;

  // Komplet GSV (wszystkie systemy, wiele linii) konczy sie na pierwszej linii innego typu.
  if (!isGsv && gsvPending) {
    tracked = trackedAcc;
    trackedMs = millis();
    trackedAcc = 0;
    gsvPending = false;
  }
  if (!checksumOk(s)) return !isGsv;

  if (strncmp(type, "RMC", 3) == 0) {
    rmcCount++;  // jedno RMC na pomiar
    onRmc(s);
  } else if (strncmp(type, "GGA", 3) == 0) {
    const char* q = field(s, 6);
    if (q) ggaQuality = (uint8_t)atoi(q);
  } else if (strncmp(type, "GNS", 3) == 0) {
    // $xxGNS,time,lat,NS,lon,EW,posMode,numSV,... - tylko linia z pozycja (laczne rozwiazanie);
    // przy kilku systemach moga dojsc linie z satelitami pojedynczego systemu i pusta pozycja.
    const char* lat = field(s, 2);
    const char* n = field(s, 7);
    if (lat && *lat != ',' && n && *n >= '0' && *n <= '9') {
      gnsSats = (uint8_t)min(atoi(n), 255);
      gnsMs = millis();
    }
  } else if (strncmp(type, "GST", 3) == 0) {
    // $xxGST,time,rangeRms,stdMajor,stdMinor,orient,stdLat,stdLong,stdAlt - odchylenia w metrach
    const char* la = field(s, 6);
    const char* lo = field(s, 7);
    if (la && lo && *la >= '0' && *la <= '9' && *lo >= '0' && *lo <= '9') {
      const float a = (float)atof(la), b = (float)atof(lo);
      gstAccM = sqrtf(a * a + b * b);
      gstMs = millis();
    }
  } else if (isGsv) {
    // $xxGSV,liczbaLinii,nrLinii,satWidocznych,{prn,elew,azym,snr} x <=4[,signalId]*cs
    const char* nr = field(s, 2);
    const char* inView = field(s, 3);
    if (!nr || !inView) return false;
    int inThisLine = atoi(inView) - (atoi(nr) - 1) * 4;
    for (int i = 0; i < 4 && i < inThisLine; i++) {
      const char* snr = field(s, 7 + 4 * i);
      if (snr && atoi(snr) > 0 && trackedAcc < 255) trackedAcc++;
    }
    gsvPending = true;
  }
  return !isGsv;
}

// Satelity uzyte w fixie: ze swiezego GNS (bez limitu 12), inaczej z GGA.
uint32_t usedSats() {
  if (gnsMs != 0 && millis() - gnsMs <= kGnsMaxAgeMs) return gnsSats;
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

void feed(char c);  // nizej - wolane takze w trakcie konfiguracji modulu

#ifdef GPS_UBLOX_M10
// ---------- Autokonfiguracja u-blox M10 (UBX-CFG-VALSET) ----------
// Stan: szukamy predkosci UART -> konfigurujemy -> przechodzimy na GPS_UART_BAUD -> praca.
// W pracy pilnujemy, czy z modulu dalej plyna poprawne zdania z zadana czestotliwoscia.
// Gdy nie (odlaczenie, reset modulu do ustawien fabrycznych) - wszystko od nowa.
enum class Cfg : uint8_t { Detect, Verify, Run };
constexpr uint32_t kBauds[] = {115200, 38400, 9600, 57600, 230400};
constexpr size_t kBaudCount = sizeof(kBauds) / sizeof(kBauds[0]);
constexpr uint32_t kDetectWindowMs = 1200;
constexpr uint32_t kVerifyTimeoutMs = 2500;
constexpr uint32_t kRetryCfgMs = 30000;  // modul nie odpowiadal na komendy: kolejna proba po tym czasie
constexpr uint8_t kLowRateSecs = 3;      // tyle sekund ponizej polowy zadanej czestotliwosci = reset modulu

Cfg cfg = Cfg::Detect;
size_t baudIdx = 0;
uint32_t windowStartMs = 0;
uint32_t passedAtStart = 0;
bool cfgUnanswered = false;  // modul nie odpowiedzial na ostatnia konfiguracje
uint32_t cfgAttemptMs = 0;

// Kontrola czestotliwosci w pracy
uint32_t rateWindowMs = 0;
uint32_t rateRmcAtStart = 0;
bool rateReached = false;  // modul osiagnal juz co najmniej polowe zadanej czestotliwosci
uint8_t lowRateSecs = 0;

// Odpowiedzi modulu na UBX-CFG-VALSET: UBX-ACK-ACK / UBX-ACK-NAK
uint32_t ackCount = 0;
uint32_t nakCount = 0;

// ---------- Ramki UBX w strumieniu z modulu (wymieszane z NMEA) ----------
// Bajty ramki UBX nie trafiaja do TinyGPS++ ani do zapisu NMEA na SD.
enum class Ubx : uint8_t { Sync1, Sync2, Cls, Id, Len1, Len2, Payload, CkA, CkB };
Ubx ubxSt = Ubx::Sync1;
uint8_t ubxCls, ubxId, ubxCkA, ubxCkB;
uint16_t ubxLen, ubxPos;
uint8_t ubxBuf[8 + 12 * kMaxSats];  // najwiekszy obslugiwany: UBX-NAV-SAT

uint16_t u16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
uint32_t u32(const uint8_t* p) { return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

void handleUbx(uint8_t cls, uint8_t id, const uint8_t* p, uint16_t len) {
  if (cls == 0x05 && len >= 2 && p[0] == 0x06 && p[1] == 0x8A) {  // ACK/NAK na CFG-VALSET
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
    if (navPollPending && p[4] >= 2) {
      navPollPending = false;
      const uint32_t ttff = u32(p + 8), msss = u32(p + 12);
      syslog::logf("[GPS] wg modulu: pierwszy fix po %lu.%lu s, modul dziala od %lu s%s\n",
                   (unsigned long)(ttff / 1000), (unsigned long)(ttff % 1000 / 100), (unsigned long)(msss / 1000),
                   msss > millis() + 5000 ? " (pracowal juz przed startem ESP - start nie byl zimny)" : "");
    }
  }
}

// true = bajt nalezy do ramki UBX (nie przekazywac do NMEA). NMEA to sam ASCII, a ramka UBX
// zaczyna sie od 0xB5 0x62, wiec nie da sie ich pomylic.
bool ubxByte(uint8_t c) {
  switch (ubxSt) {
    case Ubx::Sync1:
      if (c != 0xB5) return false;
      ubxSt = Ubx::Sync2;
      return true;
    case Ubx::Sync2:
      if (c != 0x62) {
        ubxSt = Ubx::Sync1;
        return false;
      }
      ubxCkA = ubxCkB = 0;
      ubxSt = Ubx::Cls;
      return true;
    case Ubx::CkA:
      ubxSt = c == ubxCkA ? Ubx::CkB : Ubx::Sync1;
      return true;
    case Ubx::CkB:
      ubxSt = Ubx::Sync1;
      if (c == ubxCkB && ubxLen <= sizeof(ubxBuf)) handleUbx(ubxCls, ubxId, ubxBuf, ubxLen);
      return true;
    default:
      break;
  }
  ubxCkA += c;
  ubxCkB += ubxCkA;
  switch (ubxSt) {
    case Ubx::Cls:
      ubxCls = c;
      ubxSt = Ubx::Id;
      break;
    case Ubx::Id:
      ubxId = c;
      ubxSt = Ubx::Len1;
      break;
    case Ubx::Len1:
      ubxLen = c;
      ubxSt = Ubx::Len2;
      break;
    case Ubx::Len2:
      ubxLen |= (uint16_t)c << 8;
      ubxPos = 0;
      ubxSt = ubxLen > 4096 ? Ubx::Sync1 : (ubxLen ? Ubx::Payload : Ubx::CkA);  // > 4 kB = smieci
      break;
    case Ubx::Payload:
      if (ubxPos < sizeof(ubxBuf)) ubxBuf[ubxPos] = c;
      if (++ubxPos == ubxLen) ubxSt = Ubx::CkA;
      break;
    default:
      break;
  }
  return true;
}

// Klucze konfiguracji M10 (u-blox M10 Interface Description)
constexpr uint32_t CFG_RATE_MEAS = 0x30210001;          // U2, okres pomiaru [ms]
constexpr uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021;    // U1, 4 = automotive
constexpr uint32_t CFG_UART1_BAUDRATE = 0x40520001;     // U4
constexpr uint32_t CFG_MSGOUT_NMEA_GGA_UART1 = 0x209100bb;
constexpr uint32_t CFG_MSGOUT_NMEA_RMC_UART1 = 0x209100ac;
constexpr uint32_t CFG_MSGOUT_NMEA_GLL_UART1 = 0x209100ca;
constexpr uint32_t CFG_MSGOUT_NMEA_GNS_UART1 = 0x209100b6;
constexpr uint32_t CFG_MSGOUT_NMEA_GST_UART1 = 0x209100d4;
constexpr uint32_t CFG_MSGOUT_NMEA_GSA_UART1 = 0x209100c0;
constexpr uint32_t CFG_MSGOUT_NMEA_GSV_UART1 = 0x209100c5;
constexpr uint32_t CFG_MSGOUT_NMEA_VTG_UART1 = 0x209100b1;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016;
constexpr uint32_t CFG_MSGOUT_UBX_MON_RF_UART1 = 0x2091035a;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_STATUS_UART1 = 0x2091001b;
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

// Komunikaty diagnostyczne raz na sekunde: satelity z C/N0, szum i AGC wejscia radiowego,
// stan nawigacji z TTFF. Do tego detektor zaklocen (jammingState). Wartosc dla NAV = co ile
// epok pomiarowych, dla MON = co ile sekund (sprawdzone: MON-RF z wartoscia 25 szedl co 25 s).
// Tylko przy otwartej stronie /gps - w jezdzie modul nie ma dodatkowej pracy.
void setDiagOutput(bool on) {
  const uint32_t r = on ? mode.hz : 0;
  const KeyVal kv[] = {{CFG_MSGOUT_UBX_NAV_SAT_UART1, r},
                       {CFG_MSGOUT_UBX_MON_RF_UART1, on ? 1u : 0u},
                       {CFG_MSGOUT_UBX_NAV_STATUS_UART1, r},
                       {CFG_ITFM_ENABLE, on ? 1u : 0u}};
  sendValset(kv, sizeof(kv) / sizeof(kv[0]));
  Serial.printf("[GPS] komunikaty diagnostyczne modulu %s\n", on ? "wlaczone" : "wylaczone");
}

void pollNavStatus() { sendUbx(0x01, 0x03, nullptr, 0); }

// Czeka ms milisekund, caly czas przetwarzajac dane z modulu: odpowiedzi ACK/NAK sa znane
// przed decyzja o zmianie predkosci, a zdania NMEA nie przepadaja.
void pump(uint32_t ms) {
  const uint32_t start = millis();
  do {
    while (Serial2.available()) feed((char)Serial2.read());
    delay(1);
  } while (millis() - start < ms);
}

// Czeka na odpowiedz ACK/NAK na ostatnia komende (najwyzej ms), przetwarzajac dane z modulu.
bool waitAck(uint32_t ms) {
  const uint32_t before = ackCount + nakCount;
  const uint32_t start = millis();
  do {
    while (Serial2.available()) feed((char)Serial2.read());
    if (ackCount + nakCount != before) return true;
    delay(1);
  } while (millis() - start < ms);
  return false;
}

void startDetect(size_t idx) {
  diagSync = true;  // modul mogl wrocic do ustawien fabrycznych
  baudIdx = idx % kBaudCount;
  Serial2.updateBaudRate(kBauds[baudIdx]);
  cfg = Cfg::Detect;
  windowStartMs = millis();
  passedAtStart = gps.passedChecksum();
}

void enterRun() {
  diagSync = true;
  lastRmcCs = -1;  // liczenie epok od nowa (inna czestotliwosc albo przerwa)
  curSec = -1;
  skipFirstSec = true;
  cfg = Cfg::Run;
  rateWindowMs = millis();
  rateRmcAtStart = rmcCount;
  rateReached = false;
  lowRateSecs = 0;
}

void configureModule() {
  const KeyVal settings[] = {
      {CFG_RATE_MEAS, 1000u / mode.hz},
      {CFG_NAVSPG_DYNMODEL, 4},
      {CFG_MSGOUT_NMEA_GGA_UART1, 1},
      {CFG_MSGOUT_NMEA_RMC_UART1, 1},
      {CFG_MSGOUT_NMEA_GLL_UART1, 0},
      {CFG_MSGOUT_NMEA_GSA_UART1, 0},
      // GSV raz na sekunde (co mode.hz pomiarow): ile satelitow slychac (diagnostyka)
      {CFG_MSGOUT_NMEA_GSV_UART1, mode.hz},
      {CFG_MSGOUT_NMEA_VTG_UART1, 0},
  };
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
  sendValset(settings, sizeof(settings) / sizeof(settings[0]));
  Serial2.flush();
  waitAck(500);
  // GNS i GST raz na sekunde: prawdziwa liczba uzytych satelitow i szacowana dokladnosc.
  // Osobno - gdyby modul ich nie przyjal, odrzuci tylko je, a nie cala konfiguracje powyzej.
  const KeyVal extra[] = {{CFG_MSGOUT_NMEA_GNS_UART1, mode.hz}, {CFG_MSGOUT_NMEA_GST_UART1, mode.hz}};
  sendValset(extra, sizeof(extra) / sizeof(extra[0]));
  Serial2.flush();
  waitAck(500);

  // Zadnej odpowiedzi = modul nas nie slyszy (przewod IO22 -> RX modulu). Zmiana predkosci
  // odcielaby wtedy takze odbior, wiec zostajemy na wykrytej predkosci i ustawieniach modulu
  // (zwykle 1 Hz) - lepsze to niz nic - i ponawiamy konfiguracje co kRetryCfgMs.
  if (ackCount == 0 && nakCount == 0) {
    if (!cfgUnanswered)
      syslog::logf("[GPS] UWAGA: modul nie odpowiada na komendy - sprawdz przewod IO22 -> RX modulu. "
                   "Zostaje na %lu bps i ustawieniach modulu, ponawiam co %lu s\n",
                   (unsigned long)kBauds[baudIdx], (unsigned long)(kRetryCfgMs / 1000));
    cfgUnanswered = true;
    enterRun();
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
    lineLen = 0;
  }

  syslog::logf("[GPS] u-blox M10: %u Hz (%s), automotive, RMC+GGA, %lu bps%s\n", mode.hz, mode.name,
               (unsigned long)(switched ? GPS_UART_BAUD : kBauds[baudIdx]),
               switched ? "" : " (modul odrzucil zmiane predkosci)");
  cfg = Cfg::Verify;
  windowStartMs = millis();
  passedAtStart = gps.passedChecksum();
}

void autoConfigStep() {
  uint32_t now = millis();
  switch (cfg) {
    case Cfg::Detect:
      if (gps.passedChecksum() - passedAtStart >= 2) {
        syslog::logf("[GPS] wykryto modul na %lu bps\n", (unsigned long)kBauds[baudIdx]);
        configureModule();
      } else if (now - windowStartMs > kDetectWindowMs) {
        startDetect(baudIdx + 1);
      }
      break;
    case Cfg::Verify:
      if (gps.passedChecksum() - passedAtStart >= 2) {
        enterRun();
        if (nakCount > 0)
          syslog::logf("[GPS] UWAGA: NAK x%lu - modul nie przyjal czesci ustawien\n", (unsigned long)nakCount);
        else
          syslog::logf("[GPS] konfiguracja OK (ACK x%lu)\n", (unsigned long)ackCount);
      } else if (now - windowStartMs > kVerifyTimeoutMs) {
        syslog::logf("[GPS] brak danych po konfiguracji, szukam od nowa\n");
        startDetect(0);
      }
      break;
    case Cfg::Run:
      // Liczy sie tylko poprawne zdanie: przy niezgodnej predkosci UART (np. modul po resecie
      // wrocil na fabryczna) bajty dalej przychodza, tylko ze to smieci.
      if (now - lastValidMs > kNoDataTimeoutMs) {
        syslog::logf("[GPS] brak poprawnych danych z modulu, szukam od nowa\n");
        startDetect(0);
        break;
      }
      if (now - rateWindowMs >= 1000) {
        // Reset modulu (np. chwilowy brak zasilania) przywraca ustawienia fabryczne, zwykle 1 Hz,
        // a przy tej samej predkosci UART dane dalej wygladaja poprawnie. Liczy sie tylko spadek
        // po osiagnieciu zadanej czestotliwosci - modul, ktory jej nie osiaga, nie wpadnie w petle.
        const uint32_t hz = rmcCount - rateRmcAtStart;
        rateRmcAtStart = rmcCount;
        rateWindowMs = now;
        const bool rateOk = hz * 2 >= mode.hz;
        if (rateOk) rateReached = true;
        lowRateSecs = (rateReached && !rateOk) ? lowRateSecs + 1 : 0;
        if (lowRateSecs >= kLowRateSecs) {
          syslog::logf("[GPS] czestotliwosc spadla do %lu Hz (reset modulu?), konfiguruje od nowa\n",
                       (unsigned long)hz);
          startDetect(0);
          break;
        }
      }
      // Modul nas nie slyszal: ponawiamy, ale tylko gdy nie nadaje z zadana czestotliwoscia
      // (konfiguracja wstrzymuje wysylke pozycji na ~350 ms - bez potrzeby szkoda punktow).
      if (cfgUnanswered && !rateReached && now - cfgAttemptMs >= kRetryCfgMs) configureModule();
      break;
  }
}
#endif  // GPS_UBLOX_M10

// Normalna praca z zadana czestotliwoscia (tylko wtedy liczenie zgubionych epok ma sens).
bool running() {
#ifdef GPS_UBLOX_M10
  return cfg == Cfg::Run && !cfgUnanswered;
#else
  return true;
#endif
}

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
#ifdef GPS_UBLOX_M10
    navPollPending = true;  // TTFF i czas pracy wg modulu: zimny start czy modul juz dzialal
    pollNavStatus();
#endif
  } else if (now - fixLostMs >= 2000) {
    syslog::logf("[GPS] fix wrocil po %lu s przerwy\n", (unsigned long)((now - fixLostMs) / 1000));
  }
}

// Kazde RMC = jedna epoka pomiarowa modulu.
void onRmc(const char* s) {
  const char* t = field(s, 1);
  const char* status = field(s, 2);
  onFixStatus(status && *status == 'A');
  if (!running() || !t) {
    lastRmcCs = -1;
    curSec = -1;
    skipFirstSec = true;
    return;
  }
  for (int i = 0; i < 6; i++)
    if (t[i] < '0' || t[i] > '9') return;  // modul nie zna jeszcze czasu
  int32_t cs = (((t[0] - '0') * 10 + (t[1] - '0')) * 3600 + ((t[2] - '0') * 10 + (t[3] - '0')) * 60 +
                (t[4] - '0') * 10 + (t[5] - '0')) * 100;
  if (t[6] == '.' && t[7] >= '0' && t[7] <= '9') {
    cs += (t[7] - '0') * 10;
    if (t[8] >= '0' && t[8] <= '9') cs += t[8] - '0';
  }
  const int32_t period = 100 / mode.hz;  // setne sekundy: 25 Hz = 4, 20 Hz = 5, 10 Hz = 10

  portENTER_CRITICAL(&diagMux);
  if (lastRmcCs >= 0) {
    int32_t dt = cs - lastRmcCs;
    if (dt < 0) dt += 8640000;  // polnoc UTC
    if (dt > 300) {
      live.gaps++;
    } else if (dt > 0) {
      const int32_t n = (dt + period / 2) / period;  // ile okresow minelo
      if (n > 1) {
        live.missed += n - 1;
        if (diagOn) live.missedDiag += n - 1;
      }
    }
  }
  lastRmcCs = cs;
  const int32_t sec = cs / 100;
  if (sec != curSec) {
    if (curSec >= 0) {
      if (!skipFirstSec) {
        pushRate(epochsInSec);
        int32_t ds = sec - curSec;
        if (ds < 0) ds += 86400;
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

#ifdef GPS_DEBUG
// Co sekunde: ile bajtow i zdan NMEA przyszlo, stan fixa i ostatnia linia.
uint32_t dbgChars = 0;
uint32_t dbgLastMs = 0;
uint32_t dbgLastPassed = 0;
uint32_t dbgLastFailed = 0;
char dbgLine[sizeof(line)] = "";

void debugStep() {
  uint32_t now = millis();
  if (now - dbgLastMs < 1000) return;
  dbgLastMs = now;
  uint32_t passed = gps.passedChecksum();
  uint32_t failed = gps.failedChecksum();
  Quality q = quality();
  static uint32_t lastRmc = 0;
  Serial.printf("[GPS] %lu Hz | %lu bps | %lu B/s | NMEA ok %lu, zle %lu | sat %u/%u hdop %.1f blad %.1fm q%u | %s",
                (unsigned long)(rmcCount - lastRmc), (unsigned long)Serial2.baudRate(), (unsigned long)dbgChars,
                (unsigned long)(passed - dbgLastPassed), (unsigned long)(failed - dbgLastFailed),
                q.used, q.tracked, q.hdop, q.accM, q.fix,
                dbgChars == 0 ? "BRAK DANYCH" : (gps.location.isValid() ? "FIX" : "brak fixa"));
  if (gps.location.isValid())
    Serial.printf(" %.6f,%.6f %.1f km/h", gps.location.lat(), gps.location.lng(), gps.speed.kmph());
  Serial.println();
  if (dbgLine[0]) Serial.printf("[GPS]   ostatnia linia: %s\n", dbgLine);
  lastRmc = rmcCount;
  dbgChars = 0;
  dbgLine[0] = '\0';
  dbgLastPassed = passed;
  dbgLastFailed = failed;
}
#endif  // GPS_DEBUG

// Jeden bajt z modulu: TinyGPS++, odpowiedzi UBX i wlasne parsowanie linii NMEA (+ backup na SD).
void feed(char c) {
#ifdef GPS_DEBUG
  dbgChars++;
#endif
#ifdef GPS_UBLOX_M10
  if (ubxByte((uint8_t)c)) return;
#endif
  if (gps.encode(c)) lastValidMs = millis();  // true = zdanie z poprawna suma kontrolna

  // Surowe linie NMEA do backupu
  if (c == '\n' || c == '\r') {
    if (lineLen > 0) {
      line[lineLen] = '\0';
      if (processLine(line)) gpx::nmeaLine(line);
#ifdef GPS_DEBUG
      memcpy(dbgLine, line, lineLen + 1);
#endif
      lineLen = 0;
    }
  } else if (lineLen < sizeof(line) - 1) {
    line[lineLen++] = c;
  }
}

// Kopia stanu i jakosci dla ekranu (patrz stateSnap).
void publish() {
  const uint32_t now = millis();
  const bool fresh = gps.location.isValid() && gps.location.age() <= kMaxFixAgeMs;
  State s = State::Fix;
  if (lastValidMs == 0 || now - lastValidMs > kNoDataTimeoutMs) s = State::NoData;
#ifdef GPS_UBLOX_M10
  else if (cfg != Cfg::Run) s = State::NoData;
#endif
  else if (!fresh) s = State::NoFix;
  stateSnap = s;

  // Satelity uzyte w rozwiazaniu - takze przed fixem (wtedy zwykle 0-3), bez "slyszanych".
  qUsed = (s == State::NoData) ? 0 : (uint8_t)min<uint32_t>(usedSats(), 255);
  qHdop = (fresh && gps.hdop.isValid()) ? (float)gps.hdop.hdop() : 0.0f;
  qAcc = (fresh && gstMs != 0 && now - gstMs <= kGnsMaxAgeMs) ? gstAccM : 0.0f;
  qTracked = (now - trackedMs <= kTrackedMaxAgeMs) ? tracked : 0;
  qFix = ggaQuality;
}

}  // namespace

void begin(uint8_t gpsMode) {
  mode = modeFor(gpsMode);
  Serial2.setRxBufferSize(2048);
#ifdef GPS_UBLOX_M10
  Serial2.begin(kBauds[0], SERIAL_8N1, GPS_UART_RX, GPS_UART_TX);
  startDetect(0);
  syslog::logf("[GPS] UART RX=%d TX=%d, szukam modulu u-blox M10\n", GPS_UART_RX, GPS_UART_TX);
#else
  Serial2.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RX, GPS_UART_TX);
  syslog::logf("[GPS] UART RX=%d TX=%d %d bps\n", GPS_UART_RX, GPS_UART_TX, GPS_UART_BAUD);
#endif
}

bool update(rc::GpsFix& out) {
  while (Serial2.available()) feed((char)Serial2.read());

#ifdef GPS_UBLOX_M10
  autoConfigStep();
  if (cfg == Cfg::Run && !cfgUnanswered) {
    const uint32_t req = diagReqMs;
    const bool want = req != 0 && millis() - req < kDiagHoldMs;
    if (want != diagOn || diagSync) {
      setDiagOutput(want);
      diagOn = want;
      diagSync = false;
    }
  }
#endif
  publish();
#ifdef GPS_DEBUG
  debugStep();
#endif
#ifdef GPS_UBLOX_M10
  if (cfg != Cfg::Run) return false;
#endif

  // Jedna pozycja na epoke: RMC i GGA niosa ten sam czas, wysylamy przy pierwszym.
  if (!gps.time.isUpdated() || !gps.location.isValid() || !gps.date.isValid()) return false;
  uint32_t epoch = gps.time.value();
  if (epoch == lastEpoch) return false;
  // TinyGPS++ zatwierdza czas z kazdego RMC/GGA, a pozycje, predkosc i kurs tylko z fixem
  // (a location.isValid() po pierwszym fixie juz nie wraca do false). Po utracie fixa czas
  // idzie dalej, a pozycja stoi - taka "zamrozona" pozycja z nowym czasem nie moze isc dalej.
  if (gps.location.age() > gps.time.age()) return false;
  lastEpoch = epoch;

  out.year = gps.date.year();
  out.month = gps.date.month();
  out.day = gps.date.day();
  out.hour = gps.time.hour();
  out.minute = gps.time.minute();
  out.second = gps.time.second();
  out.millis = gps.time.centisecond() * 10;
  out.lat = gps.location.lat();
  out.lon = gps.location.lng();
  out.altM = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
  out.speedKmh = gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f;
  out.bearingDeg = gps.course.isValid() ? (float)gps.course.deg() : 0.0f;
  out.satellites = (uint8_t)min<uint32_t>(usedSats(), 62);  // RaceChrono: 6 bitow, 63 = brak
  // RaceChrono: 1 = GPS, 2 = z korekcja (SBAS/EGNOS, DGPS, RTK)
  uint8_t q = ggaQuality;
  out.fixQuality = (q == 2 || q == 4 || q == 5) ? 2 : 1;
  out.hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 9.9f;
  out.vdop = out.hdop;
  return true;
}

State state() { return stateSnap; }

Quality quality() { return Quality{qUsed, qTracked, qHdop, qFix, qAcc}; }

void requestDiag() { diagReqMs = millis() | 1; }

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
void requestDiag() {}
void stop() {}
void diag(Diag& out) {
  out = Diag{};
  out.satAge = out.rfAge = out.navAge = UINT32_MAX;
  out.modeName = "brak modulu GPS";
}
}  // namespace gpsrx

#endif
