#include "GpsReceiver.h"

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
  uint32_t galileo, beidou;  // 1 = wlaczony (BeiDou razem z QZSS)
  const char* name;
};

Mode modeFor(uint8_t m) {
  switch (m) {
    case 25:
      return {25, 0, 0, "GPS"};  // najwiecej punktow, najmniej satelitow
    case 10:
      return {10, 1, 1, "GPS+Galileo+BeiDou+QZSS"};  // najwiecej satelitow, najlepszy HDOP
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
uint8_t ubxTail[8] = {};

// Szuka w strumieniu (NMEA i UBX wymieszane) ramki B5 62 05 0x 02 00 06 8A.
void ubxScan(uint8_t c) {
  memmove(ubxTail, ubxTail + 1, sizeof(ubxTail) - 1);
  ubxTail[sizeof(ubxTail) - 1] = c;
  if (ubxTail[0] != 0xB5 || ubxTail[1] != 0x62 || ubxTail[2] != 0x05 || ubxTail[4] != 0x02 ||
      ubxTail[5] != 0x00 || ubxTail[6] != 0x06 || ubxTail[7] != 0x8A)
    return;
  if (ubxTail[3] == 0x01) {
    ackCount++;
    Serial.println("[GPS] modul potwierdzil konfiguracje (UBX-ACK-ACK)");
  } else if (ubxTail[3] == 0x00) {
    nakCount++;
    Serial.println("[GPS] modul ODRZUCIL konfiguracje (UBX-ACK-NAK)");
  }
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

// UBX-CFG-VALSET (0x06 0x8A), warstwa RAM: po odcieciu zasilania modul wraca do
// ustawien fabrycznych, a my konfigurujemy go od nowa przy kazdym starcie.
void sendValset(const KeyVal* kv, size_t count) {
  uint8_t msg[8 + 4 + 12 * 8 + 2];
  size_t n = 0;
  msg[n++] = 0xB5;
  msg[n++] = 0x62;
  msg[n++] = 0x06;
  msg[n++] = 0x8A;
  size_t lenPos = n;
  n += 2;
  msg[n++] = 0x00;  // version
  msg[n++] = 0x01;  // layers: RAM
  msg[n++] = 0x00;
  msg[n++] = 0x00;
  for (size_t i = 0; i < count; i++) {
    for (int b = 0; b < 4; b++) msg[n++] = (kv[i].key >> (8 * b)) & 0xFF;
    size_t vs = valueSize(kv[i].key);
    for (size_t b = 0; b < vs; b++) msg[n++] = (b < 4) ? (kv[i].value >> (8 * b)) & 0xFF : 0;
  }
  uint16_t len = n - 6;
  msg[lenPos] = len & 0xFF;
  msg[lenPos + 1] = len >> 8;
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < n; i++) {
    ckA += msg[i];
    ckB += ckA;
  }
  msg[n++] = ckA;
  msg[n++] = ckB;
  Serial2.write(msg, n);
}

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
  baudIdx = idx % kBaudCount;
  Serial2.updateBaudRate(kBauds[baudIdx]);
  cfg = Cfg::Detect;
  windowStartMs = millis();
  passedAtStart = gps.passedChecksum();
}

void enterRun() {
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
  // Zmiana systemow restartuje odbiornik GNSS - dlatego wysylana pierwsza, a potem wg u-blox:
  // czekamy na ACK i jeszcze 0.5 s, zanim pojdzie nastepna komenda.
  const KeyVal signals[] = {
      {CFG_SIGNAL_GAL_ENA, mode.galileo},
      {CFG_SIGNAL_BDS_ENA, mode.beidou},
      {CFG_SIGNAL_QZSS_ENA, mode.beidou},  // QZSS (wzmocnienie GPS) razem z pelnym zestawem
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
      Serial.printf("[GPS] UWAGA: modul nie odpowiada na komendy - sprawdz przewod IO22 -> RX modulu. "
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

  Serial.printf("[GPS] u-blox M10: %u Hz (%s), automotive, RMC+GGA, %lu bps%s\n", mode.hz, mode.name,
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
        Serial.printf("[GPS] wykryto modul na %lu bps\n", (unsigned long)kBauds[baudIdx]);
        configureModule();
      } else if (now - windowStartMs > kDetectWindowMs) {
        startDetect(baudIdx + 1);
      }
      break;
    case Cfg::Verify:
      if (gps.passedChecksum() - passedAtStart >= 2) {
        enterRun();
        if (nakCount > 0)
          Serial.printf("[GPS] UWAGA: NAK x%lu - modul nie przyjal czesci ustawien\n", (unsigned long)nakCount);
        else
          Serial.printf("[GPS] konfiguracja OK (ACK x%lu)\n", (unsigned long)ackCount);
      } else if (now - windowStartMs > kVerifyTimeoutMs) {
        Serial.println("[GPS] brak danych po konfiguracji, szukam od nowa");
        startDetect(0);
      }
      break;
    case Cfg::Run:
      // Liczy sie tylko poprawne zdanie: przy niezgodnej predkosci UART (np. modul po resecie
      // wrocil na fabryczna) bajty dalej przychodza, tylko ze to smieci.
      if (now - lastValidMs > kNoDataTimeoutMs) {
        Serial.println("[GPS] brak poprawnych danych z modulu, szukam od nowa");
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
          Serial.printf("[GPS] czestotliwosc spadla do %lu Hz (reset modulu?), konfiguruje od nowa\n",
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
  if (gps.encode(c)) lastValidMs = millis();  // true = zdanie z poprawna suma kontrolna
#ifdef GPS_DEBUG
  dbgChars++;
#endif
#ifdef GPS_UBLOX_M10
  ubxScan((uint8_t)c);
#endif

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
  Serial.printf("[GPS] UART RX=%d TX=%d, szukam modulu u-blox M10\n", GPS_UART_RX, GPS_UART_TX);
#else
  Serial2.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RX, GPS_UART_TX);
  Serial.printf("[GPS] UART RX=%d TX=%d %d bps\n", GPS_UART_RX, GPS_UART_TX, GPS_UART_BAUD);
#endif
}

bool update(rc::GpsFix& out) {
  while (Serial2.available()) feed((char)Serial2.read());

#ifdef GPS_UBLOX_M10
  autoConfigStep();
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

}  // namespace gpsrx

#else  // bez GPS_UART_RX: brak modulu

namespace gpsrx {
void begin(uint8_t) {}
bool update(rc::GpsFix&) { return false; }
State state() { return State::Off; }
Quality quality() { return {}; }
}  // namespace gpsrx

#endif
