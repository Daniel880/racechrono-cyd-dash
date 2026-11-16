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
#ifndef GPS_RATE_HZ
#define GPS_RATE_HZ 10
#endif

namespace gpsrx {
namespace {

constexpr uint32_t kNoDataTimeoutMs = 3000;
constexpr uint32_t kMaxFixAgeMs = 1500;

TinyGPSPlus gps;
char line[128];
size_t lineLen = 0;
uint32_t lastCharMs = 0;
uint32_t lastEpoch = 0xFFFFFFFF;  // czas ostatnio wyslanej pozycji (hhmmsscc)

#ifdef GPS_UBLOX_M10
// ---------- Autokonfiguracja u-blox M10 (UBX-CFG-VALSET) ----------
// Stan: szukamy predkosci UART -> konfigurujemy -> przechodzimy na GPS_UART_BAUD -> praca.
// Po utracie danych (odlaczenie / reset modulu) wszystko od nowa.
enum class Cfg : uint8_t { Detect, Verify, Run };
constexpr uint32_t kBauds[] = {115200, 38400, 9600, 57600, 230400};
constexpr size_t kBaudCount = sizeof(kBauds) / sizeof(kBauds[0]);
constexpr uint32_t kDetectWindowMs = 1200;
constexpr uint32_t kVerifyTimeoutMs = 2500;

Cfg cfg = Cfg::Detect;
size_t baudIdx = 0;
uint32_t windowStartMs = 0;
uint32_t passedAtStart = 0;

// Klucze konfiguracji M10 (u-blox M10 Interface Description)
constexpr uint32_t CFG_RATE_MEAS = 0x30210001;          // U2, okres pomiaru [ms]
constexpr uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021;    // U1, 4 = automotive
constexpr uint32_t CFG_UART1_BAUDRATE = 0x40520001;     // U4
constexpr uint32_t CFG_MSGOUT_NMEA_GGA_UART1 = 0x209100bb;
constexpr uint32_t CFG_MSGOUT_NMEA_RMC_UART1 = 0x209100ac;
constexpr uint32_t CFG_MSGOUT_NMEA_GLL_UART1 = 0x209100ca;
constexpr uint32_t CFG_MSGOUT_NMEA_GSA_UART1 = 0x209100c0;
constexpr uint32_t CFG_MSGOUT_NMEA_GSV_UART1 = 0x209100c5;
constexpr uint32_t CFG_MSGOUT_NMEA_VTG_UART1 = 0x209100b1;

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

void startDetect(size_t idx) {
  baudIdx = idx % kBaudCount;
  Serial2.updateBaudRate(kBauds[baudIdx]);
  cfg = Cfg::Detect;
  windowStartMs = millis();
  passedAtStart = gps.passedChecksum();
}

void configureModule() {
  const KeyVal settings[] = {
      {CFG_RATE_MEAS, 1000 / GPS_RATE_HZ},
      {CFG_NAVSPG_DYNMODEL, 4},
      {CFG_MSGOUT_NMEA_GGA_UART1, 1},
      {CFG_MSGOUT_NMEA_RMC_UART1, 1},
      {CFG_MSGOUT_NMEA_GLL_UART1, 0},
      {CFG_MSGOUT_NMEA_GSA_UART1, 0},
      {CFG_MSGOUT_NMEA_GSV_UART1, 0},
      {CFG_MSGOUT_NMEA_VTG_UART1, 0},
  };
  sendValset(settings, sizeof(settings) / sizeof(settings[0]));
  Serial2.flush();
  delay(50);

  // Zmiana predkosci na koncu: modul przelacza sie od razu, my za nim.
  const KeyVal baud[] = {{CFG_UART1_BAUDRATE, GPS_UART_BAUD}};
  sendValset(baud, 1);
  Serial2.flush();
  delay(100);
  Serial2.updateBaudRate(GPS_UART_BAUD);

  Serial.printf("[GPS] u-blox M10: %d Hz, automotive, RMC+GGA, %d bps\n", GPS_RATE_HZ, GPS_UART_BAUD);
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
        cfg = Cfg::Run;
        Serial.println("[GPS] konfiguracja OK");
      } else if (now - windowStartMs > kVerifyTimeoutMs) {
        Serial.println("[GPS] brak danych po konfiguracji, szukam od nowa");
        startDetect(0);
      }
      break;
    case Cfg::Run:
      if (now - lastCharMs > kNoDataTimeoutMs) {
        Serial.println("[GPS] utrata danych z modulu, szukam od nowa");
        startDetect(0);
      }
      break;
  }
}
#endif  // GPS_UBLOX_M10

}  // namespace

void begin() {
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
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    lastCharMs = millis();
    gps.encode(c);

    // Surowe linie NMEA do backupu
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        line[lineLen] = '\0';
        if (line[0] == '$') gpx::nmeaLine(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }

#ifdef GPS_UBLOX_M10
  autoConfigStep();
  if (cfg != Cfg::Run) return false;
#endif

  // Jedna pozycja na epoke: RMC i GGA niosa ten sam czas, wysylamy przy pierwszym.
  if (!gps.time.isUpdated() || !gps.location.isValid() || !gps.date.isValid()) return false;
  uint32_t epoch = gps.time.value();
  if (epoch == lastEpoch) return false;
  if (gps.location.age() > kMaxFixAgeMs) return false;
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
  out.satellites = gps.satellites.isValid() ? (uint8_t)min<uint32_t>(gps.satellites.value(), 62) : 0;
  out.fixQuality = 1;
  out.hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 9.9f;
  out.vdop = out.hdop;
  return true;
}

State state() {
  if (lastCharMs == 0 || millis() - lastCharMs > kNoDataTimeoutMs) return State::NoData;
#ifdef GPS_UBLOX_M10
  if (cfg != Cfg::Run) return State::NoData;
#endif
  if (!gps.location.isValid() || gps.location.age() > kMaxFixAgeMs) return State::NoFix;
  return State::Fix;
}

}  // namespace gpsrx

#else  // bez GPS_UART_RX: brak modulu

namespace gpsrx {
void begin() {}
bool update(rc::GpsFix&) { return false; }
State state() { return State::Off; }
}  // namespace gpsrx

#endif
