#include "RaceChronoBle.h"

#include <NimBLEDevice.h>

namespace rc {
namespace {

// Wzory RaceChrono (https://racechrono.com/support/equations/identifiers).
// Wartosci przychodza jako int32, wiec mnozymy, zeby nie stracic precyzji.
// Czasy sa w sekundach -> *1000 = ms. Predkosc w m/s -> *36 = km/h*10.
const char* const kEquations[MONITOR_COUNT] = {
    "channel(device(lap), delta_lap_time)*1000",
    "channel(device(lap), lap_time)*1000",
    "channel(device(lap), previous_lap_time)*1000",
    "channel(device(lap), best_lap_time)*1000",
    "channel(device(lap), lap_number)",
    "channel(device(gps), speed)*36",
    "channel(device(lap), best_lap_number)",
    "channel(device(lap), comparison_lap_time)*1000",
    "channel(device(lap), comparison_lap_number)",
    "channel(device(lap), previous_lap_number)",
    // delta_speed dziala tylko z device(calc) (z lap/gps RaceChrono wysyla "brak wartosci")
    "channel(device(calc), delta_speed)*36",
};

constexpr uint16_t kServiceUuid = 0x1FF8;
constexpr uint16_t kCanMainUuid = 0x0001;
constexpr uint16_t kCanFilterUuid = 0x0002;
constexpr uint16_t kGpsMainUuid = 0x0003;
constexpr uint16_t kGpsTimeUuid = 0x0004;
constexpr uint16_t kMonitorCfgUuid = 0x0005;
constexpr uint16_t kMonitorValUuid = 0x0006;

constexpr uint8_t kCmdRemoveAll = 0;
constexpr uint8_t kCmdAddIncomplete = 2;
constexpr uint8_t kCmdAddComplete = 3;
constexpr uint8_t kCmdUpdateAll = 4;

constexpr size_t kPayloadMax = 17;  // bajty 3..19 ramki
constexpr uint32_t kIndicateTimeoutMs = 500;
constexpr uint32_t kReplyTimeoutMs = 1500;
constexpr int32_t kInvalidValue = 0x7FFFFFFF;

enum class CfgState : uint8_t { Idle, Start, SendChunk, WaitReply, SendUpdateAll, Done };

NimBLECharacteristic* monCfgChr = nullptr;
NimBLECharacteristic* gpsMainChr = nullptr;
NimBLECharacteristic* gpsTimeChr = nullptr;
uint8_t gpsSync = 0;                    // 3 bity, zmienia sie razem z 0x0004
uint32_t gpsLastHourDate = 0xFFFFFFFF;  // ostatnia wartosc 0x0004

// Ustawiane z callbackow NimBLE (inny task), czytane w loop().
volatile bool connected = false;
volatile bool subscribed = false;
volatile bool restartCfg = false;
volatile bool indicateInFlight = false;
volatile int16_t replyFor = -1;  // monitor, ktorego dotyczyla ostatnia odpowiedz RaceChrono (-1 = zaden)

uint32_t indicateSentMs = 0;
CfgState state = CfgState::Idle;
uint8_t curMonitor = 0;
size_t curOffset = 0;
uint8_t curSeq = 0;
uint32_t waitStartMs = 0;
bool configured = false;

// Ostatnia wartosc kazdego monitora, kInvalidValue = brak. Jedna liczba 32-bit zamiast pary
// wartosc + flaga: odczyt z innego zadania jest atomowy, wiec nie da sie zlapac nowej wartosci
// ze stara flaga waznosci.
volatile int32_t values[MONITOR_COUNT];

// Statystyki odbioru (do sprawdzenia, jak czesto RaceChrono wysyla dane).
constexpr uint32_t kStatsPeriodMs = 5000;
const char* const kMonitorNames[MONITOR_COUNT] = {"delta", "current", "prev", "best", "lap", "speed", "best_lap", "ref", "ref_lap", "last_lap", "dspeed"};
volatile uint32_t statWrites = 0;
volatile uint32_t statValues[MONITOR_COUNT] = {};
uint32_t statLastMs = 0;

[[maybe_unused]] void printStats() {
  uint32_t now = millis();
  if (now - statLastMs < kStatsPeriodMs) return;
  float sec = (now - statLastMs) / 1000.0f;
  statLastMs = now;
  if (!configured) {
    statWrites = 0;
    for (auto& v : statValues) v = 0;
    return;
  }
  Serial.printf("[RC] odbior: %.1f zapisow/s |", statWrites / sec);
  statWrites = 0;
  for (int i = 0; i < MONITOR_COUNT; i++) {
    Serial.printf(" %s %.1f/s", kMonitorNames[i], statValues[i] / sec);
    statValues[i] = 0;
  }
  Serial.println();
}

void resetValues() {
  for (auto& v : values) v = kInvalidValue;
}

bool sendIndicate(const uint8_t* data, size_t len) {
  if (!monCfgChr->indicate(data, len)) return false;
  indicateInFlight = true;
  indicateSentMs = millis();
  return true;
}

// RaceChrono potwierdza kazde INDICATE; nastepne wysylamy dopiero po potwierdzeniu.
bool canSend() {
  if (indicateInFlight && millis() - indicateSentMs < kIndicateTimeoutMs) return false;
  indicateInFlight = false;
  return true;
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    connected = true;
    Serial.printf("[BLE] polaczono: %s\n", info.getAddress().toString().c_str());
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    connected = false;
    subscribed = false;
    restartCfg = true;
    resetValues();
    Serial.printf("[BLE] rozlaczono (reason %d), reklamuje dalej\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

class MonitorCfgCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t subValue) override {
    subscribed = (subValue & 0x02) != 0;  // bit 1 = INDICATE
    restartCfg = true;
    Serial.printf("[BLE] monitor cfg subscribe=%u\n", subValue);
  }

  void onStatus(NimBLECharacteristic*, int code) override {
    indicateInFlight = false;
    if (code != 0 && code != BLE_HS_EDONE) Serial.printf("[BLE] indicate status %d\n", code);
  }

  // Odpowiedz RaceChrono na "Add": [wynik][id] (+ szczegoly bledu wzoru).
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
    NimBLEAttValue v = chr->getValue();
    const uint8_t* d = v.data();
    if (v.size() < 2) return;
    replyFor = d[1];
    if (d[0] == 0) {
      Serial.printf("[RC] monitor %u OK\n", d[1]);
    } else if (d[0] == 1) {
      Serial.printf("[RC] monitor %u: payload poza kolejnoscia\n", d[1]);
    } else if (d[0] == 2 && v.size() >= 8) {
      uint16_t type = (d[2] << 8) | d[3];
      uint16_t pos = (d[4] << 8) | d[5];
      uint16_t len = (d[6] << 8) | d[7];
      Serial.printf("[RC] monitor %u: blad wzoru typ=%u pozycja=%u dlugosc=%u (%s)\n", d[1], type, pos, len,
                    d[1] < MONITOR_COUNT ? kEquations[d[1]] : "?");
    } else {
      Serial.printf("[RC] monitor %u: wynik %u\n", d[1], d[0]);
    }
  }
};

// Wartosci: powtarzane rekordy [id][int32 big-endian], max 4 na zapis.
class MonitorValCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
    NimBLEAttValue v = chr->getValue();
    const uint8_t* d = v.data();
    statWrites = statWrites + 1;
    for (size_t i = 0; i + 5 <= v.size(); i += 5) {
      uint8_t id = d[i];
      if (id >= MONITOR_COUNT) continue;
      statValues[id] = statValues[id] + 1;
      int32_t val = (int32_t)(((uint32_t)d[i + 1] << 24) | ((uint32_t)d[i + 2] << 16) |
                              ((uint32_t)d[i + 3] << 8) | (uint32_t)d[i + 4]);
      values[id] = val;
#ifdef RC_DEBUG_VALUES
      Serial.printf("[RAW] t=%lu %s(%u)=%ld\n", (unsigned long)millis(), kMonitorNames[id], id, (long)val);
#endif
    }
  }
};

ServerCallbacks serverCallbacks;
MonitorCfgCallbacks monitorCfgCallbacks;
MonitorValCallbacks monitorValCallbacks;

}  // namespace

void begin(const char* deviceName) {
  resetValues();

  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(9);  // dBm

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);

  NimBLEService* svc = server->createService(NimBLEUUID(kServiceUuid));

  // CAN-Bus: nieuzywane, ale obecne jak w referencyjnej implementacji DIY.
  svc->createCharacteristic(NimBLEUUID(kCanMainUuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  svc->createCharacteristic(NimBLEUUID(kCanFilterUuid), NIMBLE_PROPERTY::WRITE);

  // GPS: pozycje z modulu GPS albo symulatora (sendGps()).
  gpsMainChr = svc->createCharacteristic(NimBLEUUID(kGpsMainUuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  gpsTimeChr = svc->createCharacteristic(NimBLEUUID(kGpsTimeUuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  monCfgChr =
      svc->createCharacteristic(NimBLEUUID(kMonitorCfgUuid), NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  monCfgChr->setCallbacks(&monitorCfgCallbacks);

  NimBLECharacteristic* monValChr = svc->createCharacteristic(NimBLEUUID(kMonitorValUuid), NIMBLE_PROPERTY::WRITE_NR);
  monValChr->setCallbacks(&monitorValCallbacks);

  server->start();  // w NimBLE 2.x uslugi startuja razem z serwerem

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(deviceName);
  adv->addServiceUUID(svc->getUUID());
  adv->enableScanResponse(true);
  adv->start();

  Serial.printf("[BLE] reklamuje jako \"%s\"\n", deviceName);
}

void loop() {
#ifdef RC_DEBUG_STATS
  printStats();
#endif

  if (restartCfg) {
    restartCfg = false;
    configured = false;
    state = subscribed ? CfgState::Start : CfgState::Idle;
  }
  if (!connected || !subscribed) {
    state = CfgState::Idle;
    configured = false;
    return;
  }
  if (!canSend()) return;

  switch (state) {
    case CfgState::Idle:
    case CfgState::Done:
      break;

    case CfgState::Start: {
      uint8_t cmd = kCmdRemoveAll;
      if (sendIndicate(&cmd, 1)) {
        curMonitor = 0;
        curOffset = 0;
        curSeq = 0;
        state = CfgState::SendChunk;
      }
      break;
    }

    case CfgState::SendChunk: {
      const char* eq = kEquations[curMonitor];
      size_t len = strlen(eq);
      size_t n = min(kPayloadMax, len - curOffset);
      bool last = (curOffset + n >= len);

      uint8_t buf[3 + kPayloadMax];
      buf[0] = last ? kCmdAddComplete : kCmdAddIncomplete;
      buf[1] = curMonitor;
      buf[2] = curSeq;
      memcpy(buf + 3, eq + curOffset, n);

      if (curSeq == 0) replyFor = -1;
      if (!sendIndicate(buf, 3 + n)) break;  // sprobujemy w nastepnym obiegu

      curOffset += n;
      curSeq++;
      if (last) {
        waitStartMs = millis();
        state = CfgState::WaitReply;
      }
      break;
    }

    case CfgState::WaitReply: {
      // Odpowiedz musi dotyczyc biezacego monitora - spozniona (po timeoucie) odpowiedz
      // na poprzedni nie moze zaliczyc tego.
      const bool replied = replyFor == curMonitor;
      if (!replied && millis() - waitStartMs < kReplyTimeoutMs) break;
      if (!replied) Serial.printf("[RC] monitor %u: brak odpowiedzi\n", curMonitor);
      curMonitor++;
      curOffset = 0;
      curSeq = 0;
      state = (curMonitor < MONITOR_COUNT) ? CfgState::SendChunk : CfgState::SendUpdateAll;
      break;
    }

    case CfgState::SendUpdateAll: {
      uint8_t cmd = kCmdUpdateAll;
      if (sendIndicate(&cmd, 1)) {
        state = CfgState::Done;
        configured = true;
        Serial.println("[RC] konfiguracja monitorow zakonczona");
      }
      break;
    }
  }
}

static void put24(uint8_t* b, uint32_t v) {
  b[0] = v >> 16;
  b[1] = v >> 8;
  b[2] = v;
}
static void put16(uint8_t* b, uint16_t v) {
  b[0] = v >> 8;
  b[1] = v;
}
static void put32(uint8_t* b, int32_t v) {
  uint32_t u = (uint32_t)v;
  b[0] = u >> 24;
  b[1] = u >> 16;
  b[2] = u >> 8;
  b[3] = u;
}

void sendGps(const GpsFix& f) {
  if (!gpsMainChr) return;

  // 0x0004: sync (3 bity) + godzina i data (21 bitow). Zmiana -> nowy sync.
  uint32_t hourDate =
      (uint32_t)(f.year - 2000) * 8928 + (uint32_t)(f.month - 1) * 744 + (uint32_t)(f.day - 1) * 24 + f.hour;
  if (hourDate != gpsLastHourDate) {
    gpsLastHourDate = hourDate;
    gpsSync = (gpsSync + 1) & 0x07;
    uint8_t t[3];
    put24(t, ((uint32_t)gpsSync << 21) | (hourDate & 0x1FFFFF));
    gpsTimeChr->setValue(t, sizeof(t));
  }
  // Czas wysylamy w kazdym cyklu razem z pozycja (ta sama wartosc sync).
  if (connected) gpsTimeChr->notify();

  // 0x0003: glowna ramka GPS, 20 bajtow
  uint8_t b[20];
  uint32_t timeFromHour = f.minute * 30000u + f.second * 500u + f.millis / 2u;
  put24(b, ((uint32_t)gpsSync << 21) | (timeFromHour & 0x1FFFFF));
  b[3] = (uint8_t)((f.fixQuality & 0x03) << 6) | (f.satellites & 0x3F);
  put32(b + 4, (int32_t)lround(f.lat * 10000000.0));
  put32(b + 8, (int32_t)lround(f.lon * 10000000.0));
  put16(b + 12, (uint16_t)(lround((f.altM + 500.0f) * 10.0f) & 0x7FFF));
  long speed100 = lround(f.speedKmh * 100.0f);
  put16(b + 14, speed100 <= 0x7FFF ? (uint16_t)speed100 : (uint16_t)((lround(f.speedKmh * 10.0f) & 0x7FFF) | 0x8000));
  put16(b + 16, (uint16_t)(lround(f.bearingDeg * 100.0f) % 36000));
  b[18] = (uint8_t)min(254L, lround(f.hdop * 10.0f));
  b[19] = (uint8_t)min(254L, lround(f.vdop * 10.0f));

  gpsMainChr->setValue(b, sizeof(b));
  if (connected) gpsMainChr->notify();
}

bool isConnected() { return connected; }
bool isConfigured() { return configured; }

bool get(MonitorId id, int32_t& out) {
  if (id >= MONITOR_COUNT) return false;
  const int32_t v = values[id];  // jeden odczyt
  if (v == kInvalidValue) return false;
  out = v;
  return true;
}

}  // namespace rc
