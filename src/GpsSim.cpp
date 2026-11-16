#include "GpsSim.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <time.h>

namespace gpssim {
namespace {

// --- ustawienia toru ---
constexpr double kCenterLat = 52.0;  // srodek kola (pole w centralnej Polsce)
constexpr double kCenterLon = 19.0;
constexpr double kRadiusM = 92.0;    // obwod ~580 m, okrazenie ~20 s
constexpr int kPoints = 100;
constexpr uint32_t kRateMs = 100;    // 10 Hz

// Prędkosc: dwie "proste" (N i S, ~145 km/h) i dwa "zakrety" (E i W, ~75 km/h).
constexpr float kSpeedBase = 110.0f;
constexpr float kSpeedAmp = 35.0f;
constexpr float kLapVariation = 0.03f;  // kazde okrazenie losowo +-3% predkosci

// Sztuczny zegar GPS (UTC). Nie moze sie cofac po resecie ESP, bo RaceChrono
// ignoruje probki starsze niz te, ktore juz ma w sesji. Dlatego czas symulacji
// jest zapisywany w NVS i po starcie kontynuowany (+ zapas).
constexpr uint32_t kBaseEpoch = 1794398400;  // 2026-11-11 12:00:00 UTC
constexpr uint32_t kSaveEveryMs = 30000;
constexpr uint32_t kResumeMarginSec = 60;

constexpr double kMetersPerDegLat = 111320.0;

struct Waypoint {
  double lat, lon;
  float speedKmh;
  double distM;  // dystans od startu do tego punktu
};

Waypoint wp[kPoints + 1];  // ostatni = powtorzony pierwszy (zamkniecie petli)
double lapLengthM = 0;
double posM = 0;          // pozycja na okrazeniu
float lapFactor = 1.0f;   // mnoznik predkosci biezacego okrazenia
uint32_t lastUpdateMs = 0;
uint32_t lapCount = 0;

Preferences prefs;
uint32_t simStartSec = 0;  // sekundy od kBaseEpoch w chwili millis() == 0
uint32_t lastSaveMs = 0;

double distanceM(double lat1, double lon1, double lat2, double lon2) {
  double dy = (lat2 - lat1) * kMetersPerDegLat;
  double dx = (lon2 - lon1) * kMetersPerDegLat * cos(lat1 * DEG_TO_RAD);
  return sqrt(dx * dx + dy * dy);
}

float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double dy = lat2 - lat1;
  double dx = (lon2 - lon1) * cos(lat1 * DEG_TO_RAD);
  double b = atan2(dx, dy) * RAD_TO_DEG;
  return (float)(b < 0 ? b + 360.0 : b);
}

float randomLapFactor() {
  float r = (esp_random() % 10001) / 10000.0f;  // 0..1
  return 1.0f - kLapVariation + 2.0f * kLapVariation * r;
}

}  // namespace

void begin() {
  // Punkty rowno na okregu, start na polnocy, jazda zgodnie z zegarem (N -> E -> S -> W).
  for (int i = 0; i < kPoints; i++) {
    double a = 2.0 * PI * i / kPoints;
    wp[i].lat = kCenterLat + kRadiusM * cos(a) / kMetersPerDegLat;
    wp[i].lon = kCenterLon + kRadiusM * sin(a) / (kMetersPerDegLat * cos(kCenterLat * DEG_TO_RAD));
    wp[i].speedKmh = kSpeedBase + kSpeedAmp * (float)cos(2.0 * a);
  }
  wp[kPoints] = wp[0];

  wp[0].distM = 0;
  for (int i = 1; i <= kPoints; i++)
    wp[i].distM = wp[i - 1].distM + distanceM(wp[i - 1].lat, wp[i - 1].lon, wp[i].lat, wp[i].lon);
  lapLengthM = wp[kPoints].distM;

  lapFactor = randomLapFactor();
  lastUpdateMs = millis();

  prefs.begin("gpssim", false);
  uint32_t saved = prefs.getUInt("t", 0);
  simStartSec = saved ? saved + kResumeMarginSec : 0;
  prefs.putUInt("t", simStartSec);
  lastSaveMs = millis();

  Serial.printf("[SIM] GPS: kolo %d pkt, promien %.0f m, okrazenie %.0f m\n", kPoints, kRadiusM, lapLengthM);
  Serial.printf("[SIM] start/meta: %.7f, %.7f (linia N-S, jazda na wschod)\n", wp[0].lat, wp[0].lon);
}

bool update(rc::GpsFix& out) {
  if (millis() - lastUpdateMs < kRateMs) return false;
  // Sztywny krok 100 ms (bez dryfu), czas GPS = czas symulacji.
  lastUpdateMs += kRateMs;
  if (millis() - lastUpdateMs > 10 * kRateMs) lastUpdateMs = millis();  // po dlugiej przerwie nie nadrabiaj
  const uint32_t now = lastUpdateMs;
  const uint32_t dtMs = kRateMs;

  // Znajdz odcinek dla biezacej pozycji.
  int i = 0;
  while (i < kPoints - 1 && posM >= wp[i + 1].distM) i++;
  double segLen = wp[i + 1].distM - wp[i].distM;
  double frac = segLen > 0 ? (posM - wp[i].distM) / segLen : 0;

  float speed = (wp[i].speedKmh + (float)frac * (wp[i + 1].speedKmh - wp[i].speedKmh)) * lapFactor;

  out.lat = wp[i].lat + frac * (wp[i + 1].lat - wp[i].lat);
  out.lon = wp[i].lon + frac * (wp[i + 1].lon - wp[i].lon);
  out.speedKmh = speed;
  out.bearingDeg = bearingDeg(wp[i].lat, wp[i].lon, wp[i + 1].lat, wp[i + 1].lon);
  out.altM = 100.0f;
  out.satellites = 12;
  out.fixQuality = 1;
  out.hdop = 0.8f;
  out.vdop = 1.2f;

  uint32_t simSec = simStartSec + now / 1000;
  time_t t = (time_t)kBaseEpoch + simSec;
  struct tm tmv;
  gmtime_r(&t, &tmv);
  out.year = tmv.tm_year + 1900;
  out.month = tmv.tm_mon + 1;
  out.day = tmv.tm_mday;
  out.hour = tmv.tm_hour;
  out.minute = tmv.tm_min;
  out.second = tmv.tm_sec;
  out.millis = now % 1000;

  if (now - lastSaveMs >= kSaveEveryMs) {
    lastSaveMs = now;
    prefs.putUInt("t", simSec);
  }

  // Przesun pozycje na nastepny krok.
  posM += speed / 3.6 * (dtMs / 1000.0);
  if (posM >= lapLengthM) {
    posM -= lapLengthM;
    lapFactor = randomLapFactor();
    lapCount++;
#ifdef SIM_DEBUG
    Serial.printf("[SIM] okrazenie %lu, nowy mnoznik predkosci %.3f\n", (unsigned long)lapCount, lapFactor);
#endif
  }
  return true;
}

}  // namespace gpssim
