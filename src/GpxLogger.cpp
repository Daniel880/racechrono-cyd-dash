#include "GpxLogger.h"

#ifdef GPX_LOG

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <freertos/ringbuf.h>

namespace gpx {
namespace {

// Slot microSD na CYD (osobna magistrala VSPI, nie koliduje z ekranem na HSPI).
constexpr int kSdSck = 18;
constexpr int kSdMiso = 19;
constexpr int kSdMosi = 23;
constexpr int kSdCs = 5;
constexpr uint32_t kSdFreq = 20000000;

constexpr const char* kDir = "/gpx";
constexpr uint32_t kFlushEveryMs = 2000;  // zapis bufora na karte
constexpr size_t kQueueBytes = 16384;     // ~3 s danych przy 10 Hz

const char kFooter[] = "</trkseg></trk></gpx>\n";

// addFix()/nmeaLine() sa wolane z zadania GPS i tylko wrzucaja dane do kolejki;
// na karte zapisuje osobne zadanie na rdzeniu 0. Dzieki temu wolny zapis/flush karty SD
// (potrafi trwac kilkadziesiat ms) nie opoznia ani GPS, ani rysowania ekranu.
enum ItemType : uint8_t { ItemFix = 1, ItemNmea = 2 };

SPIClass sdSpi(VSPI);
File gpxFile;
File nmeaFile;
RingbufHandle_t queue = nullptr;
volatile State st = State::Off;
char baseName[40] = "";
uint32_t lastFlushMs = 0;
volatile uint32_t dropped = 0;

// GPX jest poprawny po kazdej paczce punktow: stopka na koncu, potem cofamy sie
// przed nia, zeby nastepne punkty ja nadpisaly. Odciecie zasilania nie psuje pliku.
void writeFooter() {
  size_t pos = gpxFile.position();
  gpxFile.print(kFooter);
  gpxFile.seek(pos);
}

// Nazwa z czasu pierwszego fixa (UTC), np. "2026-11-13_14-05-32"; przy kolizji "_2", "_3"...
bool openNewFiles(const rc::GpsFix& f) {
  if (!SD.exists(kDir)) SD.mkdir(kDir);
  char stamp[32], gpxPath[64], nmeaPath[64];  // zapas: kompilator liczy pola uint8_t jako 0..255
  snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u_%02u-%02u-%02u", f.year, f.month, f.day, f.hour, f.minute,
           f.second);
  for (int n = 1; n <= 99; n++) {
    if (n == 1)
      snprintf(baseName, sizeof(baseName), "%s", stamp);
    else
      snprintf(baseName, sizeof(baseName), "%s_%d", stamp, n);
    snprintf(gpxPath, sizeof(gpxPath), "%s/%s.gpx", kDir, baseName);
    snprintf(nmeaPath, sizeof(nmeaPath), "%s/%s.nmea", kDir, baseName);
    if (SD.exists(gpxPath) || SD.exists(nmeaPath)) continue;

    gpxFile = SD.open(gpxPath, "w+");
    nmeaFile = SD.open(nmeaPath, FILE_WRITE);
    if (!gpxFile || !nmeaFile) return false;

    gpxFile.print(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<gpx version=\"1.1\" creator=\"RaceChrono CYD display\"\n"
        " xmlns=\"http://www.topografix.com/GPX/1/1\"\n"
        " xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v2\">\n"
        "<trk><name>");
    gpxFile.print(baseName);
    gpxFile.print("</name><trkseg>\n");
    writeFooter();
    gpxFile.flush();
    Serial.printf("[GPX] zapis do %s i %s\n", gpxPath, nmeaPath);
    return true;
  }
  return false;
}

void enqueue(ItemType type, const void* data, size_t len) {
  uint8_t item[1 + 160];
  if (!queue || len > sizeof(item) - 1) return;
  item[0] = type;
  memcpy(item + 1, data, len);
  if (xRingbufferSend(queue, item, 1 + len, 0) != pdTRUE) dropped = dropped + 1;
}

// true = dopisano punkt do GPX
bool writeFix(const rc::GpsFix& f) {
  if (st == State::Waiting) {
    if (!openNewFiles(f)) {
      Serial.println("[GPX] nie moge utworzyc plikow");
      st = State::Error;
      return false;
    }
    st = State::Logging;
  }
  if (st != State::Logging) return false;

  char buf[400];
  snprintf(buf, sizeof(buf),
           "<trkpt lat=\"%.7f\" lon=\"%.7f\"><ele>%.1f</ele>"
           "<time>%04u-%02u-%02uT%02u:%02u:%02u.%03uZ</time>"
           "<sat>%u</sat><hdop>%.1f</hdop>"
           "<extensions><gpxtpx:TrackPointExtension>"
           "<gpxtpx:speed>%.2f</gpxtpx:speed><gpxtpx:course>%.1f</gpxtpx:course>"
           "</gpxtpx:TrackPointExtension></extensions></trkpt>\n",
           f.lat, f.lon, f.altM, f.year, f.month, f.day, f.hour, f.minute, f.second, f.millis, f.satellites,
           f.hdop, f.speedKmh / 3.6f, f.bearingDeg);
  gpxFile.print(buf);
  return true;
}

void writeQueued() {
  bool newPoints = false;
  size_t size;
  while (uint8_t* item = (uint8_t*)xRingbufferReceive(queue, &size, 0)) {
    if (item[0] == ItemFix && size == 1 + sizeof(rc::GpsFix)) {
      rc::GpsFix f;
      memcpy(&f, item + 1, sizeof(f));
      if (writeFix(f)) newPoints = true;
    } else if (item[0] == ItemNmea && st == State::Logging) {
      nmeaFile.write(item + 1, size - 1);
      nmeaFile.print("\r\n");
    }
    vRingbufferReturnItem(queue, item);
  }
  if (newPoints) writeFooter();

  if (st == State::Logging && millis() - lastFlushMs >= kFlushEveryMs) {
    lastFlushMs = millis();
    gpxFile.flush();
    nmeaFile.flush();
    if (dropped) {
      Serial.printf("[GPX] kolejka pelna, pominieto %lu wpisow\n", (unsigned long)dropped);
      dropped = 0;
    }
  }
}

void sdTask(void*) {
  while (st == State::Waiting || st == State::Logging) {
    writeQueued();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  vTaskDelete(nullptr);  // blad karty - koniec logowania
}

}  // namespace

void begin() {
  sdSpi.begin(kSdSck, kSdMiso, kSdMosi, kSdCs);
  if (!SD.begin(kSdCs, sdSpi, kSdFreq)) {
    Serial.println("[GPX] brak karty SD");
    st = State::Error;
    return;
  }
  queue = xRingbufferCreate(kQueueBytes, RINGBUF_TYPE_NOSPLIT);
  Serial.println("[GPX] karta SD OK, czekam na fix GPS");
  st = State::Waiting;
  lastFlushMs = millis();
  // rdzen 0 (tam dziala tez BLE), niski priorytet; ekran i GPS sa na rdzeniu 1
  xTaskCreatePinnedToCore(sdTask, "sd", 6144, nullptr, 1, nullptr, 0);
}

void addFix(const rc::GpsFix& f) {
  if (st != State::Waiting && st != State::Logging) return;
  // tylko sensowne dane: fix i poprawna data/czas (od pierwszego takiego punktu powstaje plik)
  if (f.fixQuality == 0 || f.year < 2020 || f.month < 1 || f.month > 12 || f.day < 1 || f.day > 31 ||
      f.hour > 23 || f.minute > 59 || f.second > 59)
    return;
  enqueue(ItemFix, &f, sizeof(f));
}

void nmeaLine(const char* line) {
  if (st != State::Logging) return;
  enqueue(ItemNmea, line, strlen(line));
}

State state() { return st; }

}  // namespace gpx

#else  // bez GPX_LOG: puste funkcje

namespace gpx {
void begin() {}
void addFix(const rc::GpsFix&) {}
void nmeaLine(const char*) {}
State state() { return State::Off; }
}  // namespace gpx

#endif
