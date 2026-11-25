#include "GpxLogger.h"

#ifdef GPX_LOG

#include <Arduino.h>
#include <SPI.h>
// SdFat zamiast SD.h z ESP32: SD.h rezerwuje ~24 kB RAM (bufor 4 kB na kazdy mozliwy plik + VFS),
// a przy BLE + WiFi tyle brakuje - start WiFi konczyl sie bledem. SdFat: ~1 kB + nasze bufory.
#define DISABLE_FS_H_WARNING  // nie uzywamy typu File (zajety przez FS.h z ESP32), tylko FsFile
#include <SdFat.h>
#include <freertos/ringbuf.h>

#include "SysLog.h"

namespace gpx {
namespace {

// Slot microSD na CYD (osobna magistrala VSPI, nie koliduje z ekranem na HSPI).
constexpr int kSdSck = 18;
constexpr int kSdMiso = 19;
constexpr int kSdMosi = 23;
constexpr int kSdCs = 5;
constexpr uint32_t kSdMhz = 20;

constexpr const char* kDir = "/gpx";
constexpr uint32_t kSyncEveryMs = 2000;   // zapis na karte i rozmiaru pliku w katalogu
constexpr uint32_t kRetryMountMs = 5000;  // brak karty / blad zapisu: kolejna proba montowania
constexpr size_t kQueueBytes = 10240;     // ~1.5 s danych przy 25 Hz (NAV-PVT + NAV-DOP + punkt na pomiar)
constexpr size_t kOutBytes = 2048;        // bufor kazdego pliku w RAM

const char kFooter[] = "</trkseg></trk></gpx>\n";

// addFix()/ubxFrame() sa wolane z zadania GPS i tylko wrzucaja dane do kolejki;
// na karte zapisuje osobne zadanie na rdzeniu 0. Dzieki temu wolny zapis karty SD
// (potrafi trwac kilkadziesiat ms) nie opoznia ani GPS, ani rysowania ekranu.
enum ItemType : uint8_t { ItemFix = 1, ItemUbx = 2, ItemLog = 3 };

// Log systemowy (SysLog) na karcie: przetrwa wylaczenie zasilania, do pobrania na /files.
constexpr const char* kLogPath = "/gpx/log.txt";
constexpr const char* kLogOldPath = "/gpx/log_old.txt";
constexpr const char* kLogName = "log.txt";
constexpr uint32_t kLogMaxBytes = 1024 * 1024;  // wiecej = stary log idzie do log_old.txt

// Plik z buforem: SdFat ma jeden bufor sektora na cala karte, wiec drobne zapisy na przemian
// do dwoch plikow wymuszalyby zapis i odczyt sektora przy kazdym wpisie. Tu na karte ida
// kawalki po kOutBytes.
struct Out {
  FsFile f;
  size_t len = 0;
  uint8_t buf[kOutBytes];
};

SPIClass sdSpi(VSPI);
SdFs sd;
Out gpxOut;
Out ubxOut;
FsFile logFile;
bool logDirty = false;
bool bootLogged = false;  // wpisy sprzed pierwszego zamontowania karty juz w pliku
RingbufHandle_t queue = nullptr;
volatile State st = State::Off;
char baseName[40] = "";
uint32_t lastSyncMs = 0;
uint32_t lastMountMs = 0;
volatile uint32_t dropped = 0;
bool writeOk = true;  // false = ktorys zapis od ostatniego montowania karty sie nie udal
bool firstSyncLogged = false;
int lastMountErr = -1;  // ostatnio zalogowana przyczyna nieudanego montowania (bez powtarzania co 5 s)
rc::GpsFix fileTime{};  // czas z GPS do dat utworzenia/zmiany plikow
uint32_t cardSizeMB = 0;

// SdFat nie jest wielozadaniowy: zadanie zapisu i strona WWW (zadanie loop) uzywaja karty
// tylko pod ta blokada. Strona trzyma otwarte pliki miedzy wywolaniami - mountGen mowi,
// czy w miedzyczasie karta nie byla odmontowana (wtedy jej pliki sa niewazne).
SemaphoreHandle_t mtx = nullptr;
uint32_t mountGen = 0;
FsFile webDir, webFile;
uint32_t webDirGen = 0, webFileGen = 0;
bool webDirOpen = false;

// shutdown(): zadanie zapisu konczy sie samo na poczatku petli (nie w polowie zapisu).
volatile bool stopReq = false;
volatile bool taskStopped = false;

// Test zaklocen: do tej chwili bez operacji na karcie.
volatile uint32_t pauseUntil = 0;
bool pausedNow = false;
uint32_t pauseSkipped = 0;

// Daty plikow na karcie = czas UTC z GPS (bez tego SdFat wpisuje 2000-01-01).
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  *date = FS_DATE(fileTime.year, fileTime.month, fileTime.day);
  *time = FS_TIME(fileTime.hour, fileTime.minute, fileTime.second);
  *ms10 = (uint8_t)((fileTime.second & 1) * 100 + fileTime.millis / 10);
}

void drain(Out& o) {
  if (o.len && writeOk && o.f.write(o.buf, o.len) != o.len) writeOk = false;
  o.len = 0;
}

// Zapis z kontrola: karta wyjeta, bez kontaktu (wibracje) albo pelna -> zapisano mniej bajtow.
void put(Out& o, const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  while (len) {
    if (o.len == kOutBytes) drain(o);
    const size_t n = len < kOutBytes - o.len ? len : kOutBytes - o.len;
    memcpy(o.buf + o.len, p, n);
    o.len += n;
    p += n;
    len -= n;
  }
}
void put(Out& o, const char* s) { put(o, s, strlen(s)); }

// Zapis na karte i rozmiaru pliku w katalogu. Bufor zawsze konczy sie na pelnym wpisie, wiec po
// sync plik GPX to naglowek i pelne punkty - bez stopki. Sesja konczy sie wylaczeniem zasilania,
// wiec stopke dopisuje repairGpx() przy nastepnym zamontowaniu karty (biezacej sesji nie da sie
// pobrac). (Stopka pisana co sync i nadpisywana nowymi punktami nie dzialala: rozmiar w katalogu
// zostawal stary, a jej miejsce zajmowaly juz punkty.)
void syncFiles() {
  drain(gpxOut);
  if (writeOk && !gpxOut.f.sync()) writeOk = false;
  drain(ubxOut);
  if (writeOk && !ubxOut.f.sync()) writeOk = false;
}

constexpr size_t kFooterLen = sizeof(kFooter) - 1;

// Pliki .gpx bez stopki (sesja przerwana wylaczeniem zasilania): stopka za ostatnim pelnym
// punktem. Wolane raz po zamontowaniu karty, przed nowymi wpisami; biezacy plik pomijany.
bool isActive(const char* n);  // nizej
bool repairPending = false;
void repairGpx() {
  static uint8_t tail[512];  // dluzej niz jeden punkt (~330 B)
  FsFile dir, e, f;
  if (!dir.open(kDir, O_RDONLY)) return;
  char name[48], path[64];
  int fixed = 0, broken = 0;
  while (e.openNext(&dir, O_RDONLY)) {
    const size_t len = e.isDir() ? 0 : e.getName(name, sizeof(name));
    e.close();
    if (len < 5 || strcmp(name + len - 4, ".gpx") != 0 || isActive(name)) continue;
    snprintf(path, sizeof(path), "%s/%s", kDir, name);
    if (!f.open(path, O_RDWR)) continue;
    const uint64_t size = f.fileSize();
    const size_t n = size < sizeof(tail) ? (size_t)size : sizeof(tail);
    const bool read = n > 0 && f.seekSet(size - n) && f.read(tail, n) == (int)n;
    if (read && !(n >= kFooterLen && memcmp(tail + n - kFooterLen, kFooter, kFooterLen) == 0)) {
      // koniec ostatniego pelnego punktu albo naglowka (plik bez punktow)
      int cut = -1;
      for (int i = (int)n - 1; i >= 8 && cut < 0; i--)
        if (tail[i] == '\n' && (memcmp(tail + i - 8, "</trkpt>", 8) == 0 || memcmp(tail + i - 8, "<trkseg>", 8) == 0))
          cut = i + 1;
      const uint64_t end = size - n + (cut < 0 ? 0 : cut);
      if (cut >= 0 && f.truncate(end) && f.seekSet(end) && f.write(kFooter, kFooterLen) == kFooterLen && f.sync())
        fixed++;
      else
        broken++;
    }
    f.close();
  }
  dir.close();
  if (fixed || broken)
    syslog::logf("[GPX] dopisano zakonczenie do %d plikow GPX (sesje konczone wylaczeniem zasilania)%s\n", fixed,
                 broken ? ", czesci nie udalo sie naprawic" : "");
}

// Otwiera (i w razie potrzeby obraca) plik logu. Przy pierwszym zamontowaniu po starcie
// dopisuje naglowek i wszystko, co SysLog zebral od startu.
void openLog() {
  logDirty = false;
  if (!sd.exists(kDir) && !sd.mkdir(kDir)) return;
  if (!logFile.open(kLogPath, O_WRONLY | O_CREAT | O_APPEND)) return;
  if (logFile.fileSize() > kLogMaxBytes) {
    logFile.close();
    sd.remove(kLogOldPath);
    sd.rename(kLogPath, kLogOldPath);
    if (!logFile.open(kLogPath, O_WRONLY | O_CREAT | O_APPEND)) return;
  }
  if (!bootLogged) {
    bootLogged = true;
    char head[120];
    const int n = snprintf(head, sizeof(head), "\n===== start ESP, poprzednie uruchomienie: %s =====\n",
                           syslog::resetReason());
    logFile.write(head, n);
    char* buf = (char*)malloc(syslog::kBufBytes + 1);
    if (buf) {
      logFile.write(buf, syslog::copy(false, buf));
      free(buf);
    }
  } else {
    const char head[] = "----- karta zamontowana ponownie (wpisy z przerwy tylko w /log) -----\n";
    logFile.write(head, sizeof(head) - 1);
  }
  logFile.sync();
}

bool mount() {
  // SdFat::end() wylacza magistrale SPI, wiec przy kazdej probie uruchamiamy ja od nowa.
  sdSpi.begin(kSdSck, kSdMiso, kSdMosi, kSdCs);
  // SHARED_SPI, nie DEDICATED_SPI: w trybie dedykowanym SdFat trzyma rezerwacje magistrali
  // (blokade SPI) miedzy wywolaniami w imieniu zadania zapisu, a strona WWW (inne zadanie)
  // zwalniala ja przy odczycie - FreeRTOS konczyl to bledem (xTaskPriorityDisinherit).
  if (!sd.begin(SdSpiConfig(kSdCs, SHARED_SPI, SD_SCK_MHZ(kSdMhz), &sdSpi))) {
    // karta odpowiada, ale nie ma systemu plikow, ktory SdFat zna (np. tablica partycji GPT)
    const int err = sd.sdErrorCode() ? 1 : 2;
    sd.end();
    if (err != lastMountErr) {
      lastMountErr = err;
      if (err == 1)
        syslog::logf("[GPX] brak karty SD (albo nie odpowiada), sprawdzam co %lu s\n",
                     (unsigned long)(kRetryMountMs / 1000));
      else
        syslog::logf("[GPX] karta SD jest, ale bez FAT32/exFAT na tablicy MBR (GPT nieobslugiwane) - "
                     "sformatuj ja, sprawdzam co %lu s\n",
                     (unsigned long)(kRetryMountMs / 1000));
    }
    return false;
  }
  lastMountErr = -1;
  writeOk = true;
  baseName[0] = '\0';
  mountGen++;
  cardSizeMB = (uint32_t)(sd.card()->sectorCount() / 2048);
  lastSyncMs = millis();
  repairPending = true;
  // Przed st = Waiting: wpisy logu z tej chwili sa juz w kopii z pamieci RTC, a nowe
  // (od st = Waiting) przyjda kolejka - bez dubli.
  openLog();
  st = State::Waiting;
  const uint8_t fat = sd.fatType();
  syslog::logf("[GPX] karta SD OK (%s, %s, %lu MB), wolny RAM %u B - czekam na fix GPS\n",
               sd.card()->type() == SD_CARD_TYPE_SDHC ? "SDHC/SDXC" : "SD",
               fat == FAT_TYPE_EXFAT ? "exFAT" : (fat == 32 ? "FAT32" : "FAT16"),
               (unsigned long)cardSizeMB, (unsigned)ESP.getFreeHeap());
  return true;
}

void closeFiles() {
  gpxOut.f.close();
  ubxOut.f.close();
  gpxOut.len = ubxOut.len = 0;
}

// Blad karty: zamykamy pliki i odmontowujemy. Zadanie zapisu co kRetryMountMs probuje
// zamontowac karte od nowa - po udanej probie nastepny fix zaczyna nowa pare plikow.
void fail(const char* why) {
  st = State::Error;  // najpierw: zadanie GPS przestaje dokladac do kolejki
  syslog::logf("[GPX] %s - zapis zatrzymany, ponowna proba co %lu s\n", why,
               (unsigned long)(kRetryMountMs / 1000));
  closeFiles();
  logFile.close();
  logDirty = false;
  sd.end();
  baseName[0] = '\0';
  lastMountMs = millis();
}

// Nazwa z czasu pierwszego fixa (UTC), np. "2026-11-13_14-05-32"; przy kolizji "_2", "_3"...
bool openNewFiles(const rc::GpsFix& f) {
  syslog::logf("[GPX] pierwszy fix - tworze pliki\n");
  fileTime = f;
  if (!sd.exists(kDir) && !sd.mkdir(kDir)) return false;
  char stamp[32], gpxPath[64], ubxPath[64];  // zapas: kompilator liczy pola uint8_t jako 0..255
  snprintf(stamp, sizeof(stamp), "%04u-%02u-%02u_%02u-%02u-%02u", f.year, f.month, f.day, f.hour, f.minute,
           f.second);
  for (int n = 1; n <= 99; n++) {
    if (n == 1)
      snprintf(baseName, sizeof(baseName), "%s", stamp);
    else
      snprintf(baseName, sizeof(baseName), "%s_%d", stamp, n);
    snprintf(gpxPath, sizeof(gpxPath), "%s/%s.gpx", kDir, baseName);
    snprintf(ubxPath, sizeof(ubxPath), "%s/%s.ubx", kDir, baseName);
    if (sd.exists(gpxPath) || sd.exists(ubxPath)) continue;

    gpxOut.len = ubxOut.len = 0;
    const bool opened = gpxOut.f.open(gpxPath, O_WRONLY | O_CREAT | O_EXCL) &&
                        ubxOut.f.open(ubxPath, O_WRONLY | O_CREAT | O_EXCL);
    if (opened) {
      put(gpxOut,
          "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
          "<gpx version=\"1.1\" creator=\"RaceChrono CYD display\"\n"
          " xmlns=\"http://www.topografix.com/GPX/1/1\"\n"
          " xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v2\">\n"
          "<trk><name>");
      put(gpxOut, baseName);
      put(gpxOut, "</name><trkseg>\n");
      syncFiles();
    }
    if (!opened || !writeOk) {
      // np. pelna karta - bez zostawiania pustych plikow po kazdej nieudanej probie
      closeFiles();
      sd.remove(gpxPath);
      sd.remove(ubxPath);
      baseName[0] = '\0';
      return false;
    }
    syslog::logf("[GPX] zapis do %s i %s\n", gpxPath, ubxPath);
    lastSyncMs = millis();
    firstSyncLogged = false;
    return true;
  }
  baseName[0] = '\0';
  return false;
}

void enqueue(ItemType type, const void* data, size_t len) {
  uint8_t item[1 + 160];
  if (!queue || len > sizeof(item) - 1) return;
  item[0] = type;
  memcpy(item + 1, data, len);
  if (xRingbufferSend(queue, item, 1 + len, 0) != pdTRUE) dropped = dropped + 1;
}

void writeFix(const rc::GpsFix& f) {
  if (st == State::Waiting) {
    if (!openNewFiles(f)) {
      fail("nie moge utworzyc plikow");
      return;
    }
    st = State::Logging;
  }
  if (st != State::Logging) return;
  fileTime = f;

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
  put(gpxOut, buf);
}

void writeQueued() {
  if (repairPending) {  // przed pierwszym fixem, wiec nowy plik jeszcze nie istnieje
    repairPending = false;
    repairGpx();
  }
  size_t size;
  while (uint8_t* item = (uint8_t*)xRingbufferReceive(queue, &size, 0)) {
    if (item[0] == ItemFix && size == 1 + sizeof(rc::GpsFix)) {
      rc::GpsFix f;
      memcpy(&f, item + 1, sizeof(f));
      writeFix(f);
    } else if (item[0] == ItemUbx && st == State::Logging) {
      put(ubxOut, item + 1, size - 1);
    } else if (item[0] == ItemLog && logFile.isOpen() && writeOk) {
      if (logFile.write(item + 1, size - 1) != size - 1) writeOk = false;
      logDirty = true;
    }
    vRingbufferReturnItem(queue, item);
  }

  if (millis() - lastSyncMs >= kSyncEveryMs) {
    lastSyncMs = millis();
    if (st == State::Logging) syncFiles();
    if (logDirty && writeOk) {
      logDirty = false;
      if (!logFile.sync()) writeOk = false;
    }
    if (dropped) {
      syslog::logf("[GPX] kolejka pelna, pominieto %lu wpisow\n", (unsigned long)dropped);
      dropped = 0;
    }
    if (st == State::Logging && !firstSyncLogged && writeOk) {
      firstSyncLogged = true;
      syslog::logf("[GPX] pierwszy zapis na karte OK (stos zadania sd: wolne %u B, wolny RAM %u B)\n",
                   (unsigned)uxTaskGetStackHighWaterMark(nullptr), (unsigned)ESP.getFreeHeap());
    }
  }
  if (!writeOk) fail("blad zapisu na karte");
}

bool pausing() {
  const uint32_t until = pauseUntil;
  const bool p = until != 0 && (int32_t)(until - millis()) > 0;
  if (p != pausedNow) {
    pausedNow = p;
    if (p) {
      pauseSkipped = 0;
      syslog::logf("[TEST] zapis na SD wstrzymany\n");
    } else {
      pauseUntil = 0;
      syslog::logf("[TEST] zapis na SD wznowiony (pominieto %lu wpisow)\n", (unsigned long)pauseSkipped);
    }
  }
  return p;
}

void sdTask(void*) {
  for (;;) {
    if (stopReq) {
      taskStopped = true;
      vTaskDelete(nullptr);
    }
    if (pausing()) {
      // bez karty: wpisy z kolejki przepadaja, pliki zostaja otwarte
      size_t size;
      while (void* item = xRingbufferReceive(queue, &size, 0)) {
        vRingbufferReturnItem(queue, item);
        pauseSkipped++;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    xSemaphoreTake(mtx, portMAX_DELAY);
    if (st == State::Error) {
      // Wpisy sprzed bledu sa juz nieaktualne (nowe w tym stanie nie trafiaja do kolejki).
      size_t size;
      while (void* item = xRingbufferReceive(queue, &size, 0)) vRingbufferReturnItem(queue, item);
      if (millis() - lastMountMs >= kRetryMountMs) {
        lastMountMs = millis();
        mount();
      }
    } else {
      writeQueued();
    }
    xSemaphoreGive(mtx);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

bool mounted() { return st == State::Waiting || st == State::Logging; }

bool lock() { return mtx && xSemaphoreTake(mtx, pdMS_TO_TICKS(3000)) == pdTRUE; }
void unlock() { xSemaphoreGive(mtx); }

bool validName(const char* n) {
  const size_t len = strlen(n);
  if (len == 0 || len >= sizeof(FileInfo::name) || n[0] == '.') return false;
  for (size_t i = 0; i < len; i++) {
    const char c = n[i];
    if (!isalnum((unsigned char)c) && c != '-' && c != '_' && c != '.') return false;
  }
  return true;
}

// Plik biezacej pary zapisu ("<baseName>.gpx" / ".ubx") - wolac pod blokada.
bool isSession(const char* n) {
  if (st != State::Logging || !baseName[0]) return false;
  const size_t len = strlen(baseName);
  return strncmp(n, baseName, len) == 0 && n[len] == '.';
}

// Plik, do ktorego teraz trafiaja dane: biezaca para albo log (dopisywany na biezaco).
bool isActive(const char* n) { return isSession(n) || (logFile.isOpen() && strcmp(n, kLogName) == 0); }

// SysLog -> kolejka -> plik logu (zadanie zapisu). Z dowolnego zadania; bez kopii na stosie,
// bo loguja tez callbacki Bluetooth z malym stosem.
void logLine(const char* s, size_t len) {
  if (!queue || (st != State::Waiting && st != State::Logging)) return;
  if (len > 250) len = 250;
  void* p = nullptr;
  if (xRingbufferSendAcquire(queue, &p, 1 + len, 0) != pdTRUE || !p) {
    dropped = dropped + 1;
    return;
  }
  uint8_t* b = (uint8_t*)p;
  b[0] = ItemLog;
  memcpy(b + 1, s, len);
  xRingbufferSendComplete(queue, p);
}

}  // namespace

void begin(bool enabled) {
  if (!enabled) {
    st = State::Disabled;
    return;
  }
  FsDateTime::setCallback(dateTime);
  queue = xRingbufferCreate(kQueueBytes, RINGBUF_TYPE_NOSPLIT);
  if (!queue) {
    syslog::logf("[GPX] brak RAM na kolejke zapisu\n");
    st = State::Disabled;
    return;
  }
  mtx = xSemaphoreCreateMutex();
  if (!mtx) {
    syslog::logf("[GPX] brak RAM na blokade karty\n");
    st = State::Disabled;
    return;
  }
  if (!mount()) st = State::Error;
  lastMountMs = millis();
  syslog::setSink(logLine);
  // rdzen 0 (tam dziala tez BLE), niski priorytet; ekran i GPS sa na rdzeniu 1
  if (xTaskCreatePinnedToCore(sdTask, "sd", 6144, nullptr, 1, nullptr, 0) != pdPASS) {
    syslog::logf("[GPX] brak RAM na zadanie zapisu\n");
    st = State::Disabled;
    sd.end();
  }
}

void addFix(const rc::GpsFix& f) {
  if (st != State::Waiting && st != State::Logging) return;
  // tylko sensowne dane: fix i poprawna data/czas (od pierwszego takiego punktu powstaje plik)
  if (f.fixQuality == 0 || f.year < 2020 || f.month < 1 || f.month > 12 || f.day < 1 || f.day > 31 ||
      f.hour > 23 || f.minute > 59 || f.second > 59)
    return;
  enqueue(ItemFix, &f, sizeof(f));
}

// Ramka do 1 kB - prosto do kolejki, bez kopii na stosie zadania GPS.
void ubxFrame(const uint8_t* frame, size_t len) {
  if (!queue || st != State::Logging) return;
  void* p = nullptr;
  if (xRingbufferSendAcquire(queue, &p, 1 + len, 0) != pdTRUE || !p) {
    dropped = dropped + 1;
    return;
  }
  uint8_t* b = (uint8_t*)p;
  b[0] = ItemUbx;
  memcpy(b + 1, frame, len);
  xRingbufferSendComplete(queue, p);
}

State state() { return st; }
const char* fileName() { return st == State::Logging ? baseName : ""; }

bool listOpen() {
  if (!lock()) return false;
  const bool ok = mounted();
  webDir.close();
  webDirOpen = ok && webDir.open(kDir, O_RDONLY);  // brak /gpx = pusta lista
  webDirGen = mountGen;
  unlock();
  return ok;
}

bool listNext(FileInfo& out) {
  if (!webDirOpen || !lock()) return false;
  bool found = false;
  if (mounted() && webDirGen == mountGen) {
    FsFile f;
    while (!found && f.openNext(&webDir, O_RDONLY)) {
      if (!f.isDir() && f.getName(out.name, sizeof(out.name)) && validName(out.name)) {
        const uint64_t size = f.fileSize();
        out.size = size > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)size;
        uint16_t d = 0, t = 0;
        out.modified[0] = '\0';
        if (f.getModifyDateTime(&d, &t) && FS_YEAR(d) >= 2020)  // przed fixem GPS daty nie ma
          snprintf(out.modified, sizeof(out.modified), "%04u-%02u-%02u %02u:%02u", FS_YEAR(d), FS_MONTH(d),
                   FS_DAY(d), FS_HOUR(t), FS_MINUTE(t));
        out.active = isActive(out.name);
        out.recording = isSession(out.name);
        found = true;
      }
      f.close();
    }
  }
  if (!found) {
    webDir.close();
    webDirOpen = false;
  }
  unlock();
  return found;
}

void listClose() {
  if (!lock()) return;
  webDir.close();
  webDirOpen = false;
  unlock();
}

bool recording(const char* name) {
  if (!lock()) return false;
  const bool r = isSession(name);
  unlock();
  return r;
}

bool readOpen(const char* name, uint32_t& size) {
  if (!validName(name) || !lock()) return false;
  char path[64];
  snprintf(path, sizeof(path), "%s/%s", kDir, name);
  webFile.close();
  // Biezaca sesja nie: pobieranie pliku, do ktorego zadanie zapisu wlasnie dopisuje, zawiesilo
  // wyswietlacz (watchdog). Pliki poprzednich sesji sa zamkniete.
  bool ok = mounted() && !isSession(name) && webFile.open(path, O_RDONLY) && !webFile.isDir();
  if (ok) {
    const uint64_t s = webFile.fileSize();
    size = s > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)s;
    webFileGen = mountGen;
  } else {
    webFile.close();
  }
  unlock();
  return ok;
}

int readChunk(uint8_t* buf, size_t len) {
  if (!lock()) return kReadBusy;
  int n = -1;
  if (mounted() && webFileGen == mountGen && webFile.isOpen()) n = webFile.read(buf, len);
  unlock();
  return n;
}

void readClose() {
  if (!lock()) return;
  webFile.close();
  unlock();
}

const char* removeFile(const char* name) {
  if (!validName(name)) return "niepoprawna nazwa pliku";
  if (!lock()) return "karta zajeta, sprobuj ponownie";
  const char* err = nullptr;
  char path[64];
  snprintf(path, sizeof(path), "%s/%s", kDir, name);
  if (!mounted())
    err = "brak karty";
  else if (isActive(name))
    err = "to biezacy zapis - nie mozna usunac";
  else if (!sd.remove(path))
    err = "nie udalo sie usunac (brak pliku?)";
  if (!err) syslog::logf("[GPX] usunieto %s (strona WWW)\n", path);
  unlock();
  return err;
}

int removeInactive() {
  if (!listOpen()) return -1;
  int removed = 0;
  FileInfo fi;
  while (listNext(fi)) {
    // usuwanie wpisu za biezaca pozycja katalogu nie przesuwa kolejnych wpisow
    if (!fi.active && removeFile(fi.name) == nullptr) removed++;
  }
  return removed;
}

uint32_t cardMB() { return mounted() ? cardSizeMB : 0; }

void shutdown() {
  if (!queue) {  // zapis nie wystartowal (tryb awaryjny, brak RAM)
    st = State::Disabled;
    return;
  }
  syslog::setSink(nullptr);
  stopReq = true;
  for (int i = 0; i < 100 && !taskStopped; i++) delay(10);
  if (mtx) xSemaphoreTake(mtx, pdMS_TO_TICKS(1000));
  if (taskStopped) {
    if (st == State::Logging) {  // punkty z buforow i stopka - plik konczy sie poprawnie
      syncFiles();
      gpxOut.f.write(kFooter, kFooterLen);
      gpxOut.f.sync();
    }
    if (logFile.isOpen()) logFile.sync();
    closeFiles();
    logFile.close();
    webFile.close();
    webDir.close();
    webDirOpen = false;
    if (st == State::Waiting || st == State::Logging) sd.end();
  }
  st = State::Disabled;
  if (mtx) xSemaphoreGive(mtx);
  if (taskStopped) {
    delay(20);  // inne zadanie moglo byc w trakcie logLine()/addFix() sprzed zmiany stanu
    vRingbufferDelete(queue);
    queue = nullptr;
  }
  syslog::logf("[GPX] zapis zatrzymany przed wgrywaniem programu%s\n",
               taskStopped ? "" : " (zadanie zapisu nie odpowiedzialo - pliki niedomkniete)");
}

void pauseFor(uint32_t ms) { pauseUntil = (millis() + ms) | 1; }

uint32_t pauseLeftMs() {
  const uint32_t until = pauseUntil;
  const int32_t left = until ? (int32_t)(until - millis()) : 0;
  return left > 0 ? (uint32_t)left : 0;
}

}  // namespace gpx

#else  // bez GPX_LOG: puste funkcje

namespace gpx {
void begin(bool) {}
void addFix(const rc::GpsFix&) {}
void ubxFrame(const uint8_t*, size_t) {}
State state() { return State::Off; }
const char* fileName() { return ""; }
bool listOpen() { return false; }
bool listNext(FileInfo&) { return false; }
void listClose() {}
bool recording(const char*) { return false; }
bool readOpen(const char*, uint32_t&) { return false; }
int readChunk(uint8_t*, size_t) { return -1; }
void readClose() {}
const char* removeFile(const char*) { return "zapis na SD wylaczony w programie"; }
void shutdown() {}
int removeInactive() { return -1; }
uint32_t cardMB() { return 0; }
void pauseFor(uint32_t) {}
uint32_t pauseLeftMs() { return 0; }
}  // namespace gpx

#endif

namespace gpx {
const char* stateName() {
  switch (state()) {
    case State::Off: return "wylaczony w programie (bez -DGPX_LOG)";
    case State::Error: return "BLAD - brak karty, zly format albo blad zapisu (proba co 5 s)";
    case State::Disabled: return "WYLACZONY - tryb awaryjny, wgrywanie programu albo brak RAM (szczegoly w logu)";
    case State::Waiting: return "karta OK, czeka na fix GPS";
    case State::Logging: return "zapisuje";
  }
  return "?";
}
}  // namespace gpx
