#include "SysLog.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <stdarg.h>

namespace syslog {
namespace {

constexpr uint32_t kMagic = 0x52434C47;  // "RCLG"
constexpr uint32_t kStableMs = 20000;    // dluzej = uruchomienie udane (awaria pozniej to nie petla)
constexpr uint8_t kSafeAfter = 2;        // tyle awarii z rzedu tuz po starcie -> tryb awaryjny

struct Ring {
  uint32_t head;  // ile bajtow zapisano od poczatku (pozycja = head % kBufBytes)
  char data[kBufBytes];
};

// Pamiec RTC bez zerowania przy starcie. Po wlaczeniu zasilania zawiera smieci - stad magic.
struct Rtc {
  uint32_t magic;
  uint8_t cur;      // ktory bufor nalezy do biezacego uruchomienia
  uint8_t stable;   // biezace uruchomienie przetrwalo kStableMs
  uint8_t crashes;  // kolejne awarie tuz po starcie
  uint8_t prevValid;
  Ring ring[2];
};
RTC_NOINIT_ATTR Rtc rtc;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool safe = false;
void (*volatile sinkFn)(const char*, size_t) = nullptr;
esp_reset_reason_t reason = ESP_RST_UNKNOWN;

bool isCrash(esp_reset_reason_t r) {
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
         r == ESP_RST_BROWNOUT;
}

void append(const char* s, size_t len) {
  Ring& r = rtc.ring[rtc.cur];
  portENTER_CRITICAL(&mux);  // logf wolaja rozne zadania na obu rdzeniach
  for (size_t i = 0; i < len; i++) {
    r.data[r.head % kBufBytes] = s[i];
    r.head++;
  }
  portEXIT_CRITICAL(&mux);
}

}  // namespace

void begin() {
  reason = esp_reset_reason();
  const bool valid = rtc.magic == kMagic && rtc.cur < 2;
  if (!valid || !isCrash(reason))
    rtc.crashes = 0;  // zasilanie, przycisk RST, restart programowy (wgranie, ustawienia)
  else if (rtc.stable)
    rtc.crashes = 1;  // awaria po dluzszej pracy - nie petla
  else if (rtc.crashes < 99)
    rtc.crashes++;
  safe = rtc.crashes >= kSafeAfter;

  rtc.prevValid = valid;
  rtc.cur = valid ? (uint8_t)(rtc.cur ^ 1) : 0;
  rtc.ring[rtc.cur].head = 0;
  if (!valid) rtc.ring[1].head = 0;
  rtc.stable = 0;
  rtc.magic = kMagic;

  logf("[SYS] start, poprzednie uruchomienie: %s\n", resetReason());
  if (safe)
    logf("[SYS] TRYB AWARYJNY: %u awarie z rzedu tuz po starcie - zapis na SD wylaczony "
         "(wylacz i wlacz zasilanie, zeby sprobowac ponownie)\n",
         (unsigned)rtc.crashes);
  else if (rtc.crashes)
    logf("[SYS] awaria nr %u z rzedu; po %u tryb awaryjny bez SD\n", (unsigned)rtc.crashes,
         (unsigned)kSafeAfter);
}

void loop() {
  if (!rtc.stable && millis() >= kStableMs) {
    rtc.stable = 1;
    logf("[SYS] dziala %lu s - uruchomienie stabilne, wolny RAM %u B (minimum %u B)\n",
         (unsigned long)(kStableMs / 1000), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  }
}

void logf(const char* fmt, ...) {
  char line[192];
  const uint32_t ms = millis();
  int n = snprintf(line, sizeof(line), "%5lu.%03lu ", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000));
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line + n, sizeof(line) - n, fmt, ap);
  va_end(ap);
  const size_t len = strlen(line);
  Serial.write((const uint8_t*)line + n, len - n);  // na Serial bez czasu (jak dotad)
  append(line, len);
  if (auto sink = sinkFn) sink(line, len);
}

void setSink(void (*sink)(const char*, size_t)) { sinkFn = sink; }

bool safeMode() { return safe; }
uint8_t crashCount() { return rtc.crashes; }

const char* resetReason() {
  switch (reason) {
    case ESP_RST_POWERON: return "wlaczenie zasilania";
    case ESP_RST_EXT: return "przycisk RST";
    case ESP_RST_SW: return "restart programowy (wgranie programu / ustawienia)";
    case ESP_RST_PANIC: return "BLAD PROGRAMU (panic / wyjatek)";
    case ESP_RST_INT_WDT: return "BLAD: watchdog przerwan";
    case ESP_RST_TASK_WDT: return "BLAD: watchdog zadan (cos sie zawiesilo)";
    case ESP_RST_WDT: return "BLAD: watchdog";
    case ESP_RST_BROWNOUT: return "SPADEK NAPIECIA (brownout) - za slabe zasilanie";
    case ESP_RST_DEEPSLEEP: return "wybudzenie";
    default: return "nieznana";
  }
}

size_t copy(bool previous, char* out) {
  if (previous && !rtc.prevValid) {
    out[0] = '\0';
    return 0;
  }
  const Ring& r = rtc.ring[previous ? rtc.cur ^ 1 : rtc.cur];
  portENTER_CRITICAL(&mux);
  const uint32_t head = r.head;
  const size_t len = head < kBufBytes ? head : kBufBytes;
  const uint32_t start = head - len;
  for (size_t i = 0; i < len; i++) out[i] = r.data[(start + i) % kBufBytes];
  portEXIT_CRITICAL(&mux);
  out[len] = '\0';
  return len;
}

}  // namespace syslog
