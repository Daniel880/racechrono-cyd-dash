#include "Dashboard.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "LGFX_CYD.h"
#include "fonts_saira.h"  // wygenerowane: tools/make_fonts.py

// Uklad 240x320 (pionowo). Delty i czasy duze, rozmiar dopasowany do panelu:
//  +--------------------------+
//  | o GPS SD   145 km/h      |  0..23    status + predkosc
//  +--------------------------+
//  |            +3            |  24..81   delta predkosci [km/h]
//  | [=======|             ]  |           + pasek
//  +--------------------------+
//  |          -0.23           |  83..140  delta czasu [s]
//  | [=======|             ]  |           + pasek
//  +--------------------------+
//  | CURRENT #12     1:23.45  |  142..199 etykieta mala, czas duzy
//  +--------------------------+
//  | LAST #11        1:24.10  |  201..258
//  +--------------------------+
//  | BEST #7         1:22.98  |  260..317 albo REF #4 (zolty), gdy w RC
//  +--------------------------+           wybrano inne okrazenie porownawcze

namespace dash {
namespace {

constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

constexpr uint16_t COL_BG = rgb(0, 0, 0);
constexpr uint16_t COL_TEXT = rgb(255, 255, 255);
constexpr uint16_t COL_LABEL = rgb(140, 140, 140);
constexpr uint16_t COL_DIM = rgb(70, 70, 70);
constexpr uint16_t COL_LINE = rgb(50, 50, 50);
constexpr uint16_t COL_GREEN = rgb(0, 220, 80);
constexpr uint16_t COL_RED = rgb(255, 50, 50);
constexpr uint16_t COL_YELLOW = rgb(255, 210, 0);
constexpr uint16_t COL_PURPLE = rgb(200, 90, 255);

constexpr int32_t kDeltaBarRangeMs = 2000;     // pelny pasek = +-2 s
constexpr int32_t kDeltaBarRangeKmhX10 = 200;  // pelny pasek = +-20 km/h
constexpr uint32_t kLastHighlightMs = 10000;    // podswietlenie LAST po przejechaniu mety

// Kazdy panel jest rysowany zawsze od zera, wiec wszystkie duze panele (240x58) dziela jeden
// bufor 16-bitowy (gladkie krawedzie czcionki, ~28 KB RAM zamiast ~140 KB przy osobnych).
// Pasek stanu ma wlasny maly bufor 8-bitowy.
constexpr int16_t kBigH = 58;
LGFX_Sprite statusSpr;
LGFX_Sprite bigSpr;

struct Panel {
  int16_t x, y, w, h;
  LGFX_Sprite* spr;
  char key[64];  // co jest aktualnie narysowane
};

LGFX* lcd = nullptr;
Panel pStatus{0, 0, 240, 24, &statusSpr};
Panel pDeltaSpeed{0, 24, 240, kBigH, &bigSpr};
Panel pDelta{0, 83, 240, kBigH, &bigSpr};
Panel pCurrent{0, 142, 240, kBigH, &bigSpr};
Panel pLast{0, 201, 240, kBigH, &bigSpr};
Panel pBest{0, 260, 240, kBigH, &bigSpr};  // 260..317, 2 dolne wiersze ekranu zostaja czarne

void createSpr(LGFX_Sprite& s, int16_t w, int16_t h, uint8_t depth) {
  s.setColorDepth(depth);
  if (s.createSprite(w, h)) return;
  Serial.printf("[UI] brak RAM na bufor %dx%d w %u bit\n", w, h, depth);
  if (depth > 8) createSpr(s, w, h, 8);  // awaryjnie 8 bit (mniej plynne wygladzanie)
}

// Czcionki Saira Bold (VLW, wygladzane) w dokladnych rozmiarach - bez skalowania.
lgfx::PointerWrapper fontDeltaData(kFontDelta, sizeof(kFontDelta));
lgfx::PointerWrapper fontTimeData(kFontTime, sizeof(kFontTime));
lgfx::VLWfont fontDelta;
lgfx::VLWfont fontTime;

// true = trzeba przerysowac (klucz sie zmienil)
bool changed(Panel& p, const char* key) {
  if (strncmp(p.key, key, sizeof(p.key)) == 0) return false;
  strncpy(p.key, key, sizeof(p.key) - 1);
  p.key[sizeof(p.key) - 1] = '\0';
  return true;
}

void push(Panel& p) { p.spr->pushSprite(lcd, p.x, p.y); }

void formatLapTime(char* out, size_t n, bool has, int32_t ms) {
  if (!has || ms < 0) {
    snprintf(out, n, "-:--.--");
    return;
  }
  unsigned long m = ms / 60000;
  unsigned long s = (ms / 1000) % 60;
  unsigned long cs = (ms / 10) % 100;
  snprintf(out, n, "%lu:%02lu.%02lu", m, s, cs);
}

// Duza czcionka w naturalnym rozmiarze; zmniejszana tylko, gdy tekst nie miesci sie w maxW
// (np. delta czasu powyzej 99 s).
void setBigFont(LGFX_Sprite& s, const lgfx::IFont& font, const char* text, int maxW) {
  s.setFont(&font);
  s.setTextSize(1);
  int w = s.textWidth(text);
  if (w > maxW) s.setTextSize((float)maxW / w);
}

int32_t clampI(int32_t v, int32_t lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

void drawStatus(const DashData& d) {
  uint16_t dot = !d.connected ? COL_DIM : (!d.configured ? COL_YELLOW : COL_GREEN);

  // srodek paska: predkosc, a gdy RC jeszcze nie gotowe - stan polaczenia
  char mid[16];
  uint16_t midCol = COL_TEXT;
  if (!d.connected) {
    snprintf(mid, sizeof(mid), "Czekam na RC");
    midCol = COL_LABEL;
  } else if (!d.configured) {
    snprintf(mid, sizeof(mid), "Konfiguracja");
    midCol = COL_LABEL;
  } else if (d.hasSpeed) {
    snprintf(mid, sizeof(mid), "%ld km/h", (long)((d.speedKmhX10 + 5) / 10));
  } else {
    snprintf(mid, sizeof(mid), "-- km/h");
    midCol = COL_DIM;
  }

  // Wskazniki: GPS (szary brak modulu/danych, zolty brak fixa, zielony fix),
  //             SD (szary wylaczone, czerwony blad, zolty czeka na fix, zielony zapis)
  uint16_t gpsCol = d.gpsState == 3 ? COL_GREEN : (d.gpsState == 2 ? COL_YELLOW : COL_DIM);
  uint16_t sdCol = d.sdState == 2 ? COL_GREEN : (d.sdState == 1 ? COL_YELLOW : (d.sdState < 0 ? COL_RED : COL_DIM));
  bool showGps = d.gpsState != 0;
  bool showSd = d.sdState != 0;

  char key[64];
  snprintf(key, sizeof(key), "%u|%s|%u|%u|%u|%d%d", (unsigned)dot, mid, (unsigned)midCol, (unsigned)gpsCol,
           (unsigned)sdCol, showGps, showSd);
  if (!changed(pStatus, key)) return;

  auto& s = *pStatus.spr;
  s.fillScreen(COL_BG);
  s.fillCircle(8, 12, 5, dot);
  s.setFont(&fonts::Font2);
  s.setTextDatum(middle_left);
  if (showGps) {
    s.setTextColor(gpsCol);
    s.drawString("GPS", 20, 12);
  }
  if (showSd) {
    s.setTextColor(sdCol);
    s.drawString("SD", 50, 12);
  }
  s.setTextColor(midCol);
  s.setTextDatum(middle_center);
  s.drawString(mid, 126, 12);
  push(pStatus);
}

// Panel delty: duze cyfry na srodku, pod nimi pasek (zysk w lewo na zielono, strata w prawo na czerwono).
void drawDeltaPanel(Panel& p, const char* text, uint16_t color, int barLen) {
  char key[64];
  snprintf(key, sizeof(key), "%s|%u|%d", text, (unsigned)color, barLen);
  if (!changed(p, key)) return;

  auto& s = *p.spr;
  s.fillScreen(COL_BG);
  const int barH = 6;
  const int barY = p.h - barH - 2;
  const int baseline = barY - 4;
  setBigFont(s, fontDelta, text, p.w - 8);
  s.setTextColor(color, COL_BG);
  s.setTextDatum(baseline_center);
  s.drawString(text, p.w / 2, baseline);
  s.setTextSize(1);

  const int cx = p.w / 2;
  s.fillRect(4, barY, p.w - 8, barH, COL_LINE);
  if (barLen < 0) s.fillRect(cx + barLen, barY, -barLen, barH, COL_GREEN);
  if (barLen > 0) s.fillRect(cx, barY, barLen, barH, COL_RED);
  s.drawFastVLine(cx, barY - 2, barH + 4, COL_TEXT);
  push(p);
}

int deltaHalfBar(const Panel& p) { return p.w / 2 - 4; }

// Delta predkosci w pelnych km/h: + = szybciej (zielony).
void drawSpeedDelta(const DashData& d) {
  char text[12];
  uint16_t color = COL_DIM;
  int barLen = 0;
  if (!d.hasDeltaSpeed) {
    snprintf(text, sizeof(text), "--");
  } else {
    int32_t v = clampI(d.deltaSpeedKmhX10, 9999);
    long kmh = (long)((v + (v >= 0 ? 5 : -5)) / 10);  // zaokraglenie do 1 km/h
    if (kmh == 0)
      snprintf(text, sizeof(text), "0");
    else
      snprintf(text, sizeof(text), "%+ld", kmh);
    color = kmh > 0 ? COL_GREEN : (kmh < 0 ? COL_RED : COL_TEXT);
    barLen = (int)(-clampI(v, kDeltaBarRangeKmhX10) * deltaHalfBar(pDeltaSpeed) / kDeltaBarRangeKmhX10);
  }
  drawDeltaPanel(pDeltaSpeed, text, color, barLen);
}

// Delta czasu w sekundach: - = szybciej (zielony).
void drawTimeDelta(const DashData& d) {
  char text[16];
  uint16_t color = COL_DIM;
  int barLen = 0;
  if (!d.hasDelta) {
    snprintf(text, sizeof(text), "--.--");
  } else {
    int32_t ms = clampI(d.deltaMs, 99990);
    snprintf(text, sizeof(text), "%+.2f", ms / 1000.0f);
    color = ms < 0 ? COL_GREEN : (ms > 0 ? COL_RED : COL_TEXT);
    barLen = (int)(clampI(ms, kDeltaBarRangeMs) * deltaHalfBar(pDelta) / kDeltaBarRangeMs);
  }
  drawDeltaPanel(pDelta, text, color, barLen);
}

// Etykieta (+ opcjonalnie numer okrazenia, np. "#5") mala w lewym gornym rogu,
// czas pod nia jak najwiekszy, na cala szerokosc panelu.
// bg != COL_BG = podswietlenie calego panelu (tlo w kolorze, wszystkie napisy czarne).
void drawTimeRow(Panel& p, const char* label, bool has, int32_t ms, uint16_t valueColor, const char* sub = nullptr,
                 uint16_t bg = COL_BG) {
  char text[16];
  formatLapTime(text, sizeof(text), has, ms);
  if (!has) valueColor = COL_DIM;
  const bool lit = bg != COL_BG;
  if (lit) valueColor = COL_BG;

  char key[64];
  snprintf(key, sizeof(key), "%s|%s|%u|%s|%u", label, text, (unsigned)valueColor, sub ? sub : "", (unsigned)bg);
  if (!changed(p, key)) return;

  auto& s = *p.spr;
  s.fillScreen(bg);

  // Najpierw duzy czas: z kolorem tla zamalowuje cala wysokosc linii czcionki, wiec etykiety
  // narysowane wczesniej by znikaly. Czcionka dobrana pod "00:00.00" - po 9:59.99 cyfry nie maleja.
  setBigFont(s, fontTime, text, p.w - 8);
  s.setTextColor(valueColor, bg);
  s.setTextDatum(baseline_right);
  s.drawString(text, p.w - 4, p.h - 4);
  s.setTextSize(1);

  s.setFont(&fonts::Font2);
  s.setTextDatum(top_left);
  s.setTextColor(lit ? COL_BG : COL_LABEL);
  s.drawString(label, 4, 1);
  if (sub) {
    s.setTextColor(lit ? COL_BG : COL_TEXT);
    s.drawString(sub, 8 + s.textWidth(label), 1);
  }
  push(p);
}

}  // namespace

void begin(LGFX& display) {
  lcd = &display;
  lcd->fillScreen(COL_BG);
  fontDelta.loadFont(&fontDeltaData);
  fontTime.loadFont(&fontTimeData);
  createSpr(statusSpr, 240, 24, 8);
  createSpr(bigSpr, 240, kBigH, 16);
  Serial.printf("[UI] wolny RAM %u B, najwiekszy blok %u B\n", (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap());
  // linie oddzielajace panele (1 px przerwy miedzy nimi)
  for (int y : {82, 141, 200, 259}) lcd->drawFastHLine(0, y, lcd->width(), COL_LINE);
}

void update(const DashData& d) {
  drawStatus(d);
  drawSpeedDelta(d);
  drawTimeDelta(d);
  char curLap[12] = "#-", bestLap[12] = "#-", lastLap[12] = "#-";
  if (d.hasLapNumber) snprintf(curLap, sizeof(curLap), "#%ld", (long)d.lapNumber);
  if (d.hasLastLapNumber) snprintf(lastLap, sizeof(lastLap), "#%ld", (long)d.lastLapNumber);
  if (d.hasBestLapNumber) snprintf(bestLap, sizeof(bestLap), "#%ld", (long)d.bestLapNumber);

  drawTimeRow(pCurrent, "CURRENT", d.hasCurrent, d.currentMs, COL_TEXT, curLap);
  bool lastIsBest = d.hasLast && d.hasBest && d.lastMs == d.bestMs;

  // Po przejechaniu mety (wzrost numeru okrazenia) LAST podswietlony przez 10 s:
  // fiolet = nowe BEST, zielony = lepsze od okrazenia porownawczego, czerwony = gorsze.
  // Kolor liczony na biezaco - wartosci z RC przy mecie przychodza po kolei w ciagu ~100 ms.
  static int32_t prevLap = -1;
  static uint32_t highlightStart = 0;
  static bool highlight = false;
  if (d.hasLapNumber) {
    if (prevLap >= 0 && d.lapNumber > prevLap) {
      highlight = true;
      highlightStart = millis();
    }
    prevLap = d.lapNumber;
  }
  if (highlight && millis() - highlightStart >= kLastHighlightMs) highlight = false;

  uint16_t lastBg = COL_BG;
  if (highlight && d.hasLast) {
    int32_t refMs = d.hasRef ? d.refMs : d.bestMs;
    bool hasRefMs = d.hasRef || d.hasBest;
    if (lastIsBest)
      lastBg = COL_PURPLE;
    else if (hasRefMs)
      lastBg = d.lastMs < refMs ? COL_GREEN : COL_RED;
  }
  drawTimeRow(pLast, "LAST", d.hasLast, d.lastMs, lastIsBest ? COL_PURPLE : COL_TEXT, lastLap, lastBg);

  // Dolny wiersz: okrazenie, do ktorego RaceChrono liczy delte. Gdy to najlepsze -> BEST (fiolet),
  // gdy w RC wybrano inne -> REF (zolty). Bez danych porownawczych - jak dawniej najlepsze.
  if (d.hasRef) {
    // czas + numer: samo porownanie numeru mylaloby przy okrazeniu porownawczym z innej sesji
    bool refIsBest = d.hasBest && d.refMs == d.bestMs &&
                     (!d.hasRefLapNumber || !d.hasBestLapNumber || d.refLapNumber == d.bestLapNumber);
    char refLap[12] = "#-";
    if (d.hasRefLapNumber) snprintf(refLap, sizeof(refLap), "#%ld", (long)d.refLapNumber);
    drawTimeRow(pBest, refIsBest ? "BEST" : "REF", true, d.refMs, refIsBest ? COL_PURPLE : COL_YELLOW, refLap);
  } else {
    drawTimeRow(pBest, "BEST", d.hasBest, d.bestMs, COL_PURPLE, bestLap);
  }
}

}  // namespace dash
