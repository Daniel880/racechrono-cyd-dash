#pragma once
// Backup trasy na karte SD (CYD), niezalezny od telefonu i RaceChrono.
// Kazde uruchomienie = nowa para plikow, nazwa = data i czas (UTC) pierwszego fixa GPS:
//   /gpx/2026-11-13_14-05-32.gpx   - trasa (czas, wysokosc, predkosc, kurs, satelity)
//   /gpx/2026-11-13_14-05-32.nmea  - surowe zdania NMEA z modulu GPS
// Pliki powstaja przy pierwszym fixie, wiec bez GPS nie ma pustych plikow.
// Gdyby plik o tej nazwie juz istnial, dopisywany jest numer: ..._2.gpx, ..._3.gpx.
// Karta: FAT16/FAT32 albo exFAT (fabryczny na kartach 64 GB+) na tablicy partycji MBR. GPT
// (czesto po formatowaniu w Linuksie) nie jest obslugiwane - stan Error jak brak karty.
// Brak karty albo blad zapisu (karta wyjeta, bez kontaktu, pelna) = stan Error. Co 5 s jest
// proba ponownego montowania; po udanej nastepny fix zaczyna nowa pare plikow.

#include <stdint.h>

#include "RaceChronoBle.h"  // rc::GpsFix

namespace gpx {

// Disabled = zapis wylaczony do restartu, bez prob montowania: tryb awaryjny po awariach (SysLog)
// albo brak RAM na kolejke / zadanie zapisu.
enum class State : int8_t { Off = 0, Error = -1, Disabled = -2, Waiting = 1, Logging = 2 };

// enabled = false: tylko stan Disabled (czerwone SD), karta nie jest nawet montowana.
void begin(bool enabled);             // montuje karte SD i startuje zadanie zapisu (rdzen 0)
void addFix(const rc::GpsFix& fix);   // dopisuje punkt (tylko z fixem)
void nmeaLine(const char* line);      // dopisuje surowe zdanie NMEA

State state();
const char* stateName();
const char* fileName();  // biezaca para plikow (bez katalogu i rozszerzenia), "" gdy nie zapisuje

// ---------- Pliki w /gpx dla strony WWW (inne zadanie niz zapis) ----------
// Kazde wywolanie bierze blokade karty tylko na chwile, wiec zapis trasy idzie dalej.
// Nazwa pliku bez katalogu: litery, cyfry, '-', '_', '.' (inne nazwy sa pomijane na liscie).
struct FileInfo {
  char name[48];
  uint32_t size;      // bajty zapisane na karcie (biezacy plik: do ostatniego zapisu, co 2 s)
  char modified[20];  // "RRRR-MM-DD GG:MM" (UTC z GPS)
  bool active;        // biezaca para zapisu - nie do usuniecia
};
bool listOpen();  // false = karta niezamontowana
bool listNext(FileInfo& out);
void listClose();
bool readOpen(const char* name, uint32_t& size);
constexpr int kReadBusy = -2;              // readChunk(): karta zajeta przez zapis - sprobuj znowu
int readChunk(uint8_t* buf, size_t len);  // liczba bajtow, 0 = koniec, kReadBusy, inne < 0 = blad
void readClose();
const char* removeFile(const char* name);  // nullptr = usunieto, inaczej powod
int removeInactive();                      // usuwa wszystko poza biezaca para; < 0 = brak karty
uint32_t cardMB();                         // 0 = brak karty

// Przed wgrywaniem programu: zapisuje zalegle dane i domyka pliki, odmontowuje karte, konczy
// zadanie zapisu i zwalnia kolejke (~16 kB RAM). Stan Disabled do restartu.
void shutdown();

// Test zaklocen GPS: przez ms bez zadnych operacji na karcie (dane z tego czasu przepadaja).
void pauseFor(uint32_t ms);
uint32_t pauseLeftMs();

}  // namespace gpx
