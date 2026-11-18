#pragma once
// Backup trasy na karte SD (CYD), niezalezny od telefonu i RaceChrono.
// Kazde uruchomienie = nowa para plikow, nazwa = data i czas (UTC) pierwszego fixa GPS:
//   /gpx/2026-11-13_14-05-32.gpx   - trasa (czas, wysokosc, predkosc, kurs, satelity)
//   /gpx/2026-11-13_14-05-32.nmea  - surowe zdania NMEA z modulu GPS
// Pliki powstaja przy pierwszym fixie, wiec bez GPS nie ma pustych plikow.
// Gdyby plik o tej nazwie juz istnial, dopisywany jest numer: ..._2.gpx, ..._3.gpx.
// Brak karty albo blad zapisu (karta wyjeta, bez kontaktu, pelna) = stan Error. Co 5 s jest
// proba ponownego montowania; po udanej nastepny fix zaczyna nowa pare plikow.

#include <stdint.h>

#include "RaceChronoBle.h"  // rc::GpsFix

namespace gpx {

enum class State : int8_t { Off = 0, Error = -1, Waiting = 1, Logging = 2 };

void begin();                         // montuje karte SD i startuje zadanie zapisu (rdzen 0)
void addFix(const rc::GpsFix& fix);   // dopisuje punkt (tylko z fixem)
void nmeaLine(const char* line);      // dopisuje surowe zdanie NMEA

State state();

}  // namespace gpx
