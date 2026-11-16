#pragma once
// Symulator GPS: jazda po kole ze 100 punktow (~20 s/okrazenie), 10 Hz, do testow bez wyjezdzania na tor.

#include "RaceChronoBle.h"

namespace gpssim {
void begin();                  // wypisuje na Serial wspolrzedne linii start/meta
bool update(rc::GpsFix& out);  // true co 100 ms (10 Hz), gdy jest nowa pozycja
}  // namespace gpssim
