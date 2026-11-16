#pragma once
// Modul GPS (NMEA) na UART. Kazda nowa pozycja trafia jako rc::GpsFix,
// a kazde surowe zdanie NMEA do gpx::nmeaLine() (backup na SD).

#include "RaceChronoBle.h"  // rc::GpsFix

namespace gpsrx {

enum class State : int8_t { Off = 0, NoData = 1, NoFix = 2, Fix = 3 };

void begin();
bool update(rc::GpsFix& out);  // true = nowa pozycja (jedna na epoke GPS)
State state();

}  // namespace gpsrx
