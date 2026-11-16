#pragma once
// Modul GPS (NMEA) na UART. Kazda nowa pozycja trafia jako rc::GpsFix,
// a kazde surowe zdanie NMEA do gpx::nmeaLine() (backup na SD).

#include <stdint.h>

#include "RaceChronoBle.h"  // rc::GpsFix

namespace gpsrx {

enum class State : int8_t { Off = 0, NoData = 1, NoFix = 2, Fix = 3 };

// Jakosc odbioru do wskaznika na ekranie
struct Quality {
  uint8_t used;     // satelity uzyte w fixie (GGA), 0 bez fixa
  uint8_t tracked;  // satelity z sygnalem (GSV) - rosnie juz przed fixem
  float hdop;       // 0 = brak
  uint8_t fix;      // pole 6 GGA: 0 brak, 1 GPS, 2 SBAS/DGPS
};

void begin();
bool update(rc::GpsFix& out);  // true = nowa pozycja (jedna na epoke GPS)
State state();
Quality quality();

}  // namespace gpsrx
