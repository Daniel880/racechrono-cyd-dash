#pragma once
// Ustawienia zmieniane na stronie WiFi (http://192.168.4.1), zapisane w pamieci NVS -
// przetrwaja restart i wgranie nowego programu.

#include <stddef.h>
#include <stdint.h>

namespace settings {

// Nazwa BLE miesci sie wtedy w pakiecie rozglaszania razem z UUID uslugi RaceChrono.
constexpr size_t kNameMax = 20;

struct Settings {
  char btName[kNameMax + 1];
  uint8_t gpsMode;  // 25 = 25 Hz GPS, 20 = 20 Hz GPS+Galileo, 10 = 10 Hz GPS+Galileo+BeiDou
};

Settings load();               // domyslne wartosci, gdy nic nie zapisano
bool save(const Settings& s);  // false = niepoprawne wartosci albo blad zapisu
bool validName(const char* name);  // 1-20 znakow: litery, cyfry, spacja, - _ # .
bool validMode(int mode);

}  // namespace settings
