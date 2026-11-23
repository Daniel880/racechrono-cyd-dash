#pragma once
// WiFi wyswietlacza (wlasna siec "RC-CYD-OTA"): ustawienia i wgrywanie programu.
// Przez 30 s po starcie wyswietlacz nadaje siec WiFi. Gdy nikt sie nie podlaczy, WiFi jest
// calkowicie wylaczane do nastepnego startu (nie zuzywa pradu). Z podlaczonym urzadzeniem
// siec trwa, dopoki jest polaczenie, i jeszcze 30 s po rozlaczeniu (najwyzej 15 min).
// Strona http://192.168.4.1: nazwa Bluetooth, tryb GPS i wgrywanie pliku firmware.bin;
// wgrywanie z PlatformIO: srodowisko cyd_ota w platformio.ini.

#include <stdint.h>

namespace ota {

enum class State : int8_t { Off = 0, Window = 1, Client = 2, Updating = 3 };

// onProgress: postep wgrywania 0..100 (do ekranu). onUpdateStart: tuz przed przyjmowaniem pliku -
// zwalnia RAM (GPS, zapis SD, Bluetooth). Po nieudanym wgrywaniu CYD robi restart.
void begin(void (*onProgress)(int percent), void (*onUpdateStart)());
void loop();                                  // wywolywac w loop(); podczas wgrywania blokuje
State state();

}  // namespace ota
