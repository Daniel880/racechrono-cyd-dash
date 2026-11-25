#pragma once
// Log systemowy: kazdy wpis idzie na Serial i do bufora w pamieci RTC, ktora przetrwa
// restart po bledzie (panic, watchdog, brownout) i ESP.restart(); znika przy odlaczeniu zasilania.
// Dwa bufory na zmiane: biezace uruchomienie i poprzednie - oba widac na stronie WiFi (/log),
// wiec po awarii mozna przeczytac ostatnie komunikaty sprzed restartu bez kabla USB.
//
// Ochrona przed petla restartow: gdy dwa kolejne uruchomienia skonczyly sie bledem w ciagu
// kStableMs od startu, safeMode() = true i main.cpp uruchamia tylko ekran i WiFi (bez GPS,
// Bluetooth i zapisu na SD) - obudowa jest zaklejona, wiec poprawke da sie wgrac tylko przez WiFi.
// Na stronie widac log awarii.
// Wylaczenie i wlaczenie zasilania albo restart programowy (np. po wgraniu programu) kasuje licznik.

#include <stddef.h>
#include <stdint.h>

namespace syslog {

void begin();  // pierwsza rzecz w setup(): przyczyna restartu, licznik awarii, nowy bufor
void loop();   // w loop(): po kStableMs uznaje uruchomienie za stabilne
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));  // jak Serial.printf

// Dodatkowe miejsce na kazdy wpis (z czasem od startu), np. plik na karcie SD. Wolane z zadania,
// ktore logowalo - funkcja musi byc szybka i nie moze sama logowac.
void setSink(void (*sink)(const char* line, size_t len));

bool safeMode();
uint8_t crashCount();      // kolejne uruchomienia zakonczone bledem tuz po starcie
const char* resetReason();  // dlaczego zakonczylo sie poprzednie uruchomienie (opis po polsku)

// Kopia logu (bez zakonczenia \0 w srodku) - previous = poprzednie uruchomienie.
// Zwraca liczbe bajtow; out musi miec miejsce na kBufBytes + 1.
constexpr size_t kBufBytes = 3000;
size_t copy(bool previous, char* out);

}  // namespace syslog
