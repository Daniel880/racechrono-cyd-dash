#pragma once
// Strony WWW wyswietlacza (serwer w OtaUpdate.cpp, dziala tylko przy wlaczonym WiFi):
//   /files - pliki z karty SD: pobieranie i usuwanie
//   /gps   - diagnostyka GPS: czestotliwosc, satelity, sygnal, test zaklocen
// Wszystko obslugiwane w zadaniu loop() - podczas pobierania pliku ekran stoi.

class WebServer;

namespace web {

extern const char kHead[];  // poczatek kazdej strony: styl i naglowek (definicja w OtaUpdate.cpp)

void addFileRoutes(WebServer& server);
void addGpsRoutes(WebServer& server);

}  // namespace web
