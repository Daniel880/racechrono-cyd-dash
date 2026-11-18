# racechrono-cyd-dash

Wyswietlacz do [RaceChrono](https://racechrono.com) na plytce ESP32 "Cheap Yellow Display"
(ESP32-2432S028, panel ST7789 240x320). Laczy sie z telefonem przez BLE jako urzadzenie
[RaceChrono DIY](https://github.com/aollin/racechrono-ble-diy-device) i:

- pokazuje delte czasu i predkosci do okrazenia porownawczego oraz czasy okrazen (Monitor API),
- wysyla do RaceChrono pozycje z zewnetrznego modulu GPS u-blox M10 (do 25 Hz),
- zapisuje kopie trasy na karte microSD (GPX + surowe NMEA), niezalezna od telefonu.

## Sprzet

| Element | Polaczenie |
|---|---|
| Wyswietlacz | wbudowany: HSPI (SCLK 14, MOSI 13, MISO 12, CS 15, DC 2), podswietlenie 21 |
| Karta microSD (FAT32) | wbudowany slot: VSPI (SCK 18, MISO 19, MOSI 23, CS 5) |
| GPS GEPRC GEP-M10 (u-blox M10) | zlacze CN1: GPS TX -> IO27, GPS RX <- IO22, GND |

Jesli kolory sa odwrocone albo czerwony i niebieski sa zamienione, zmien `invert` / `rgb_order`
w `include/LGFX_CYD.h`.

## Ekran

```
| o GPS SD   145 km/h   14 sat 0.8 |  pasek stanu
|               +3                 |  delta predkosci [km/h] + pasek (zielony = szybciej)
|             -0.23                |  delta czasu [s] + pasek
| CURRENT #12            1:23.45   |  biezace okrazenie
| LAST #11               1:24.10   |  poprzednie okrazenie
| BEST #7                1:22.98   |  najlepsze (albo REF, gdy delta liczona jest do innego)
```

- Kropka: szara - brak polaczenia z RaceChrono, zolta - rejestracja monitorow, zielona - gotowe.
- `GPS`: szary - brak poprawnych danych z modulu, zolty - brak fixa, zielony - fix.
- `SD`: czerwony - brak karty albo blad zapisu (ponowna proba co 5 s), zolty - czeka na fix,
  zielony - zapis.
- Ikona WiFi (obok `SD`): zolte luki - siec WiFi czeka na podlaczenie, zielony wachlarz - ktos
  jest podlaczony.
- Prawy rog: satelity uzyte w rozwiazaniu (przed fixem szare, zwykle 0-3), z fixem dodatkowo
  szacowany blad pozycji, np. `18 sat 1.2m` (zielony < 2.5 m, zolty < 5 m, czerwony >= 5 m).
  To lepszy wskaznik niz HDOP: odbiornik liczy go z jakosci pomiarow, nie tylko z geometrii
  satelitow, wiec widac w nim tez slaby sygnal i odbicia.
- `LAST` po przejechaniu mety przez 10 s: fioletowy - nowe najlepsze, zielony / czerwony -
  szybciej / wolniej niz okrazenie porownawcze.
- Dolny wiersz: `BEST` (fiolet) albo `REF` (zolty), gdy w RaceChrono wybrano inne okrazenie
  porownawcze niz najlepsze.

## Konfiguracja

Flagi w `build_flags` w `platformio.ini`:

| Flaga | Znaczenie |
|---|---|
| `GPX_LOG` | backup na SD: `/gpx/RRRR-MM-DD_GG-MM-SS.gpx` i `.nmea` (czas UTC pierwszego fixa) |
| `GPS_UART_RX`, `GPS_UART_TX` | piny UART modulu GPS; bez `GPS_UART_RX` - praca bez modulu |
| `GPS_UBLOX_M10` | autokonfiguracja M10: wykrycie predkosci UART, czestotliwosc, tryb automotive |
| `GPS_UART_BAUD` | docelowa predkosc UART modulu |
| `GPS_MODE` | domyslny tryb GPS (zmiana na stronie WiFi): `25` - 25 Hz, sam GPS; `20` - 20 Hz, GPS+Galileo; `10` - 10 Hz, GPS+Galileo+BeiDou+QZSS |
| `SIM_GPS` | symulowany GPS (kolo ~20 s, 10 Hz) zamiast modulu - nie razem z `GPS_UART_RX` |
| `OTA_WIFI` | wgrywanie przez WiFi: siec `RC-CYD-OTA` przez 60 s po starcie (patrz nizej) |
| `GPS_DEBUG`, `RC_DEBUG_VALUES`, `RC_DEBUG_STATS`, `SIM_DEBUG` | diagnostyka na porcie szeregowym |

Modul jest konfigurowany przy kazdym starcie (tylko RAM modulu). Jesli przestanie wysylac
poprawne dane albo jego czestotliwosc spadnie (np. reset po chwilowym braku zasilania),
konfiguracja odbywa sie od nowa. Gdy modul nie odpowiada na komendy (przewod IO22 -> RX modulu),
urzadzenie pracuje na jego wlasnych ustawieniach i co 30 s ponawia konfiguracje.

## Budowanie

[PlatformIO](https://platformio.org):

```
pio run -t upload
pio device monitor
```

Port szeregowy: 115200 bps.

### WiFi: ustawienia i wgrywanie

Przez 60 s po wlaczeniu wyswietlacz nadaje siec WiFi `RC-CYD-OTA` (zolta ikona WiFi na pasku
stanu). Gdy nikt sie nie podlaczy, WiFi wylacza sie do nastepnego startu i nie zuzywa pradu.
Z podlaczonym urzadzeniem (zielona ikona) siec trwa, dopoki jest polaczenie, i 60 s po rozlaczeniu.

Strona `http://192.168.4.1` pozwala ustawic nazwe Bluetooth (widoczna w RaceChrono) i tryb GPS
(25 / 20 / 10 Hz). Ustawienia sa zapisane w pamieci plytki - przetrwaja restart i wgranie nowego
programu; po zapisie wyswietlacz sie restartuje.

1. Wlacz wyswietlacz i w ciagu 60 s podlacz laptop albo telefon do sieci `RC-CYD-OTA`.
2. Wgraj program jednym ze sposobow:
   - **przegladarka:** otworz `http://192.168.4.1`, wybierz plik `.pio/build/cyd/firmware.bin`
     (powstaje przy kazdym Build) i kliknij Wgraj. Telefon moze ostrzec, ze siec nie ma
     internetu - trzeba zostac polaczonym;
   - **PlatformIO:** `pio run -e cyd_ota -t upload` (w VSCode: wybor srodowiska na dolnym pasku,
     potem Upload).
3. Na ekranie widac postep, po wgraniu wyswietlacz sam sie restartuje.

Haslo sieci i wgrywania: sekcja `[ota]` w pliku `ota_secret.ini` (poza gitem), np.

```
[ota]
password = twoje-haslo
```

Bez tego pliku obowiazuje haslo domyslne z `platformio.ini`. Pierwsze wgranie po przejsciu na
dwie partycje aplikacji (`min_spiffs.csv`) musi byc przez USB (srodowisko `cyd`). Jesli wgrywanie
nie startuje, sprawdz zapore na laptopie - wyswietlacz laczy sie zwrotnie z laptopem po TCP.

Czcionki w `include/fonts_saira.h` generuje `tools/make_fonts.py` (wymaga Pillow) z czcionki
Saira (SIL Open Font License 1.1).

## RaceChrono

Urzadzenie rozglasza sie przez BLE jako `RC DIY CYD` (nazwe mozna zmienic na stronie WiFi;
RaceChrono rozpoznaje urzadzenie po usludze BLE, nie po nazwie). Po polaczeniu z RaceChrono samo rejestruje
potrzebne monitory (kropka na pasku stanu robi sie zielona). Zeby RaceChrono korzystalo z pozycji
z modulu GPS, wybierz to urzadzenie w RaceChrono jako zrodlo GPS.
