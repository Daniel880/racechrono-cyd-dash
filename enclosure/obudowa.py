#!/usr/bin/env python3
"""Obudowa wyswietlacza RaceChrono CYD na motocykl (build123d) - dwie polowki 50/50 na wpust.

Przod = obudowa_v2 (projekt w SolidWorks, poza repozytorium): wneka na szklo ekranu, okno
ekranu, kieszen i okno GPS (antena wystaje przed obudowe) oraz kanal na wtyczke GPS 1:1.
  przod.stl - ekran, GPS, otwor na karte SD; drukowac ekranem do stolu
  tyl.stl   - bateria, ladowarka (prowadnice), przelacznik, USB-C; drukowac plecami do stolu
Wszystkie otwory (SD, USB-C, przelacznik) na prawym boku.
  zaslepka_tpu.stl - (TPU_COVER = True) klapka z TPU na SD i USB-C: czopy w otworach, trzpien z grzybkiem w otworze
  w scianie tylu (od strony GPS); drukowac plaska strona do stolu (juz tak ulozona)
Polowki laczy pioro (tyl) we wpuscie (przod) z luzem JOINT_TOL (wcisk, ewentualnie klej/tasma).
Otwory w scianach (SD, USB-C, przelacznik): opcjonalne zeberka podporowe do wylamania (SUPPORTS).

Uklad wspolrzednych jak w obudowa_v2: X = dlugosc (X=0 strona GPS/USB = gora ekranu,
X=109.2 strona anteny ESP32), Y = glebokosc (Y=0 przod/ekran, Y=25 tyl), Z = szerokosc.
Patrzac na ekran (GPS u gory): Z=0 po lewej, Z=57 po prawej.
Uzycie: python3 obudowa.py  -> stl/przod.stl, stl/tyl.stl (+ .step do SolidWorks), stl/elementy.stl
"""
import math
from pathlib import Path

from build123d import Align, Axis, Box, Cone, Cylinder, Pos, Rot, chamfer, export_step, export_stl, fillet

TOL = 0.2  # luz na wcisk

# ---------------- Przod z obudowa_v2 (wymiary ekranu i GPS NIE ZMIENIAC) ----------------
L, D, W = 109.2, 25.0, 57.0          # dlugosc X, glebokosc Y, szerokosc Z
WALL = 2.0                           # scianka v2 - od niej liczone wnetrze (NIE ZMIENIAC)
FRONT = 4.0                          # przednia plyta Y 0..4
SKIN = 1.0                           # warstwa przed szklem Y 0..1
RECESS = (28.2, 98.2, 3.5, 53.5)     # wneka na szklo ekranu: X0, X1, Z0, Z1 (Y 1..4)
RECESS_TOL = 0.1                     # dodatkowy luz wneki na kazda strone (w v2 ekran wchodzil ciezko)
WINDOW = (34.2, 95.2, 5.5, 51.5)     # okno ekranu (Y 0..1)
GPS_RECESS = (5.0, 28.2, 17.4, 39.6)  # kieszen GPS (Y 1..4)
GPS_WINDOW = (7.0, 25.2, 19.4, 37.6)  # okno - antena GPS wystaje przez nie przed obudowe
GPS_PLUG = (2.0, 5.0, 24.5, 32.5)    # kanal na wtyczke GPS (Y 1..4)

# ---------------- Ksztalt i polaczenie polowek ----------------
WALL_T = 2.4      # rzeczywista grubosc scian bocznych - pogrubione na zewnatrz, wnetrze bez zmian
E = WALL_T - WALL  # o tyle obudowa wieksza z kazdej strony niz v2
R_IN = 1.0        # narozniki wnetrza: plytka CYD wchodzi w rogi po stronie anteny, wiecej sie nie da
R_OUT = R_IN + WALL_T  # zewnetrzne wspolsrodkowe z wnetrzem - scianka rowna takze w naroznikach
FRONT_CH = 1.2    # faza przedniej krawedzi (na stole)
BACK_CH = 1.0     # faza tylnej krawedzi (na stole)
SEAM_CH = 0.4     # fazki na linii podzialu (rowek zamiast szpary)
BACK = 2.0        # tylna scianka
SPLIT = 13.0      # podzial polowek (ponizej: szczelina SD w przodzie, powyzej: USB-C i przelacznik w tyle)
LIP_H = 2.5       # dlugosc piora
LIP_T = 1.1       # grubosc piora (sciana przodu przy wpuscie: WALL_T - LIP_T - JOINT_TOL = 1.2)
JOINT_TOL = 0.1   # luz piora na boki (wpust na glebokosc ma TOL, zeby szew zawsze sie domknal)

# ---------------- Plytka CYD ----------------
PCB = (85.8, 1.5, 50.3)      # X, grubosc, Z (zmierzone)
PCB_X1 = L - WALL            # krawedz plytki po stronie anteny ESP32 (przy scianie, jak w v2) - SPRAWDZIC
PCB_X0 = PCB_X1 - PCB[0]
PCB_Y0 = SKIN + 4.0          # przod plytki: szklo od Y=1, do plytki 4 mm (zmierzone)
LDR_H = 1.0                  # fotorezystor wystaje z przodu plytki (pas bez szkla po stronie anteny ESP32)
LDR_RELIEF = 1.0             # wybranie w przedniej plycie nad tym pasem (plyta zostaje FRONT - LDR_RELIEF)
BACK_CLEAR = 5.0             # najwyzszy element pod plytka (zmierzone)
SD_X = PCB_X1 - 47.5         # srodek slotu SD: 47.5 mm od krawedzi po stronie anteny (zmierzone), prawy bok
SD_HOLE = (15.0, 3.2)        # otwor na karte: X (z zapasem), Y
SD_Y = PCB_Y0 + PCB[1] + 1.0  # srodek szczeliny SD (za plytka)
GPS_BASE_T, GPS_ANT_T = 3.7, 4.5  # GPS M10-DQ (8.2): plytka we wnece, antena przez okno - przyblizone

# ---------------- Elementy w tylnej polowce ----------------
Y_BACK = D - BACK            # wewnetrzna strona tylnej scianki
GUIDE_H, GUIDE_T = 1.5, 1.0  # prowadnice baterii i ladowarki
BAT = (54.0, 9.7, 41.0)      # bateria: X, grubosc Y, Z (zmierzone)
BAT_X0 = WALL + 1.0          # bateria od strony GPS, z dala od anteny ESP32
CHG = (17.0, 7.0, 22.0)      # ladowarka 17 x 22, wysokosc z przewodami 7; USB-C na krotszym boku, przy prawej scianie
CHG_X0 = 59.0
SW_HOLE = (13.0, 8.0)        # otwor na przelacznik: X, Y (zmierzone)
SW_DEPTH = 18.0              # glebokosc korpusu przelacznika (zmierzone)
SW_X, SW_Y = 86.0, 18.0      # srodek przelacznika (prawa sciana, korpus do srodka)
USB_HOLE = (9.6, 3.8)        # otwor na wtyk USB-C: X, Y

# ---------------- Podpory do wylamania (otwory w scianach bocznych) ----------------
SUPPORTS = False  # True = zeberka podporowe w otworach w scianach (po druku wylamac)
SUP_T = 0.8      # grubosc zeberka
SUP_GAP = 0.3    # przerwa do krawedzi drukowanej jako mostek (zeby sie nie zlaly)
SUP_SPAN = 5.0   # maksymalny rozstaw zeberek

# ---------------- Zaslepka TPU na SD i USB-C ----------------
TPU_COVER = False     # True = zaslepka_tpu.stl + otwor na jej trzpien w scianie tylu (na razie wylaczone)
COVER_T = 1.2         # grubosc klapki
COVER_MARGIN = 2.0    # klapka wieksza od otworow
COVER_R = 2.0         # zaokraglenie naroznikow klapki
COVER_NAIL = 0.6      # fazka od strony sciany - do podwazenia paznokciem
PLUG_DEPTH = 1.6      # czopy w otworach (sciana ma WALL_T)
PLUG_FIT = 0.05       # luz czopow na boki - TPU lekko na wcisk
ANCHOR = (45.0, 16.5)  # trzpien mocujacy: X, Y (tylna polowka, obok baterii - w srodku wolne miejsce)
ANCHOR_D = 3.0        # otwor w scianie i trzpien
BARB_D, BARB_H = 4.2, 1.4  # grzybek od srodka (stozek - przeciska sie przez otwor)


def bx(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def rbx(x0, x1, y0, y1, z0, z1, r):
    """Prostopadloscian z zaokraglonymi krawedziami rownoleglymi do Y (narozniki w widoku z przodu)."""
    return fillet(bx(x0, x1, y0, y1, z0, z1).edges().filter_by(Axis.Y), radius=r)


def ring(grow_in, grow_out, y0, y1):
    """Pierscien wzdluz scian: od wnetrza powiekszonego o grow_in do powiekszonego o grow_out."""
    def grown(g):
        return rbx(WALL - g, L - WALL + g, y0, y1, WALL - g, W - WALL + g, R_IN + g)
    return grown(grow_out) - grown(grow_in) if grow_in > -1 else grown(grow_out)


# ---------------- Polowki ----------------
def sd_rect():
    return (SD_X - SD_HOLE[0] / 2, SD_X + SD_HOLE[0] / 2, SD_Y - SD_HOLE[1] / 2, SD_Y + SD_HOLE[1] / 2)


def usb_rect():
    ux, uy = CHG_X0 + CHG[0] / 2, Y_BACK - CHG[1] + 1.6
    return (ux - USB_HOLE[0] / 2, ux + USB_HOLE[0] / 2, uy - USB_HOLE[1] / 2, uy + USB_HOLE[1] / 2)


def front_half(supports=SUPPORTS):
    f = rbx(-E, L + E, 0, SPLIT, -E, W + E, R_OUT)
    f = chamfer(f.faces().sort_by(Axis.Y)[0].edges(), length=FRONT_CH)
    f = chamfer(f.faces().sort_by(Axis.Y)[-1].edges(), length=SEAM_CH)
    f -= rbx(WALL, L - WALL, FRONT, SPLIT + 1, WALL, W - WALL, R_IN)                 # wnetrze
    f -= ring(-2, LIP_T + JOINT_TOL, SPLIT - LIP_H - TOL, SPLIT + 1)                # wpust na pioro
    rx0, rx1, rz0, rz1 = RECESS
    recess = (rx0 - RECESS_TOL, rx1 + RECESS_TOL, rz0 - RECESS_TOL, rz1 + RECESS_TOL)
    for x0, x1, z0, z1 in (recess, GPS_RECESS, GPS_PLUG):                          # wneki w przedniej plycie
        f -= bx(x0, x1, SKIN, FRONT + 0.01, z0, z1)
    # fotorezystor: plytka jest 4 mm za szklem, fotorezystor wystaje ~1 mm i dotykalby plyty
    f -= bx(recess[1], L - WALL, FRONT - LDR_RELIEF, FRONT + 0.01, recess[2], recess[3])
    for x0, x1, z0, z1 in (WINDOW, GPS_WINDOW):                                    # okna
        f -= bx(x0, x1, -1, SKIN + 0.01, z0, z1)
    # karta SD - prawa sciana, w plaszczyznie szczeliny
    sd = sd_rect()
    f -= bx(sd[0], sd[1], sd[2], sd[3], W - WALL - 1, W + E + 1)
    # druk ekranem do stolu (w gore = +Y): podpora stoi na dolnej krawedzi otworu
    if supports:
        f += breakaway(sd[0], sd[1], sd[2], sd[3], W - WALL, W + E)
    return f


def breakaway(x0, x1, y_bed, y_top, z0, z1):
    """Zeberka podpory w otworze w scianie: stoja na krawedzi y_bed (blizej stolu), koncza sie SUP_GAP
    przed krawedzia y_top (drukowana jako mostek); z0..z1 = grubosc sciany. Po druku wylamac."""
    n = max(1, math.ceil((x1 - x0) / SUP_SPAN) - 1)
    if y_top > y_bed:
        ya, yb = y_bed - 0.01, y_top - SUP_GAP
    else:
        ya, yb = y_top + SUP_GAP, y_bed + 0.01
    fins = None
    for i in range(n):
        x = x0 + (x1 - x0) * (i + 1) / (n + 1)
        fin = bx(x - SUP_T / 2, x + SUP_T / 2, ya, yb, z0, z1)
        fins = fin if fins is None else fins + fin
    return fins


def guides(x0, x1, z0, z1):
    """Ramka-prowadnica na tylnej sciance (wnetrze z luzem TOL)."""
    o = bx(x0 - TOL - GUIDE_T, x1 + TOL + GUIDE_T, Y_BACK - GUIDE_H, Y_BACK, z0 - TOL - GUIDE_T, z1 + TOL + GUIDE_T)
    return o - bx(x0 - TOL, x1 + TOL, Y_BACK - GUIDE_H - 1, Y_BACK + 1, z0 - TOL, z1 + TOL)


def back_half(supports=SUPPORTS):
    b = rbx(-E, L + E, SPLIT, D, -E, W + E, R_OUT)
    b = chamfer(b.faces().sort_by(Axis.Y)[-1].edges(), length=BACK_CH)
    b = chamfer(b.faces().sort_by(Axis.Y)[0].edges(), length=SEAM_CH)
    b -= rbx(WALL, L - WALL, SPLIT - 1, Y_BACK, WALL, W - WALL, R_IN)               # wnetrze
    b += ring(0, LIP_T, SPLIT - LIP_H, SPLIT + 0.01)                                # pioro
    # USB-C ladowania i przelacznik - prawa sciana (Z=57, ta sama strona co SD); ladowarka w prowadnicy
    usb = usb_rect()
    sw = (SW_X - SW_HOLE[0] / 2, SW_X + SW_HOLE[0] / 2, SW_Y - SW_HOLE[1] / 2, SW_Y + SW_HOLE[1] / 2)  # przelacznik
    for x0, x1, y0, y1 in (usb, sw):
        b -= bx(x0, x1, y0, y1, W - WALL - 1, W + E + 1)
        # druk plecami do stolu (w gore = -Y): podpora stoi na krawedzi blizej plecow (y1)
        if supports:
            b += breakaway(x0, x1, y1, y0, W - WALL, W + E)
    if TPU_COVER:  # otwor na trzpien zaslepki TPU
        b -= Pos(ANCHOR[0], ANCHOR[1], W - WALL - 0.5) * Cylinder(ANCHOR_D / 2, WALL_T + 1,
                                                                align=(Align.CENTER, Align.CENTER, Align.MIN))
    # prowadnice baterii i ladowarki (ladowarka bez scianki od strony gniazda USB)
    b += guides(BAT_X0, BAT_X0 + BAT[0], (W - BAT[2]) / 2, (W + BAT[2]) / 2)
    cz1 = W - WALL - TOL - GUIDE_T  # ladowarka gniazdem USB-C do prawej sciany
    g = guides(CHG_X0, CHG_X0 + CHG[0], cz1 - CHG[2], cz1)
    b += g - bx(CHG_X0 - 5, CHG_X0 + CHG[0] + 5, Y_BACK - 5, Y_BACK + 1, W - WALL - 2, W + E + 1)
    return b


def tpu_cover():
    """Klapka TPU na SD i USB-C (w ukladzie obudowy, przylega do prawego boku)."""
    zo = W + E                      # zewnetrzna powierzchnia prawej sciany
    rects = (sd_rect(), usb_rect())
    x0 = ANCHOR[0] - ANCHOR_D / 2 - COVER_MARGIN
    x1 = max(r[1] for r in rects) + COVER_MARGIN
    y0 = min(min(r[2] for r in rects), ANCHOR[1] - ANCHOR_D / 2) - COVER_MARGIN
    y1 = max(max(r[3] for r in rects), ANCHOR[1] + ANCHOR_D / 2) + COVER_MARGIN
    flap = bx(x0, x1, y0, y1, zo, zo + COVER_T)
    flap = fillet(flap.edges().filter_by(Axis.Z), radius=COVER_R)
    # fazka od strony sciany dookola - paznokiec wchodzi pod krawedz (przy druku na gorze)
    flap = chamfer(flap.faces().sort_by(Axis.Z)[0].edges(), length=COVER_NAIL)
    # czopy w otworach
    for r in rects:
        plug = bx(r[0] + PLUG_FIT, r[1] - PLUG_FIT, r[2] + PLUG_FIT, r[3] - PLUG_FIT, zo - PLUG_DEPTH, zo + 0.01)
        flap += chamfer(plug.faces().sort_by(Axis.Z)[0].edges(), length=0.3)
    # trzpien przez sciane + grzybek od srodka (stozek: wezszy na koncu, zeby sie przecisnal)
    flap += Pos(ANCHOR[0], ANCHOR[1], W - WALL) * Cylinder(ANCHOR_D / 2, zo - (W - WALL) + 0.01,
                                                            align=(Align.CENTER, Align.CENTER, Align.MIN))
    flap += Pos(ANCHOR[0], ANCHOR[1], W - WALL - BARB_H) * Cone(1.2, BARB_D / 2, BARB_H + 0.01,
                                                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    return flap


def tpu_cover_print():
    """Klapka obrocona plaska strona do stolu."""
    c = Rot(180, 0, 0) * tpu_cover()
    bb = c.bounding_box()
    return Pos(-bb.min.X, -bb.min.Y, -bb.min.Z) * c


def components():
    """Elementy do podgladu i sprawdzania kolizji (nie do druku)."""
    zc = W / 2
    return {
        "plytka": bx(PCB_X0, PCB_X1, PCB_Y0, PCB_Y0 + PCB[1], zc - PCB[2] / 2, zc + PCB[2] / 2),
        "ekran": bx(RECESS[0], RECESS[1], SKIN, PCB_Y0, RECESS[2], RECESS[3]),
        "GPS": bx(GPS_RECESS[0] + 0.6, GPS_RECESS[1] - 0.6, SKIN, SKIN + GPS_BASE_T, GPS_RECESS[2] + 0.1, GPS_RECESS[3] - 0.1)
        + bx(GPS_WINDOW[0] + 0.1, GPS_WINDOW[1] - 0.1, SKIN - GPS_ANT_T, SKIN, GPS_WINDOW[2] + 0.1, GPS_WINDOW[3] - 0.1),
        "fotorezystor": bx(RECESS[1] + 1, L - WALL - 1, PCB_Y0 - LDR_H, PCB_Y0, RECESS[2] + 1, RECESS[3] - 1),
        "elementy za plytka": bx(PCB_X0, PCB_X1, PCB_Y0 + PCB[1], PCB_Y0 + PCB[1] + BACK_CLEAR,
                                 zc - PCB[2] / 2, zc + PCB[2] / 2),
        "bateria": bx(BAT_X0, BAT_X0 + BAT[0], Y_BACK - BAT[1], Y_BACK, zc - BAT[2] / 2, zc + BAT[2] / 2),
        "ladowarka": bx(CHG_X0, CHG_X0 + CHG[0], Y_BACK - CHG[1], Y_BACK, W - WALL - TOL - GUIDE_T - CHG[2], W - WALL),
        "przelacznik": bx(SW_X - SW_HOLE[0] / 2, SW_X + SW_HOLE[0] / 2, SW_Y - SW_HOLE[1] / 2, SW_Y + SW_HOLE[1] / 2,
                          W - WALL - SW_DEPTH, W - WALL),
    }


if __name__ == "__main__":
    out = Path(__file__).parent / "stl"
    out.mkdir(exist_ok=True)
    for f in out.glob("*"):
        f.unlink()
    front, back, comp = front_half(), back_half(), components()
    parts = {"przod": front, "tyl": back}
    if TPU_COVER:
        parts["zaslepka_tpu"] = tpu_cover_print()
    for name, part in parts.items():
        export_stl(part, str(out / f"{name}.stl"))
        export_step(part, str(out / f"{name}.step"))
    allc = None
    for c in comp.values():
        allc = c if allc is None else allc + c
    export_stl(allc, str(out / "elementy.stl"))
    # kontrola: polowki nie nachodza na siebie, elementy nie koliduja z obudowa ani ze soba
    v = (front & back).volume
    if v > 0.05:
        print(f"KOLIZJA: przod z tylem ({v:.2f} mm3)")
    if TPU_COVER:  # zaslepka vs obudowa bez zeberek (te sie wylamuje) i vs elementy
        cover = tpu_cover()
        for part, pn in ((front_half(False), "przod"), (back_half(False), "tyl")):
            v = (cover & part).volume
            if v > 0.3:
                print(f"KOLIZJA: zaslepka z {pn} ({v:.1f} mm3)")
        for a, c in comp.items():
            v = (cover & c).volume
            if v > 0.3:
                print(f"KOLIZJA: zaslepka z {a} ({v:.1f} mm3)")
    names = list(comp)
    for i, a in enumerate(names):
        for part, pn in ((front, "przod"), (back, "tyl")):
            v = (comp[a] & part).volume
            if v > 0.3:
                print(f"KOLIZJA: {a} z {pn} ({v:.1f} mm3)")
        for b2 in names[i + 1:]:
            v = (comp[a] & comp[b2]).volume
            if v > 0.3 and {a, b2} not in ({"ekran", "plytka"}, {"fotorezystor", "plytka"}):  # stykaja sie
                print(f"KOLIZJA: {a} z {b2} ({v:.1f} mm3)")
    print(f"zewnetrzne: {L + 2 * E:.1f} x {W + 2 * E:.1f} x {D} mm, sciany {WALL_T} mm, podzial na glebokosci {SPLIT} mm")
