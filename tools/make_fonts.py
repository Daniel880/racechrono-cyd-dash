#!/usr/bin/env python3
"""Generuje include/fonts_saira.h: czcionki VLW (wygladzane) Saira Bold dla LovyanGFX.

Tylko znaki potrzebne na ekranie: cyfry i "+-.:". Dwa rozmiary, dobrane pod panele w Dashboard.cpp:
  kFontDelta - panele delt (cyfry 44 px),
  kFontTime  - wiersze czasow (cyfry 37 px, "00:00.00" miesci sie w 232 px).
Cyfry dostaja jednakowa szerokosc (tabelaryczne), zeby czas nie skakal w poziomie.

Uzycie (wymaga Pillow):
  python3 tools/make_fonts.py <Saira[wdth,wght].ttf> include/fonts_saira.h
Czcionka Saira: Google Fonts, licencja SIL Open Font License 1.1.
"""
import struct
import sys

from PIL import Image, ImageFont

CHARS = "+-.0123456789:"
WEIGHT, WIDTH = 700, 100  # Bold, normalna szerokosc (osie: Weight, Width)
MAX_W = 232


def load(path, px):
    f = ImageFont.truetype(path, px)
    f.set_variation_by_axes([WEIGHT, WIDTH])
    return f


def digit_h(f):
    b = f.getbbox("0", anchor="ls")
    return b[3] - b[1]


def pick(path, target_h, widest):
    px = 10
    while digit_h(load(path, px + 1)) <= target_h:
        px += 1
    while load(path, px).getlength(widest) > MAX_W:
        px -= 1
    return load(path, px)


def vlw(f):
    glyphs = []
    for ch in CHARS:
        mask, (ox, oy) = f.getmask2(ch, mode="L", anchor="ls")
        w, h = mask.size
        img = Image.frombytes("L", (w, h), bytes(mask)) if w and h else None
        glyphs.append([ch, w, h, round(f.getlength(ch)), -oy, ox, img])
    # cyfry tabelaryczne: wspolny xAdvance, glif wysrodkowany
    adv = max(g[3] for g in glyphs if g[0].isdigit())
    for g in glyphs:
        if g[0].isdigit():
            g[5] += (adv - g[3]) // 2
            g[3] = adv
    ascent, descent = f.getmetrics()
    out = struct.pack(">6i", len(glyphs), 11, f.size, 0, ascent, descent)
    for ch, w, h, xa, dy, dx, _ in glyphs:
        out += struct.pack(">7i", ord(ch), h, w, xa, dy, dx, 0)
    for *_, img in glyphs:
        if img:
            out += img.tobytes()
    return out, f


def c_array(name, data):
    lines = [f"// {len(data)} B", f"static const uint8_t {name}[] PROGMEM = {{"]
    for i in range(0, len(data), 20):
        lines.append("  " + ", ".join(f"0x{b:02X}" for b in data[i:i + 20]) + ",")
    lines.append("};")
    return "\n".join(lines)


def main():
    ttf, out_path = sys.argv[1], sys.argv[2]
    delta, fd = vlw(pick(ttf, 44, "-99.99"))
    time, ft = vlw(pick(ttf, 37, "00:00.00"))
    with open(out_path, "w") as o:
        o.write("#pragma once\n// Wygenerowane przez tools/make_fonts.py - nie edytowac recznie.\n")
        o.write("// Saira Bold (SIL Open Font License 1.1), znaki: " + CHARS + "\n")
        o.write("#include <Arduino.h>\n\n")
        o.write(f"// panele delt: {fd.size} px, cyfry {digit_h(fd)} px\n")
        o.write(c_array("kFontDelta", delta) + "\n\n")
        o.write(f"// wiersze czasow: {ft.size} px, cyfry {digit_h(ft)} px\n")
        o.write(c_array("kFontTime", time) + "\n")
    print(f"delta {fd.size}px/{digit_h(fd)}px {len(delta)}B, time {ft.size}px/{digit_h(ft)}px {len(time)}B")


if __name__ == "__main__":
    main()
