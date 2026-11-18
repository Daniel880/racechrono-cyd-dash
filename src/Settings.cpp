#include "Settings.h"

#include <Preferences.h>
#include <string.h>

// Domyslny tryb GPS (przed pierwszym zapisem na stronie WiFi).
#ifndef GPS_MODE
#define GPS_MODE 20
#endif
static_assert(GPS_MODE == 10 || GPS_MODE == 20 || GPS_MODE == 25, "GPS_MODE musi byc 25, 20 albo 10");

namespace settings {
namespace {
constexpr const char* kNamespace = "cfg";
constexpr const char* kDefaultName = "RC DIY CYD";
}  // namespace

bool validMode(int mode) { return mode == 10 || mode == 20 || mode == 25; }

bool validName(const char* name) {
  const size_t len = strlen(name);
  if (len == 0 || len > kNameMax || name[0] == ' ' || name[len - 1] == ' ') return false;
  for (const char* p = name; *p; p++) {
    const char c = *p;
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ' ||
                    c == '-' || c == '_' || c == '#' || c == '.';
    if (!ok) return false;
  }
  return true;
}

Settings load() {
  Settings s{};
  strncpy(s.btName, kDefaultName, kNameMax);
  s.gpsMode = GPS_MODE;
  Preferences p;
  if (p.begin(kNamespace, true)) {  // false = jeszcze nic nie zapisano
    char name[kNameMax + 1] = "";
    if (p.getString("btname", name, sizeof(name)) > 0 && validName(name)) strcpy(s.btName, name);
    const uint8_t mode = p.getUChar("gpsmode", 0);
    if (validMode(mode)) s.gpsMode = mode;
    p.end();
  }
  return s;
}

bool save(const Settings& s) {
  if (!validName(s.btName) || !validMode(s.gpsMode)) return false;
  Preferences p;
  if (!p.begin(kNamespace, false)) return false;
  const bool ok = p.putString("btname", s.btName) > 0 && p.putUChar("gpsmode", s.gpsMode) == 1;
  p.end();
  return ok;
}

}  // namespace settings
