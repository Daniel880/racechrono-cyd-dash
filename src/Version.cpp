#include "Version.h"

// Generowany przy kazdej kompilacji przez tools/version.py (katalog kompilacji, poza repozytorium).
#if __has_include("version_gen.h")
#include "version_gen.h"
#else
#define FW_VERSION "?"
#define FW_BUILT __DATE__ " " __TIME__
#endif

namespace fw {
const char* const kVersion = FW_VERSION;
const char* const kBuilt = FW_BUILT;
}  // namespace fw
