# PlatformIO (extra_scripts = pre:tools/version.py): wersja programu z gita - krotki hash commita,
# "+zmiany", gdy program budowany jest z niezacommitowanych zmian, i data kompilacji.
# Naglowek version_gen.h w katalogu kompilacji jest nadpisywany tylko przy zmianie tresci,
# wiec przy kazdej kompilacji przebudowuje sie tylko src/Version.cpp.
import datetime
import os
import subprocess

Import("env")  # noqa: F821 - dostarcza PlatformIO (SCons)


def git(*args):
    try:
        return subprocess.run(["git", *args], cwd=env.subst("$PROJECT_DIR"), capture_output=True, text=True,
                              timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None


head = git("rev-parse", "--short=7", "HEAD")
if head is not None and head.returncode == 0:
    version = head.stdout.strip()
    diff = git("diff", "--quiet", "HEAD", "--")  # tylko sledzone pliki: 1 = sa zmiany
    if diff is not None and diff.returncode == 1:
        version += " +zmiany"
else:
    version = "bez gita"
built = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

out_dir = os.path.join(env.subst("$BUILD_DIR"), "generated")
os.makedirs(out_dir, exist_ok=True)
path = os.path.join(out_dir, "version_gen.h")
text = '#pragma once\n#define FW_VERSION "%s"\n#define FW_BUILT "%s"\n' % (version, built)
try:
    with open(path) as f:
        old = f.read()
except OSError:
    old = None
if old != text:
    with open(path, "w") as f:
        f.write(text)
env.Append(CPPPATH=[out_dir])
print("Wersja programu: %s, kompilacja %s" % (version, built))
