#!/usr/bin/env bash
set -euo pipefail

build_dir="${1:-build}"
stage="${RUNNER_TEMP:-$(mktemp -d)}/grsim-package"
cmake --install "$build_dir" --prefix "$stage"
app="$stage/grSim.app"
qt_prefix="$(brew --prefix qt@5)"
"$qt_prefix/bin/macdeployqt" "$app" -always-overwrite -verbose=2
codesign --force --deep --sign - "$app"
codesign --verify --deep --strict "$app"
test "$(lipo -archs "$app/Contents/MacOS/grSim")" = arm64
test -f "$app/Contents/Resources/config/Parsian.ini"
test -f "$app/Contents/PlugIns/platforms/libqcocoa.dylib"

# Reject non-system runtime dependencies outside the relocated application.
python3 - "$app" <<'PY'
import pathlib, re, subprocess, sys
app = pathlib.Path(sys.argv[1])
for path in app.rglob("*"):
    if not path.is_file() or path.is_symlink():
        continue
    if "Mach-O" not in subprocess.check_output(["file", "-b", str(path)], text=True):
        continue
    architectures = subprocess.check_output(["lipo", "-archs", str(path)], text=True).split()
    if "arm64" not in architectures:
        raise SystemExit(f"Missing arm64 slice: {path}")
    load_commands = subprocess.check_output(["otool", "-arch", "arm64", "-l", str(path)], text=True)
    for block in load_commands.split("Load command "):
        if "LC_BUILD_VERSION" in block or "LC_VERSION_MIN_MACOSX" in block:
            minimum = re.search(r"(?m)^\s+(?:minos|version)\s+(\d+(?:\.\d+)*)", block)
            if minimum:
                version = tuple(map(int, minimum.group(1).split(".")))
                if (version + (0, 0))[:2] > (15, 0):
                    raise SystemExit(f"Requires newer macOS than 15.0: {path}: {minimum.group(1)}")
        if "LC_RPATH" in block:
            rpath = re.search(r"(?m)^\s+path\s+(\S+)", block)
            if rpath and rpath.group(1).startswith(("/opt/", "/usr/local/", "/Users/")):
                raise SystemExit(f"External runtime search path in {path}: {rpath.group(1)}")
    for line in subprocess.check_output(["otool", "-L", str(path)], text=True).splitlines()[1:]:
        dependency = line.strip().split(" (", 1)[0]
        if dependency.startswith("/") and not dependency.startswith(("/System/Library/", "/usr/lib/")):
            raise SystemExit(f"External runtime dependency in {path}: {dependency}")
print("PASS relocated bundle dependency audit")
PY

mkdir -p dist
archive="$PWD/dist/grSim-macos-arm64-${MACOSX_DEPLOYMENT_TARGET:-15.0}.zip"
ditto -c -k --sequesterRsrc --keepParent "$app" "$archive"
relocated="$stage/relocated"
mkdir -p "$relocated"
ditto -x -k "$archive" "$relocated"
# Run the extracted download away from the source tree, without Qt path overrides.
(
    cd "$relocated"
    unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH DYLD_LIBRARY_PATH DYLD_FRAMEWORK_PATH
    ./grSim.app/Contents/MacOS/grSim --smoke-test
)
