#!/usr/bin/env bash
set -euo pipefail

build_dir="${1:-build}"
stage="${RUNNER_TEMP:-$(mktemp -d)}/grsim-package"
cmake --install "$build_dir" --prefix "$stage"
app="$stage/grSim.app"
qt_prefix="$(brew --prefix qt@5)"
"$qt_prefix/bin/macdeployqt" "$app" -always-overwrite -verbose=2
test "$(lipo -archs "$app/Contents/MacOS/grSim")" = arm64
test -f "$app/Contents/Resources/config/Parsian.ini"
test -f "$app/Contents/Resources/config/Utama-Radio-0.ini"
test -f "$app/Contents/Resources/config/Utama-Radio-1.ini"
test -f "$app/Contents/Resources/config/Utama-Field-20261028.xml"
test -f "$app/Contents/PlugIns/platforms/libqcocoa.dylib"

# Complete transitive dependencies before removing development search paths.
python3 tests/deploy_macos.py "$app" "$(brew --prefix)" "$qt_prefix"
codesign --force --deep --sign - "$app"
codesign --verify --deep --strict "$app"

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
    QT_DEBUG_PLUGINS=1 ./grSim.app/Contents/MacOS/grSim --smoke-test
    ./grSim.app/Contents/MacOS/grSim --smoke-test \
        --settings "$PWD/grSim.app/Contents/Resources/config/Utama-Field-20261028.xml"
)
