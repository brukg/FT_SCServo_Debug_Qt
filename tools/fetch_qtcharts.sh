#!/usr/bin/env bash
# Vendor QtCharts locally without a system install (no sudo needed).
# The .pro uses the system 'charts' module if present, else this vendored copy.
set -euo pipefail
DEST="$(cd "$(dirname "$0")/.." && pwd)/third_party/qt5charts"
TMP="$(mktemp -d)"
cd "$TMP"
apt-get download libqt5charts5 libqt5charts5-dev
for d in *.deb; do dpkg -x "$d" ./x; done
mkdir -p "$DEST/include" "$DEST/lib"
cp -r x/usr/include/x86_64-linux-gnu/qt5/QtCharts "$DEST/include/"
cp -P x/usr/lib/x86_64-linux-gnu/libQt5Charts.so* "$DEST/lib/"
echo "QtCharts vendored to $DEST"
