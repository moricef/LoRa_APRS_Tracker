#!/usr/bin/env bash
# Usage:
#   ./update_version.sh 2.7.6 2026-03-20           # stable + devel même version
#   ./update_version.sh 2.7.6 2026-03-20 devel      # devel seulement
#   ./update_version.sh 2.7.6 2026-03-20 stable     # stable seulement

set -e

VERSION="$1"
DATE="$2"
TARGET="${3:-stable}"  # stable | devel | both

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# Pour devel : les fichiers sont dans le repo courant (appeler depuis LoRa_APRS_Tracker-async/)
DEVEL_DIR="$(pwd)"

# Si la version n'est pas fournie, extraire la version stable actuelle de ui_common.h
if [[ -z "$VERSION" ]]; then
    VERSION=$(grep "#define UI_VERSION " "$SCRIPT_DIR/include/ui_common.h" | awk -F'"' '{print $2}')
    echo "Version non fournie, utilisation de la version actuelle de ui_common.h : $VERSION"
fi

if [[ -z "$DATE" ]]; then
    echo "Usage: $0 [version] <date> [stable|devel|both]"
    echo "  ex (version + date): $0 2.7.6 2026-03-20"
    echo "  ex (date seule):     $0 '' 2026-03-20"
    exit 1
fi

DATE_HTML="${DATE//-/.}"

if [[ "$TARGET" == "stable" || "$TARGET" == "both" ]]; then
    # include/ui_common.h
    sed -i "s/#define UI_VERSION \".*\"/#define UI_VERSION \"$VERSION\"/" "$SCRIPT_DIR/include/ui_common.h"
    sed -i "s/#define UI_VERSION_DATE \".*\"/#define UI_VERSION_DATE \"$DATE\"/" "$SCRIPT_DIR/include/ui_common.h"

    # docs/manifest-stable.json
    sed -i "s/\"version\": \".*\"/\"version\": \"$VERSION ($DATE)\"/" "$SCRIPT_DIR/docs/manifest-stable.json"

    # docs/index.html
    sed -i "s/v[0-9]\+\.[0-9]\+\.[0-9]\+<\/strong> (Stable - [0-9.]\+)/v$VERSION<\/strong> (Stable - $DATE_HTML)/" "$SCRIPT_DIR/docs/index.html"

    echo "Stable mis à jour : $VERSION ($DATE)"
fi

if [[ "$TARGET" == "devel" || "$TARGET" == "both" ]]; then
    # include/ui_common.h
    sed -i "s/#define UI_VERSION \".*\"/#define UI_VERSION \"$VERSION\"/" "$DEVEL_DIR/include/ui_common.h"
    sed -i "s/#define UI_VERSION_DATE \".*\"/#define UI_VERSION_DATE \"$DATE\"/" "$DEVEL_DIR/include/ui_common.h"

    # docs/devel.html — format: <strong>DATE</strong> (VERSION+dev)
    sed -i "s/<strong>[0-9-]\+<\/strong> ([0-9a-z.+]\+) (Development Branch)/<strong>$DATE<\/strong> (${VERSION}+dev) (Development Branch)/" "$DEVEL_DIR/docs/devel.html"

    # docs/manifest-devel.json
    sed -i "s/\"version\": \".*\"/\"version\": \"${VERSION}+dev ($DATE)\"/" "$DEVEL_DIR/docs/manifest-devel.json"

    echo "Devel mis à jour : ${VERSION}+dev ($DATE)"
fi

echo ""
if [[ "$TARGET" == "stable" || "$TARGET" == "both" ]]; then
    grep "UI_VERSION" "$SCRIPT_DIR/include/ui_common.h"
    grep "version" "$SCRIPT_DIR/docs/manifest-stable.json"
    grep "Firmware version" "$SCRIPT_DIR/docs/index.html"
fi
if [[ "$TARGET" == "devel" || "$TARGET" == "both" ]]; then
    grep "version" "$DEVEL_DIR/docs/manifest-devel.json"
    grep "Firmware version" "$DEVEL_DIR/docs/devel.html"
fi
