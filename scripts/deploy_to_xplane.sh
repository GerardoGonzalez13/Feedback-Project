#!/usr/bin/env bash
set -e

XPLANE_DIR="/mnt/d/X-Plane 12"
DEST="$XPLANE_DIR/Resources/plugins/PythonPlugins"

mkdir -p "$DEST"
cp -v xplane_plugin/PI_*.py "$DEST/"
echo "Deployed PI_*.py -> $DEST"

