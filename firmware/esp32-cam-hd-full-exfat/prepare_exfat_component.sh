#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${IDF_PATH:-}" ]]; then
  echo "ERROR: IDF_PATH is not set."
  exit 1
fi

SRC="$IDF_PATH/components/fatfs"
DST="$(pwd)/components/fatfs"

if [[ ! -d "$SRC" ]]; then
  echo "ERROR: Could not find ESP-IDF FatFs component at: $SRC"
  exit 1
fi

mkdir -p "$(pwd)/components"
rm -rf "$DST"
cp -a "$SRC" "$DST"

CONF="$DST/src/ffconf.h"

python3 - "$CONF" <<'PY'
from pathlib import Path
import re
import sys

p = Path(sys.argv[1])
s = p.read_text()

s, exfat_count = re.subn(
    r'(?m)^(\s*#define\s+FF_FS_EXFAT\s+)0(\s*)$',
    r'\g<1>1\2',
    s,
    count=1,
)
if exfat_count != 1:
    raise SystemExit("Could not patch FF_FS_EXFAT in ffconf.h")

s, label_count = re.subn(
    r'(?m)^(\s*#define\s+FF_USE_LABEL\s+)CONFIG_FATFS_USE_LABEL(\s*)$',
    r'\g<1>0\2',
    s,
    count=1,
)
if label_count != 1:
    raise SystemExit("Could not patch FF_USE_LABEL in ffconf.h")

p.write_text(s)
print("Patched:", p)
print("  FF_FS_EXFAT = 1")
print("  FF_USE_LABEL = 0")
print("  FF_LBA64 = 0 (64-256 GB cards are within 32-bit sector addressing)")
PY

echo "Project-local exFAT FatFs override prepared."
