#!/bin/bash
set -e

INPUT_DIR="/data/fragments"
OUTPUT="/data/merged.pbf"

INPUT_FILES=""

echo "[*] Looking for .pbf files to merge..."

for f in /data/fragments/*.pbf; do
  # Skip the output file if it already exists in the same folder
  [ "$f" = "$OUTPUT" ] && continue

  # Skip if the file doesn't actually exist (in case glob doesn't match)
  [ ! -f "$f" ] && continue

  echo " [+] Including $f"
  INPUT_FILES="$INPUT_FILES $f"
done

# Check if we found any .pbf files
if [ -z "$INPUT_FILES" ]; then
  echo "[!] No .pbf files found to merge."
  exit 1
fi

echo "[*] Merging files into $OUTPUT..."
osmium merge $INPUT_FILES -o "$OUTPUT" --overwrite

echo "Merge complete: $OUTPUT"
