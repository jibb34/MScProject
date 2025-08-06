#!/bin/sh
# convert.sh

echo "Converting all .osm to .pbf..."

for f in /data/*.osm; do
  [ -e "$f" ] || continue # skip if no .osm files
  f_b=$(basename "$f")
  base="${f_b%.osm}"
  osmium cat "$f" -o "/data/fragments/${base}.pbf" -f pbf --overwrite
done

echo "Done!"
