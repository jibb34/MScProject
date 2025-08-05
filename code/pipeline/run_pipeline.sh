#!/bin/bash

# ------------------------------------------------------------------
# 0. Configuration constants
# ------------------------------------------------------------------

VENV_DIR=".venv"
REQ_FILE="requirements.txt"
CONFIG_FILE="settings.yml"
PROFILE_PATH="docker/osrm/profile.lua"
OSRM_DATA_DIR="docker/osrm/data"
export COMPOSE_FILE=docker/docker-compose.yml
# ------------------------------------------------------------------
# 1. YAML parsing function
# ------------------------------------------------------------------

function parse_yaml_value() {
  grep "^$1" "$CONFIG_FILE" | sed "s/^$1:[ ]*//" | tr -d '"'
}

if [ ! -f "$CONFIG_FILE" ]; then
  echo "[ERROR] $CONFIG_FILE not found!"
  exit 1
fi

# ------------------------------------------------------------------
# 1.1. Load all settings into variables
# ------------------------------------------------------------------

GPX_FILE=$(parse_yaml_value "gpx_file")
OSM_OUTPUT=$(parse_yaml_value "osm_output")
CHUNK_SIZE=$(parse_yaml_value "chunk_size")
JSON_DIR=$(parse_yaml_value "json_directory")
RADIUS=$(parse_yaml_value "radius")
DYNAMIC_WINDOW=$(parse_yaml_value "dynamic_radius_window")
# ... load other keys similarly ...
OSRM_BASENAME=$(basename "$OSM_OUTPUT" .osm)

# Error if no .gpx or output directory found
if [ -z "$GPX_FILE" ] || [ -z "$OSM_OUTPUT" ]; then
  "[ERROR] gpx_file and osm_output must be set in $CONFIG_FILE"
  exit 1
fi

if [ -z "$CHUNK_SIZE" ]; then
  CHUNK_SIZE=0
fi

if [ "$CHUNK_SIZE" -gt 200 ]; then
  echo "[ERROR] Chunk Size cannot go beyond 50 points, due to current GET limits"
  exit 1
fi

echo "Starting pipeline..."
echo "[INIT] GPX file: $GPX_FILE"
echo "[INIT] OSM output file: $OSM_OUTPUT"

# ------------------------------------------------------------------
# 2. Create & activate Python virtualenv
# ------------------------------------------------------------------

if [ ! -d "$VENV_DIR" ]; then
  echo "[INIT] Creating virtual environment..."
  python3 -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"

echo "[INIT] Installing Python dependencies (if needed)..."
pip install --upgrade pip
pip install --quiet -r "$REQ_FILE"

# ------------------------------------------------------------------
# 3. Prepare map & split GPX
# ------------------------------------------------------------------

echo "[PYTHON] Parsing GPX file and getting OSM map file"
python3 source/prepare_map.py \
  "$GPX_FILE" \
  "$OSM_OUTPUT" \
  "$JSON_DIR" \
  "$RADIUS" \
  "$CHUNK_SIZE"
echo "[PYTHON] Map Downloaded as .osm"

# ------------------------------------------------------------------
# 4. Copy OSM into OSRM dir
# ------------------------------------------------------------------

echo "[SHELL] Copying files to Docker Volumes"
mkdir -p "$OSRM_DATA_DIR"
cp "$OSM_OUTPUT" "$OSRM_DATA_DIR/$OSRM_BASENAME.osm"

# ------------------------------------------------------------------
# 5. Convert .osm → .pbf via docker osmium
# ------------------------------------------------------------------

#Build Docker image
echo "[DOCKER] Starting Osmium Docker container"
# docker build -t osmium-converter ./osmium
echo "[DOCKER] Performing conversion of OSM XML to PBF with Osmium"
docker compose run --rm --build osmium

# docker run --rm \
#   -v "$PWD/$OSRM_DATA_DIR:/data" \
#   osmium-converter \
#   cat "/data/$OSRM_BASENAME.osm" -o "/data/$OSRM_BASENAME.osm.pbf" -f pbf --overwrite >>/dev/null 2>&1

# Return file info for user feedback
# docker run --rm \
#   -v "$PWD/$OSRM_DATA_DIR:/data" \
#   osmium-converter \
#   fileinfo "/data/$OSRM_BASENAME.osm.pbf"

if [ ! -f "data/osm_files/merged.pbf" ]; then
  echo "[FATAL] Osmium conversion failed or output is missing"
  exit 1
fi

cp ./data/osm_files/merged.pbf ./data/osrm_map/map.pbf

echo "[DOCKER] Launching OSRM Server"
# docker compose up -d --rm osrm-server #>>/dev/null 2>&1
# ------------------------------------------------------------------
# 6. Prepare OSRM graph via docker osrm-backend
# ------------------------------------------------------------------
echo "[DOCKER] Pre-processing map.pbf with custom profile: $PROFILE_PATH"
docker compose --profile prep up --abort-on-container-exit osrm-prep
# docker run --rm \
#   -v "$PWD/$OSRM_DATA_DIR:/data" \
#   -v "$PWD/$PROFILE_PATH:/opt/profile.lua" \
#   osrm/osrm-backend \
#   bash -c "\
#   osrm-extract -p /opt/profile.lua '/data/$OSRM_BASENAME.osm' &&
#   osrm-partition '/data/$OSRM_BASENAME.osrm' &&
#   osrm-customize '/data/$OSRM_BASENAME.osrm' " >>/dev/null 2>&1

# ------------------------------------------------------------------
# 7. Start detached OSRM routing server
# ------------------------------------------------------------------

docker compose up -d osrm-server
# docker run -d \
#   -v "$PWD/$OSRM_DATA_DIR:/data" \
#   -p 5000:5000 \
#   --name osrm \
#   osrm/osrm-backend \
#   osrm-routed --algorithm mld --max-matching-size 500 "/data/$OSRM_BASENAME.osrm" >>/dev/null 2>&1

# ------------------------------------------------------------------
# 8. Batch matching of GPX chunks
# ------------------------------------------------------------------

python3 source/batch_route_calc.py "./data/temp" $DYNAMIC_WINDOW

# ------------------------------------------------------------------
# 9. Merge chunks
# ------------------------------------------------------------------
python3 source/merge_routes.py

# ------------------------------------------------------------------
# 10. Visualize results
# ------------------------------------------------------------------
# python3 plot_map.py
python3 source/plot_merged.py #OPTIONAL: Add other methods of visualisation...
# this area is just for testing purposes.

#
# ------------------------------------------------------------------
# 11. Cleanup
# ------------------------------------------------------------------

echo "[CLEANUP] Cleaning up JSON chunks..."
rm -rf data/temp/*
rm -rf data/results/*
rm -rf docker/osrm/data/*
rm -rf data/osm_files/*.pbf
rm -rf data/osm_files/fragments/*
echo "[CLEANUP] Removing excess map data..."

echo "[CLEANUP] Cleaning up virtual environment..."
deactivate
echo "[CLEANUP] Shutting down Docker container"
# docker rm -f osrm >/dev/null 2>&1 || true
docker compose down
# rm -rf "$VENV_DIR"

echo "Done :)"
