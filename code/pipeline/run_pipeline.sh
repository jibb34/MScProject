#!/bin/bash

VENV_DIR=".venv"
REQ_FILE="requirements.txt"
MAIN_SCRIPT="main.py"
DEBUG=0

if [ $# -lt 2 ]; then
  echo "[ERROR] Usage: $0 <gpx_file> <pbf_output> [--debug]"
  exit 1
fi

GPX_FILE="$1"
PBF_OUTPUT="$2"
shift 2

for arg in "$@"; do
  if [ "$arg" == "--debug" ]; then
    DEBUG=1
  fi
done

echo "Starting pipeline..."
echo "GPX file: $GPX_FILE"
echo "PBF output file: $PBF_OUTPUT"

if [ ! -d "$VENV_DIR" ]; then
  echo "Creating virtual environment..."
  python3 -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"

echo "Installing Python dependencies..."
pip install --quiet -r "$REQ_FILE"

# Launch Docker
echo "Launching OSRM Container"
docker-compose -f "$DOCKER_COMPOSE_FILE" up -d
echo "Waiting for OSRM server to initialize"
sleep 5

echo "Running pipeline..."
if [ "$DEBUG" -eq 1 ]; then
  python "$MAIN_SCRIPT" "$GPX_FILE" "$PBF_OUTPUT" --debug
else
  python "$MAIN_SCRIPT" "$GPX_FILE" "$PBF_OUTPUT"
fi

echo "Cleaning up virtual environment..."
deactivate
docker-compose -f "$DOCKER_COMPOSE_FILE" down
rm -rf "$VENV_DIR"

echo "Done :)"
