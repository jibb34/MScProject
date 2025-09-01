#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=segmentation_engine:latest
CONTAINER_NAME=segmentation_container
HOST_PORT=5005

# 1) Build
docker build -t "$IMAGE_NAME" .

# 2) Restart container
docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

# 3) Run
docker run -d \
  --name "$CONTAINER_NAME" \
  -p 5005:5005 \
  "$IMAGE_NAME"

echo "Container '$CONTAINER_NAME' on http://localhost:${HOST_PORT}"
