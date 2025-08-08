#!/bin/bash
set -e

docker build -t segmentation-engine -f docker/Dockerfile .
docker run --rm -p 5005:5005 -v "$(pwd)/input:/app/input" -v "$(pwd)/logs:/app/logs" segmentation-engine
