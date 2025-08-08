#!/usr/bin/env bash
set -e
IMAGE_NAME=segmentation_engine:latest
CONTAINER_NAME=segmentation_container

cd build && cmake .. && cd ..

# Build Docker image
docker build -t $IMAGE_NAME .

# Stop & remove existing container, if any
if [ $(docker ps -aq -f name=$CONTAINER_NAME) ]; then
  docker rm -f $CONTAINER_NAME || true
fi

# Run new container
docker run -d -p 5005:5005 --name $CONTAINER_NAME $IMAGE_NAME

echo "Container '$CONTAINER_NAME' running on http://localhost:5005"
