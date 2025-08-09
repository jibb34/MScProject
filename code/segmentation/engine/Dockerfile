FROM ubuntu:20.04
WORKDIR /app

# Non-interactive apt setup and install build tools
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
RUN apt-get update && \
  apt-get install -y --no-install-recommends cmake build-essential git && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

# Clean, configure, build
RUN rm -rf /app/build && \
  cmake -S /app -B /app/build -DCMAKE_BUILD_TYPE=Release && \
  cmake --build /app/build -j"$(nproc)"

EXPOSE 5005
# Run the segmentation engine
ENTRYPOINT ["/app/build/segmentation_engine"]
