# --- builder ---
FROM ubuntu:20.04 AS builder
WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && \
  apt-get install -y --no-install-recommends tzdata && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
  dpkg-reconfigure -f noninteractive tzdata && \
  apt-get install -y --no-install-recommends \
  cmake \
  build-essential \
  libfftw3-dev \
  libeigen3-dev \
  pkg-config \
  clangd \
  ninja-build \
  && rm -rf /var/lib/apt/lists/*

COPY . .

RUN cmake -G Ninja \
  -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
  cmake --build build -j"$(nproc)" && \
  cp build/compile_commands.json .


# --- runtime ---
FROM ubuntu:20.04
WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && \
  apt-get install -y --no-install-recommends tzdata && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
  dpkg-reconfigure -f noninteractive tzdata && \
  apt-get install -y --no-install-recommends \
  libfftw3-3 \
  && rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/build/segmentation_engine /usr/local/bin/segmentation_engine
COPY --from=builder /app/compile_commands.json /app/compile_commands.json
COPY config/ /app/config/
COPY src/javascript/ /app/public/

EXPOSE 5005
ENTRYPOINT ["/usr/local/bin/segmentation_engine"]


