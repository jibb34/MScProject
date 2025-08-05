FROM ubuntu:20.04

RUN apt update && apt install -y \
  build-essential cmake git libboost-all-dev \
  liblua5.2-dev libtbb-dev libstxxl-dev libosmpbf-dev \
  libprotobuf-dev protobuf-compiler libbz2-dev zlib1g-dev libgomp1

RUN git clone https://github.com/YOUR_USERNAME/osrm-backend.git \
  && cd osrm-backend \
  && mkdir -p build && cd build \
  && cmake .. \
  && cmake --build .

WORKDIR /data
ENTRYPOINT ["/osrm-backend/build/osrm-routed"]
