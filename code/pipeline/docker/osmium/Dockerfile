FROM ubuntu:22.04

# Install required tools
RUN apt-get update && \
  apt-get install -y software-properties-common && \
  add-apt-repository ppa:osmadmins/ppa && \
  apt-get update && \
  apt-get install -y osmium-tool && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /data
