FROM gcc:13
WORKDIR /app
COPY . .
RUN apt-get update && \
  apt-get install -y libboost-all-dev cmake libyaml-cpp-dev

RUN mkdir build && cd build && cmake .. && make
EXPOSE 5005
CMD ["./build/segmentation-engine"]
