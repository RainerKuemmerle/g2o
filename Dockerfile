FROM ubuntu:18.04

RUN apt-get update -y && \
    apt-get install -y \
    build-essential \
    cmake \
    wget \
    unzip \
    libeigen3-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libglew-dev && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

ARG NUM_THREADS=1
COPY . /g2o
WORKDIR /g2o

RUN mkdir build && \
    cd build && \
    cmake .. && \
    make install -j${NUM_THREADS} && \
    cd .. && rm -r build


