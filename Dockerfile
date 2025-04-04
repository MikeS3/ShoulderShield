# Use the latest Ubuntu Image
FROM ubuntu:22.04

#working directory set to project root
WORKDIR /app

#install build tools and ARM GNU Toolchain
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        g++ \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

