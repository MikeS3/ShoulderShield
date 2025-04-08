# Use the latest Ubuntu Image
FROM ubuntu:22.04

# Working directory set to project root
WORKDIR /app

# Install build tools and ARM GNU Toolchain
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        g++ \
        gcc-arm-none-eabi \
        binutils-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        git \
        python3 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Verify ARM toolchain is installed
RUN arm-none-eabi-gcc --version && \
    arm-none-eabi-as --version

# Set SDK path as seen from the source dir (src/)
ENV PICO_SDK_PATH=../lib/pico-sdk

# Add environment variables to ensure ARM assembler is used
ENV PICO_GCC_TRIPLE=arm-none-eabi
