#!/bin/bash

# Set default build type if not provided
config=${1:-"Release"}

# Detect OS
OS=$(uname)
if [[ "$OS" == "Darwin" ]]; then
    NUM_CORES=$(sysctl -n hw.ncpu)  # macOS
else
    NUM_CORES=$(nproc)  # Linux
fi

# Check if sudo is available
if command -v sudo &> /dev/null; then
    SUDO="sudo"
else
    SUDO=""
fi

# Configure CMake
cmake -S . -B build -DCMAKE_BUILD_TYPE=$config

# Build project
cmake --build build -- -j$NUM_CORES && $SUDO cmake --install build && cmake --build build --target examples -- -j$NUM_CORES

