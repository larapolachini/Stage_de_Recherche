#!/bin/bash
config=${1:-"Release"}

# Check if sudo is available
if command -v sudo &> /dev/null; then
    SUDO="sudo"
else
    SUDO=""
fi

cmake -S . -B build -DCMAKE_BUILD_TYPE=$config
cmake --build build -- -j$(nproc) && $SUDO cmake --build build --target install && cmake --build build --target examples -- -j$(nproc)

