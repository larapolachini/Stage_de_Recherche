#!/bin/bash
config=${1:-"Release"}

cmake -S . -B build -DCMAKE_BUILD_TYPE=$config
cmake --build build -- -j$(nproc) && sudo cmake --build build --target install && cmake --build build --target examples -- -j$(nproc)

