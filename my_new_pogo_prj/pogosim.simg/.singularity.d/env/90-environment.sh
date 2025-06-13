#!/bin/sh
# Copyright (c) Contributors to the Apptainer project, established as
#   Apptainer a Series of LF Projects LLC.
#   For website terms of use, trademark policy, privacy policy and other
#   project policies see https://lfprojects.org/policies
# Copyright (c) 2018-2021, Sylabs Inc. All rights reserved.
# This software is licensed under a 3-clause BSD license. Please consult
# https://github.com/apptainer/apptainer/blob/main/LICENSE.md regarding your
# rights to use or distribute this software.

# Custom environment shell code should follow

    export LC_ALL=C.UTF-8
    export LANG=C.UTF-8
    export DEBIAN_FRONTEND=noninteractive
    export CMAKE_MODULE_PATH="/opt/pogosim/cmake/modules"
    export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
    export PATH="/usr/local/bin:$PATH:/pogobot-sdk/tools/riscv64-unknown-elf-gcc-10.1.0-2020.08.2-x86_64-linux-ubuntu14/bin"
    export NPROC=$(  grep -i "^processor" /proc/cpuinfo | wc -l )


