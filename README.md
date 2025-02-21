# Pogosim ![Badge CI](https://github.com/Adacoma/pogosim/actions/workflows/ci.yaml/badge.svg) ![Version](https://img.shields.io/badge/version-v0.8.3-blue)
Pogosim is a simulator for the [Pogobot robots](https://pogobot.github.io/). It aims to reproduce the C API used on the robots, so that the exact same code can be used in simulations as in robotic experiments.

Pogosim is coded in C++20 and C17, using SDL2 and Box2D 3.0.

## Overview
Here are the simulated runs of several examples (C code found [here](examples)).
![Hanabi with 300 robots in a star-shaped arena](https://github.com/leo-cazenille/pogosim/blob/main/.description/hanabi_300_star.gif)
![SSR with 25 robots in a disk](https://github.com/leo-cazenille/pogosim/blob/main/.description/ssr_disk_25_3min.gif)
![SSR with 25 robots in an annulus](https://github.com/leo-cazenille/pogosim/blob/main/.description/ssr_annulus_25_3min.gif)


## Install on Linux
To install it on *Debian/Ubuntu* (tested: 24.04 LTS), use the following commands. The process will be similar on other Linux distributions.

First, install the necessary packages:
```shell
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    build-essential cmake git libboost-system-dev \
    libsdl2-dev libsdl2-image-dev libsdl2-gfx-dev libsdl2-ttf-dev \
    libyaml-cpp-dev libspdlog-dev \
    wget unzip ca-certificates lsb-release

# Install Apache Arrow
wget https://apache.jfrog.io/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt install -y -V ./apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt update
sudo apt install -y -V libarrow-dev
```

Then compile and install Box2D 3.0:
```shell
git clone https://github.com/erincatto/box2d.git
cd box2d
git checkout 28adacf82377d4113f2ed00586141463244b9d10
mkdir build && cd build
cmake -DBOX2D_BUILD_DOCS=OFF -DGLFW_BUILD_WAYLAND=OFF -DCMAKE_INSTALL_PREFIX=/usr  ..
cmake --build .
sudo make install
cd ../..
```

Clone the pogosim repository, compile pogosim and install it:
```shell
git clone https://github.com/leo-cazenille/pogosim.git
cd pogosim
./build.sh 
```


## Quickstart

Just copy paste one of the example codes from the directory "examples/" to the main directory of your project. E.g. using the Hanabi example as a base:
```shell
cp -R examples/hanabi ~/my_pogobot_project

# Copy a baseline configuration file
mkdir -p ~/my_pogobot_project/conf   # Directory to store the yaml configuration files for this new project
cp conf/test.yaml ~/my_pogobot_project/conf

# Remove the old hanabi binary, if it was present
cd ~/my_pogobot_project
rm hanabi
```

Then to compile a binary for the simulator:
```shell
cd ~/my_pogobot_project
make clean && make -j 10 sim   # Or just "make clean sim" to compile without parallelization 
```

Which can then be launched through:
```shell
./hanabi -c conf/test.yaml
```


### Compile a binary for the real Pogobots
Download the [pogobot-SDK](https://github.com/nekonaute/pogobot-sdk) somewhere:
```shell
git clone https://github.com/nekonaute/pogobot-sdk.git
```

Edit "~/my\_pogobot\_project/Makefile" to set the path of the pogobot-sdk: change the value of variable "POGO\_SDK".

Use the following commands to compile the binary:
```shell
cd ~/my_pogobot_project
make clean && make bin
```

The binary should be compiled correctly, and you can then use the usual commands to upload it to a robot. E.g. through:
```shell
make connect TTY=/dev/ttyUSB0
```
Inside the robot prompt, type "enter" to obtain a new prompt line. After type the command "serialboot" to upload the code. Cf the [pogobot-SDK documentation](https://github.com/nekonaute/pogobot-sdk) for more details.


### Headless mode
To launch your simulation in headless mode (while still exporting png files of the traces), use the "-g" command line parameter. E.g.:
```shell
./my_pogobot_project -c conf/test.yaml -g
```
The simulator is far faster in headless mode than in windowed mode.


### Command line parameters of the simulator
```shell
Usage: pogosim [options]
Options:
  -c, --config <file>             Specify the configuration file.
  -g, --no-GUI                    Disable GUI mode.
  -v, --verbose                   Enable verbose mode.
  -nr, --do-not-show-robot-msg    Suppress robot messages.
  -P, --progress                  Show progress output.
  -V, --version                   Show version information.
  -h, --help                      Display this help message.
```
- Parameter "-c" must always be provided, and corresponds to the YAML configuration file to use. See "conf/test.yaml" for an example.
- Parameter "-g" enables headless mode: no GUI shown, but the program still export frames.
- Parameter "-v" enables verbose mode (show debug messages).
- Parameter "-nr" disables messages from the robots (printf in robot code).
- Parameter "-P" displays a progress bar of the simulation, depending on the parameter value "SimulationTime" defined in the configuration file.


## Install and use the simulator in an Apptainer/Singularity container

To build the image:
```shell
sudo apptainer build --sandbox -F pogosim.simg pogosim-apptainer.def
```
Or, if you want to use Clang instead of GCC:
```shell
sudo apptainer build --sandbox -F --build-arg USE_CLANG=true pogosim.simg pogosim-apptainer.def
```


Use the image to compile a pogosim project:
```shell
cd ~/my_pogobot_project
apptainer exec /PATH/TO/pogosim.simg make clean sim
```
Note that your current directory should be a subpath of your home (~) directory -- elsewise apptainer/singularity cannot access it by default.

Then the simulator can be launched with:
```shell
apptainer exec /PATH/TO/pogosim.simg ./my_pogobot_project -c conf/test.yaml
```


## Generate gif files of the traces
By default, the frames of a simulated run are stored in the directory "frames/" (cf variable "frames\_name" in the configuration file).
They can be assembled into an animated gif file using various commands, such as mencoder, ffmpeg, or ImageMagick.
We recommand the program [gifski](https://gif.ski/), a very high-quality GIF encoder:
```shell
gifski -r 20 --output animation.gif frames/*png
```


## Development

If you want to compile the pogosim library with debugging symbols and options (e.g. -Og -g compilation parameters), you can specify the configuration Debug to the build script:
```shell
./build.sh Debug
```


## Authors

 * Leo Cazenille: Main author and maintainer.
    * email: leo "dot" cazenille "at" gmail "dot" com
 * Nicolas Bredeche
    * email: nicolas "dot" bredeche "at" sorbonne-universite "dot" fr


## Citing

```bibtex
@misc{qdpy,
    title = {pogosim: A simulator for Pogobot robots},
    author = {Cazenille, L., Bredeche, N.},
    year = {2025},
    publisher = {Github},
    journal = {GitHub repository},
    howpublished = {\url{https://github.com/Adacoma/pogosim}},
}
```


