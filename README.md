# Pogosim ![Badge CI](https://github.com/Adacoma/pogosim/actions/workflows/ci.yaml/badge.svg) ![Version](https://img.shields.io/badge/version-v0.8.5-blue)
Pogosim is a simulator for the [Pogobot robots](https://pogobot.github.io/). It aims to reproduce the C API used on the robots, so that the exact same code can be used in simulations as in robotic experiments.

Pogosim is coded in C++20 and C17, using SDL2 and Box2D 3.0.

## Overview
Here are the simulated runs of several examples (C code found [here](examples)).
![Hanabi with 300 robots in a star-shaped arena](https://github.com/Adacoma/pogosim/blob/main/.description/hanabi_300_star.gif)
![run-and-tumble with 150 robots in a 8-shaped arena](https://github.com/Adacoma/pogosim/blob/main/.description/run_and_tumble_150_8.gif)
![SSR with 25 robots in a disk](https://github.com/Adacoma/pogosim/blob/main/.description/ssr_disk_25_3min.gif)
![SSR with 25 robots in an annulus](https://github.com/Adacoma/pogosim/blob/main/.description/ssr_annulus_25_3min.gif)


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
git clone https://github.com/Adacoma/pogosim.git
cd pogosim
./build.sh 
```

## Install on MacOSX
The installation requires brew to be installed on your computer. Cf: TODO.

You can then install the necessary packages to compile Pogosim:
```shell
brew install cmake boost sdl2 sdl2_image sdl2_gfx sdl2_ttf yaml-cpp spdlog box2d apache-arrow pkg-config
```

Install Box2D 3.0.0:
```shell
```




### Launch example codes
Example codes are compiled every time you launch the "./build.sh" script, alongside the rest of the Pogosim code.

To launch examples code you can use the following commands:
```shell
./examples/helloworld/helloworld -c conf/test.yaml      # Hello world, just robots rotating left then right. The first robot prints "HELLO WORLD !" messages
./examples/run_and_tumble/run_and_tumble -c conf/test.yaml      # A very simple implementation of the run-and-tumble algorithm for locomotion
./examples/hanabi/hanabi -c conf/test.yaml      # A simple code to showcase the diffusion of information in a swarm. Immobile robots by default (uncomment "MOVING_ROBOTS" to make then move)
./examples/ssr/ssr -c conf/ssr.yaml         # More complex example. "Simple" implementation of the SSR algorithm from https://arxiv.org/abs/2403.17147  You can test it for a disk and annulus arena (see conf/ssr.yaml to change the arena).
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


### Controlling the GUI
Here is a list of shortcuts that can be used to control the GUI:
 - F1: Help message
 - F3: Slow down the simulation
 - F4: Speed up the simulation
 - ESC: quit the simulation
 - SPACE: pause the simulation
 - DOWN, UP, LEFT, RIGHT: move the visualisation coordinates
 - Right-Click + Mouse move: move the visualisation coordinates
 - PLUS, MINUS or Mouse Wheel: Zoom up or down
 - 0: Reset the zoom and visualization coordinates


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
Inside the robot prompt, type "enter" to obtain a new prompt line. 
If you connect to the robot through a Progboard, you can use the command "serialboot" to upload the code. Cf the [pogobot-SDK documentation](https://github.com/nekonaute/pogobot-sdk) for more details.
If you use the IR remote device, follow the instructions described [here](https://github.com/nekonaute/pogobot/blob/main/readme-irRemote.md).



### Simple way to create a new pogobot/pogosim project
Just copy the directory "template\_prj":
```shell
cp -R template_prj ~/my_new_pogo_prj
```

*Option 1*: Create simlinks to the necessary libraries:
```shell
cd ~/my_new_pogo_prj
ln -s PATH/TO/pogosim       # https://github.com/Adacoma/pogosim
ln -s PATH/TO/pogobot-sdk   # https://github.com/nekonaute/pogobot-sdk
ln -s PATH/TO/pogo-utils    # If you use pogo-utils in your project. https://github.com/Adacoma/pogo-utils
```

*Option 2*: Set environment variables to link to necessary libraries:
Add the following lines in the configuration file of your shell (e.g. ~/.bashrc for BASH or ~/.zshrc for ZSH):
```shell
export POGO_SDK=/ABSOLUTE/PATH/TO/pogobot-sdk
export POGOSIM_INCLUDE_DIR=/ABSOLUTE/PATH/TO/pogosim/src
export POGOUTILS_INCLUDE_DIR=/ABSOLUTE/PATH/TO/pogo-utils/src
```

*Option 3*: Edit the Makefile so that the following variables contain the paths to the necessary libraries:
```shell
POGO_SDK?=PATH/TO/pogobot-sdk
POGOSIM_INCLUDE_DIR?=PATH/TO/pogosim/src
POGOUTILS_INCLUDE_DIR?=PATH/TO/pogo-utils/src
```


After using any of these 3 options, you can compile the project:
```shell
make clean sim  # To compile the simulation
# OR
make clean bin  # To compile the binary for real Pogobots
# OR
make clean all  # To compile both the simulation and Pogobot binaries
```

By default, the name of the created simulation binary corresponds to the name of the parent directory of the project. You can then launch it using:
```shell
make clean sim
./template_prj -c conf/test.yaml        # If the parent directory is "template_prj"
```


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


### Access the pose and states of the robots in Python
After a simulation is executed, it can periodically store the pose (position and orientation) and internal states of each robot into a data file.
This feature can be enabled in the configuration file, with entries:
```yaml
enable_data_logging: true                   # Set to true to enable the generation of a data file
data_filename: "frames/data.feather"        # Path of the generated data file
save_data_period: 1.0       # In s          # Save data every 1.0 second
```

The data is stored as an Apache Arrow Feather file, a standard and convenient format to store large dataframes.
As such, it can easily be imported in Python by using Pandas:
```yaml
import pandas as pd
df = pd.read_feather("frames/data.feather")
print(df)

     time  robot_id  pogobot_ticks          x          y     angle
0    1.00         0             64   6.130253   8.420820 -0.999992
1    1.00         1             64   5.741853   0.787406  1.609105
2    1.00         2             64   6.976273  10.846928  1.205084
3    1.00         3             64   2.494493   9.393031  1.194923
4    1.00         4             64  10.689711   7.483701  1.821597
..    ...       ...            ...        ...        ...       ...
895  6.05       145            380   2.619418   1.203837  3.132243
896  6.05       146            380   5.100541   2.913250  0.923802
897  6.05       147            380   6.158785  10.048936  2.966942
898  6.05       148            380   6.318808   0.312880 -2.399829
899  6.05       149            379   5.509576   7.346396 -0.368371

[900 rows x 6 columns]
```

Custom columns can be added into this file by using the callback mechanism. See examples "hanabi" (simple) and "ssr" (complex) for more information.



## Install and use the simulator in an Apptainer/Singularity container
The main image definition file for apptainer is based on Ubuntu 24.04 LTS ("pogosim-apptainer.def"). An alternative image based on Ubuntu 22.04 LTS can also be found ("pogosim-apptainer\_ubuntu22.04.def").

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
We recommend the program [gifski](https://gif.ski/), a very high-quality GIF encoder:
```shell
gifski -r 20 --output animation.gif frames/*png
```


## Development

If you want to compile the pogosim library with debugging symbols and options (e.g. -Og -g compilation parameters), you can specify the configuration Debug to the build script:
```shell
./build.sh Debug
```

To generate Doxygen documentation:
```shell
doxygen
```
you can then open "html/index.html".
To compile the latex report:
```shell
cd latex
make
```
This will generate a PDF report named "latex/refman.pdf".

## Authors

 * Leo Cazenille: Main author and maintainer.
    * email: leo "dot" cazenille "at" gmail "dot" com
 * Nicolas Bredeche
    * email: nicolas "dot" bredeche "at" sorbonne-universite "dot" fr


## Citing

```bibtex
@misc{pogosim,
    title = {pogosim: A simulator for Pogobot robots},
    author = {Cazenille, L., Bredeche, N.},
    year = {2025},
    publisher = {Github},
    journal = {GitHub repository},
    howpublished = {\url{https://github.com/Adacoma/pogosim}},
}
```


