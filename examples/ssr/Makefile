
# Get the absolute path of the Makefile
MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))

# Extract the directory name from the path
MAKEFILE_DIR := $(notdir $(patsubst %/,%,$(dir $(MAKEFILE_PATH))))

##### Settings for the pogobot binary #####
BUILD_DIR=./build
POGO_SDK=../../pogobot-sdk
POGOSIM_INCLUDE_DIR=../../src/
POGOSIM_SRC_DIR=../../src/pogosim/

POGO_SDK_TOOLS=$(POGO_SDK)/tools
POGO_SDK_INCS=$(POGO_SDK)/includes
POGO_SDK_LIBS=$(POGO_SDK)/libs

POGO_VAR=$(POGO_SDK_TOOLS)
# Check if "bin" is one of the goals, or if "make" or "make all" is called
ifneq ($(filter bin all default,$(MAKECMDGOALS)),)
include $(POGO_VAR)/variables.mak
include $(POGO_SDK_TOOLS)/common.mak
endif
# Include files when MAKECMDGOALS is empty (e.g., `make` with no arguments)
ifeq ($(MAKECMDGOALS),)
include $(POGO_VAR)/variables.mak
include $(POGO_SDK_TOOLS)/common.mak
endif

TTY?=/dev/ttyUSB0

#SRCS=$(wildcard *.c) $(wildcard $(POGOSIM_INCLUDE_DIR)/*.c)
#SRCS := $(filter-out %/SDL*.c, $(wildcard *.c) $(wildcard $(POGOSIM_INCLUDE_DIR)/*.c))
SRCS := $(filter-out SDL_FontCache.c, $(notdir $(wildcard *.c)) $(notdir $(wildcard $(POGOSIM_SRC_DIR)/*.c)))
OBJECTS=$(SRCS:.c=.o)
#OBJECTS_BUILD = $(addprefix $(BUILD_DIR)/, $(OBJECTS))
#OBJECTS_BUILD = $(patsubst %.c,build/bin/%.o,$(SRCS))
OBJECTS_BUILD = $(patsubst %.c,build/bin/%.o,$(notdir $(SRCS)))
DEP_FILES=$(patsubst %.c,build/bin/%.d,$(notdir $(SRCS)))


##### Compiler settings for the simulator #####
SIM_CC = cc
SIM_CXX = c++
SIM_CFLAGS = -Wall -MMD -MP -O2 -std=c11
SIM_CXXFLAGS = -Wall -MMD -MP -O2 -std=c++20 `pkg-config --cflags spdlog` -pthread
SIM_LDFLAGS = -Wl,--whole-archive -lpogosim -Wl,--no-whole-archive -lboost_system -lyaml-cpp `pkg-config --libs spdlog` -lSDL2 -lSDL2_gfx -lSDL2_ttf -lbox2d -lstdc++ -lfmt -lm -larrow 
SIM_TARGET = $(MAKEFILE_DIR)

SIM_SRCS_CXX = $(wildcard *.cpp)
SIM_SRCS_C = $(wildcard *.c)
#SIM_OBJECTS_CXX = $(SIM_SRCS_CXX:.cpp=.o)
#SIM_OBJECTS_C = $(SIM_SRCS_C:.c=.o)
SIM_OBJECTS_CXX = $(patsubst %.cpp,build/sim/%.o,$(SIM_SRCS_CXX))
SIM_OBJECTS_C = $(patsubst %.c,build/sim/%.o,$(SIM_SRCS_C))
SIM_OBJECTS = $(SIM_OBJECTS_CXX) $(SIM_OBJECTS_C)


all: directories sim bin

# Ensure build directories exist
$(BUILD_DIR)/bin $(BUILD_DIR)/sim:
	mkdir -p $@

##### Rules for the simulator #####

#%.o: %.cpp
$(BUILD_DIR)/sim/%.o: %.cpp | $(BUILD_DIR)/sim
	$(SIM_CXX) $(SIM_CXXFLAGS) -c $< -o $@

#%.o: %.c
$(BUILD_DIR)/sim/%.o: %.c | $(BUILD_DIR)/sim
	$(SIM_CC) $(SIM_CFLAGS) -c $< -o $@

sim: directories $(SIM_TARGET)

$(SIM_TARGET): $(SIM_OBJECTS)
	$(SIM_CXX) $(SIM_CXXFLAGS) -o $@ $(SIM_OBJECTS) $(SIM_LDFLAGS) #-Wl,--wrap=main
	cp $(SIM_TARGET) $(BUILD_DIR)/sim/$(SIM_TARGET)

##### Rules for the Pogobot binary #####

bin: directories $(BUILD_DIR)/bin/firmware.bin

INCLUDES+=-I. -I$(POGO_SDK_INCS) -I$(POGOSIM_INCLUDE_DIR)

# pull in dependency info for *existing* .o files
-include $(OBJECTS_BUILD:.o=.d) ${SIM_OBJECTS:.o=.d} $(DEP_FILES)

$(BUILD_DIR)/bin/%.bin: $(BUILD_DIR)/bin/%.elf
	$(OBJCOPY) -O binary $< $@
	chmod -x $@

$(BUILD_DIR)/bin/firmware.elf: $(OBJECTS_BUILD)
	$(CC) $(LDFLAGS) \
		-T $(POGO_SDK_TOOLS)/linker.ld \
		-N -o $@ \
		$(OBJECTS_BUILD) \
		-Wl,--whole-archive \
		-Wl,--gc-sections \
		-L$(POGO_SDK_LIBS) -lcompiler_rt -lc -lpogobot
	chmod -x $@


$(BUILD_DIR)/bin/%.o: %.c | $(BUILD_DIR)/bin
	$(compile) -DREAL_ROBOT

$(BUILD_DIR)/bin/%.o: $(POGOSIM_SRC_DIR)/%.c | $(BUILD_DIR)/bin
	$(compile) -DREAL_ROBOT

clean:
	$(RM) -r $(BUILD_DIR) .*~ *~
	$(RM) -f $(SIM_OBJECTS) $(SIM_TARGET)

connect:
	$(POGO_SDK_TOOLS)/litex_term.py --serial-boot --kernel $(BUILD_DIR)/bin/firmware.bin --kernel-adr $(ROM_BASE) --safe $(TTY)

directories: $(BUILD_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY: all clean connect directories


# MODELINE "{{{1
# vim:noexpandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
