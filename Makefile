# Compiler settings
CC = gcc
CXX = g++
CFLAGS = -Wall -std=c11
CXXFLAGS = -Wall -std=c++17
LDFLAGS = -lyaml-cpp
TARGET = pogosim

# Object files
OBJS = simulator.o helloworld.o spogobot.o pogosim.o

# Default target
all: $(TARGET)

# Linking the final executable
$(TARGET): $(OBJS)
#	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)  -Wl,-e,start
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS) #-Wl,--wrap=main

helloworld.o: helloworld.c spogobot.h
	$(CC) $(CFLAGS) -c helloworld.c

simulator.o: simulator.cpp spogobot.h
	$(CXX) $(CXXFLAGS) -c simulator.cpp

spogobot.o: spogobot.cpp spogobot.h
	$(CXX) $(CXXFLAGS) -c spogobot.cpp

pogosim.o: pogosim.cpp pogosim.h
	$(CXX) $(CXXFLAGS) -c pogosim.cpp

# Clean build artifacts
clean:
	rm -f $(OBJS) $(TARGET)

# MODELINE "{{{1
# vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
