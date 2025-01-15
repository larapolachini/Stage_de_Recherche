#ifndef POGOBOT_H
#define POGOBOT_H

#include <stdint.h>

// TODO : version of the pogobot lib for pogosim

// Create a macro to specify that we are using a simulator, and not real Pogobots
#define SIMULATOR

// Rename the main function from the robot code. The simulator uses a different main function.
#define main robot_main

// TODO define the entire pogobot API

#define motorL "left"
#define motorR "right"
#define motorFull 100
#define motorStop 0

#ifdef __cplusplus
extern "C" {
#endif

// TODO define all functions of the pogobot API

void pogobot_init(void);
void pogobot_led_setColor(int r, int g, int b);
void pogobot_motor_set(const char* motor, int speed);
void msleep(int milliseconds);
void pogosim_printf(const char* format, ...);

#ifdef __cplusplus
}
#endif

#ifndef __cplusplus
// Define custom printf as the default in files including pogosim.h
#define printf pogosim_printf
#endif


#ifdef __cplusplus
#include <vector>

class Robot{
    // TODO

public:
    void *data;
    void (*user_init)(void);
    void (*user_step)(void);
};


extern Robot* current_robot;
extern std::vector<Robot> robots;
extern int UserdataSize;
extern void* mydata;


void create_robots(void);
#endif


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
