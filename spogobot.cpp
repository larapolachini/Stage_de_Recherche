
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>

#include "spogobot.h"

Robot* current_robot;
std::vector<Robot> robots;

void pogobot_init() {
    std::cout << "Pogobot initialized successfully." << std::endl;
}

void pogobot_led_setColor(int r, int g, int b) {
    std::cout << "LED Color set to R:" << r << " G:" << g << " B:" << b << std::endl;
}

void pogobot_motor_set(const char* motor, int speed) {
    std::cout << "Motor " << motor << " set to speed " << speed << std::endl;
}

void msleep(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void pogosim_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args); // Standard printf functionality
    va_end(args);
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
