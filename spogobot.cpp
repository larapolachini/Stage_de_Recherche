
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>
#include <sstream>


#include "spogobot.h"

/************* GLOBALS *************/ // {{{1

Robot* current_robot;
std::vector<Robot> robots;
uint64_t pogo_ticks;
std::shared_ptr<spdlog::logger> glogger;

void init_logger() {
    // Create a console logger with color support
    glogger = spdlog::stdout_color_mt("console");
    glogger->set_level(spdlog::level::info); // Set default log level
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v"); // Set log format
}

/************* SIMULATED ROBOTS *************/ // {{{1

Robot::Robot(uint16_t _id, size_t _userdatasize) {
    id = _id;
    data = malloc(_userdatasize);
}

//Robot::~Robot() {
//    free(data);
//    data = nullptr;
//}


/************* SIMULATED POGOLIB *************/ // {{{1

std::string log_current_robot() {
    std::ostringstream oss;
    oss << "[ROBOT #" << current_robot->id << "] ";
    return oss.str();
}

void pogobot_init() {
    glogger->info(log_current_robot() + "Pogobot initialized successfully.");
}

uint16_t pogobot_helper_getid(void) {
    return current_robot->id;
}

void pogobot_stopwatch_reset(time_reference_t *stopwatch) {
    // TODO
}

int32_t pogobot_stopwatch_get_elapsed_microseconds(time_reference_t *stopwatch) {
    // TODO
    return 0;
}

void pogobot_led_setColor(int r, int g, int b) {
    glogger->debug(log_current_robot() + "LED Color set to R:{} G:{} B:{}", r, g, b);
}

void pogobot_motor_set(const char* motor, int speed) {
    glogger->debug(log_current_robot() + "Motor {} set to speed {}", motor, speed);
}

void msleep(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Helper function to convert va_list arguments to a string
std::string _format_args_to_string(const char* format, va_list args) {
    va_list args_copy;
    va_copy(args_copy, args); // Copy the va_list to avoid modifying the original

    // Use vsnprintf to determine the size of the formatted string
    int size = std::vsnprintf(nullptr, 0, format, args_copy);
    va_end(args_copy);

    if (size < 0) {
        throw std::runtime_error("Error formatting string.");
    }

    // Create a string with the exact size needed
    std::string result(size, '\0');
    std::vsnprintf(&result[0], size + 1, format, args); // Format into the string

    return result;
}

void pogosim_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);

    // Convert va_list arguments to a formatted string
    std::string formatted_message = _format_args_to_string(format, args);

    // Combine the robot log prefix and the message
    std::string final_message = log_current_robot() + "[PRINTF] " + formatted_message;

    // Remove trailing newline, if present
    if (!final_message.empty() && final_message.back() == '\n') {
        final_message.pop_back();
    }

    // Log to glogger->info()
    glogger->info(final_message);

    va_end(args);

//    // Use fmt::vformat to create a formatted string dynamically
//    auto aa = fmt::make_format_args(args);
//    //std::string formatted_message = fmt::vformat(format, fmt::make_format_args(args));
//
// //   // Combine the robot log prefix and the message
// //   std::string final_message = log_current_robot() + "[PRINTF] " + formatted_message;
//
// //   // Log to glogger->info()
////    glogger->info(final_message);
//
    va_end(args);
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
