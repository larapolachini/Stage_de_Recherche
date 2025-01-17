
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
//std::chrono::time_point<std::chrono::system_clock> sim_starting_time;
uint64_t sim_starting_time_microseconds;

void init_logger() {
    // Create a console logger with color support
    glogger = spdlog::stdout_color_mt("console");
    glogger->set_level(spdlog::level::info); // Set default log level
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v"); // Set log format
}

/************* time_reference_t *************/ // {{{1

void time_reference_t::reset() {
    enabled = false;
    if (current_robot != nullptr)
        current_robot->register_stop_watch(this);

    start_time = get_current_time_microseconds();
    auto const duration = start_time - sim_starting_time_microseconds;
    hardware_value_at_time_origin = duration;
    elapsed_ms = 0;
}

void time_reference_t::enable() {
    enabled = true;
    start_time = get_current_time_microseconds();
    //glogger->debug("ENABLE!! {}", start_time);
}

void time_reference_t::disable() {
    enabled = false;
    get_elapsed_microseconds();
    //glogger->debug("DISABLE!! {}", start_time);
}

uint32_t time_reference_t::get_elapsed_microseconds() {
    //start_time = get_current_time_microseconds();
    auto const duration = get_current_time_microseconds() - start_time;
    elapsed_ms += duration;
    return elapsed_ms;
}


uint64_t get_current_time_microseconds() {
    // Get the current time in microseconds since epoch
    auto const now = std::chrono::system_clock::now();
    auto const duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());

    // Convert
    uint64_t const microseconds = static_cast<uint64_t const>(duration.count());
    return microseconds;
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

void Robot::launch_user_step() {
    update_time();
    enable_stop_watches();
    user_step();
    disable_stop_watches();
    pogo_ticks++;
}

void Robot::update_time() {
    current_time_microseconds = get_current_time_microseconds() - sim_starting_time_microseconds;
}

void Robot::register_stop_watch(time_reference_t* sw) {
    stop_watches.insert(sw);
}

void Robot::enable_stop_watches() {
    for (auto* sw : stop_watches) {
        sw->enable();
    }
}

void Robot::disable_stop_watches() {
    for (auto* sw : stop_watches) {
        sw->disable();
    }
}

/************* SIMULATED POGOLIB *************/ // {{{1

std::string log_current_robot() {
    std::ostringstream oss;
    oss << "[ROBOT #" << current_robot->id << "] ";
    return oss.str();
}

void pogobot_init() {
    glogger->info("{} Pogobot initialized successfully.", log_current_robot());
}

uint16_t pogobot_helper_getid(void) {
    return current_robot->id;
}

void pogobot_stopwatch_reset(time_reference_t *stopwatch) {
    stopwatch->reset();
}

int32_t pogobot_stopwatch_get_elapsed_microseconds(time_reference_t *stopwatch) {
    return stopwatch->get_elapsed_microseconds();
}

void pogobot_led_setColor(int r, int g, int b) {
    glogger->debug("{} LED Color set to R:{} G:{} B:{}", log_current_robot(), r, g, b);
}

void pogobot_motor_set(const char* motor, int speed) {
    glogger->debug("{} Motor {} set to speed {}", log_current_robot(), motor, speed);
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
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
