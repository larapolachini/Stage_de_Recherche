
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>
#include <sstream>
#include <string>
#include <cstdint>
#include "SDL2_gfxPrimitives.h"

#include "spogobot.h"
#include "robot.h"
#include "pogosim.h"
#include "simulator.h"
#include "version.h"


/************* GLOBALS *************/ // {{{1


uint8_t const _selected_power = 1;

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


void time_reference_t::add_elapsed_microseconds(uint64_t microseconds) {
    elapsed_ms += microseconds;
}


uint64_t get_current_time_microseconds() {
    // Get the current time in microseconds since epoch
    auto const now = std::chrono::system_clock::now();
    auto const duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());

    // Convert
    uint64_t microseconds = static_cast<uint64_t>(duration.count());
    return microseconds;
}



/************* SIMULATED POGOLIB *************/ // {{{1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

std::string log_current_robot() {
    std::ostringstream oss;
    oss << "[ROBOT #" << current_robot->id << "] ";
    return oss.str();
}

void pogobot_init() {
    glogger->debug("{} Pogobot initialized successfully.", log_current_robot());
}


/**
 * ## Infrared communication API Functions
 */


void pogobot_infrared_ll_init( void ) {
    // Do nothing ...
}

void pogobot_infrared_update( void ) {
    // Do nothing ...
}

int pogobot_infrared_message_available( void ) {
    return current_robot->messages.size() > 0;
}

void pogobot_infrared_recover_next_message( message_t *mes ) {
    if (!pogobot_infrared_message_available()) {
        glogger->warn("The function 'pogobot_infrared_recover_next_message( message_t *mes )' should *only* be called if 'pogobot_infrared_message_available()' returns >0.. Ignoring.");
        return;
    }
    message_t m = current_robot->messages.front();
    current_robot->messages.pop();
    memcpy(mes, &m, sizeof(message_t));
}

void pogobot_infrared_clear_message_queue( void ) {
    glogger->warn("Function 'pogobot_infrared_clear_message_queue' is not implemented yet!");
}


void pogobot_infrared_set_power( uint8_t power ) {
    // Do nothing ...
}

uint32_t pogobot_infrared_sendRawLongMessage( message_t *const message ) {
    if ( message->header.payload_length > MAX_PAYLOAD_SIZE_BYTES ) {
        return 1;
    }

    message->header._packet_type = ir_t_user; // User packets have type 16.
    message->header._sender_id = pogobot_helper_getid();
    message->header._receiver_ir_index = 0;

    // Rmq from original pogolib:
    // TODO: this currently emits packet via several IR at the same time (or
    // does it?). What this should to is emit packet several times, via one IR
    // at a time, with field _emitting_power_list reflecting which TX is used.
    // This will be necessary later to have robots detect each other and their
    // relative orientations.

    current_robot->send_to_neighbors(message);
    return 0;
}

uint32_t pogobot_infrared_sendRawShortMessage( ir_direction dir, short_message_t *const message ) {
    if ( message->header.payload_length > MAX_PAYLOAD_SIZE_BYTES ) {
        return 1;
    }

    message->header._packet_type = ir_t_short; // User packets have type 3.

    current_robot->send_to_neighbors(message);
    return 0;
}

uint32_t pogobot_infrared_sendLongMessage_uniSpe( ir_direction dir, uint8_t *message, uint16_t message_size ) {
    message_t m;
    m.header._emitting_power_list =
        _selected_power << ( pogobot_infrared_emitter_width_bits * dir );
    m.header.payload_length = message_size;
    m.header._sender_ir_index = dir;
    memcpy( m.payload, message, message_size );
    return pogobot_infrared_sendRawLongMessage( &m );
}

uint32_t pogobot_infrared_sendLongMessage_omniGen( uint8_t *message, uint16_t message_size ) {
    message_t m;
    m.header._emitting_power_list =
        pogobot_infrared_emitting_power_list(_selected_power, _selected_power, _selected_power, _selected_power);
    m.header.payload_length = message_size;
    m.header._sender_ir_index = ir_all;
    memcpy( m.payload, message, message_size );
    return pogobot_infrared_sendRawLongMessage( &m );
}

uint32_t pogobot_infrared_sendLongMessage_omniSpe( uint8_t *message, uint16_t message_size ) {
    int i = 0;
    int error = 0;
    message_t m;
    m.header.payload_length = message_size;
    memcpy( m.payload, message, message_size );

    for ( i = 0; i < IR_RX_COUNT; i++ ) {
        m.header._emitting_power_list =
            _selected_power << ( pogobot_infrared_emitter_width_bits * i );
        m.header._sender_ir_index = i;
        error += pogobot_infrared_sendRawLongMessage( &m );
    }

    return error;
}

uint32_t pogobot_infrared_sendShortMessage_uni( ir_direction dir, uint8_t *message, uint16_t message_size ) {
    short_message_t m;
    m.header.payload_length = message_size;
    memcpy( m.payload, message, message_size );
    return pogobot_infrared_sendRawShortMessage( dir, &m );
}

uint32_t pogobot_infrared_sendShortMessage_omni( uint8_t *message, uint16_t message_size ) {
    short_message_t m;
    m.header.payload_length = message_size;
    memcpy( m.payload, message, message_size );
    return pogobot_infrared_sendRawShortMessage( ir_all, &m );
}

void pogobot_infrared_reset_receiver_error_counter( void ) {
    // Do nothing ...
}


/**
 * ## RGB LED API
 */

void pogobot_led_setColor(const uint8_t r, const uint8_t g, const uint8_t b) {
    glogger->debug("{} LED Color {} set to R:{} G:{} B:{}", log_current_robot(), 0, r, g, b);
    current_robot->leds[0] = {.r=r, .g=g, .b=b};
}

void pogobot_led_setColors(const uint8_t r, const uint8_t g, const uint8_t b, uint8_t id) {
    glogger->debug("{} LED Color {} set to R:{} G:{} B:{}", log_current_robot(), id, r, g, b);
    current_robot->leds[id] = {.r=r, .g=g, .b=b};
}


/**
 * ## Photosensors API Values
 */

int16_t pogobot_photosensors_read( uint8_t sensor_number ) {
    return simulation->get_current_light_value();
}


/**
 * ## IMU API
 */

void pogobot_imu_read( float *acc, float *gyro ) {
    glogger->warn("Function 'pogobot_imu_read' is not implemented yet!");
}

float pogobot_imu_readTemp( void ) {
    glogger->warn("Function 'pogobot_imu_readTemp' is not implemented yet!");
    return 0.0f;
}



/**
 * ## Battery API
 */
int16_t pogobot_battery_voltage_read( void ) {
    glogger->warn("Function 'pogobot_battery_voltage_read' is not implemented yet!");
    return 0;
}



/**
 * ## Motors API Values
 */
void pogobot_motor_power_set( motor_id motor, uint16_t value ) {
    glogger->debug("{} Motor {} set to speed {}", log_current_robot(), static_cast<uint8_t>(motor), value);
    current_robot->set_motor(motor, value);
}


void pogobot_motor_set ( motor_id motor, uint16_t value ) {
    pogobot_motor_power_set(motor, value);
}

uint32_t pogobot_motor_dir_current_status( void ) {
    glogger->warn("Function 'pogobot_motor_dir_current_status' is not implemented yet!");
    return 0;
}

int8_t pogobot_motor_dir_mem_get( uint8_t *p_directions ) {
    glogger->warn("Function 'pogobot_motor_dir_mem_get' is not implemented yet!");
    return 0;
}

int8_t pogobot_motor_dir_mem_set( uint8_t *p_directions) {
    glogger->warn("Function 'pogobot_motor_dir_mem_set' is not implemented yet!");
    return 0;
}

void pogobot_motor_dir_set( motor_id motor, uint8_t value ) {
    glogger->warn("Function 'pogobot_motor_dir_set' is not implemented yet!");
}

uint8_t pogobot_motor_power_mem_get( uint16_t *p_powers ) {
    glogger->warn("Function 'pogobot_motor_power_mem_get' is not implemented yet!");
    return 0;
}

uint8_t pogobot_motor_power_mem_set( uint16_t *p_powers ) {
    glogger->warn("Function 'pogobot_motor_power_mem_set' is not implemented yet!");
    return 0;
}



/**
 * ## Helper API
 */

uint16_t pogobot_helper_getid(void) {
    return current_robot->id;
}

int16_t pogobot_helper_getRandSeed( void ) {
    // Define the distribution for int16_t range
    std::uniform_int_distribution<int16_t> dis(std::numeric_limits<int16_t>::min(),
                                               std::numeric_limits<int16_t>::max());

    // Generate a random number of type int16_t
    return dis(rnd_gen);
}

void pogobot_helper_print_version( void ) {
    printf(" Pogolib Version : %s\n", POGOLIB_RELEASE_VERSION);
}



/**
 * ## Time API
 */

void pli_timer_sleep_stopwatch_init( void ) {
    // Do nothing ...
}


void pogobot_stopwatch_reset(time_reference_t *stopwatch) {
    stopwatch->reset();
}

int32_t pogobot_stopwatch_lap( time_reference_t *stopwatch ) {
    glogger->warn("Function 'pogobot_stopwatch_lap' is not implemented yet!");
    return 0;
}


int32_t pogobot_stopwatch_get_elapsed_microseconds(time_reference_t *stopwatch) {
    return stopwatch->get_elapsed_microseconds();
}

void pogobot_stopwatch_offset_origin_microseconds( time_reference_t *stopwatch, int32_t microseconds_offset ) {
    glogger->warn("Function 'pogobot_stopwatch_offset_origin_microseconds' is not implemented yet!");
}

void pogobot_timer_init( time_reference_t *timer, int32_t microseconds_to_go ) {
    glogger->warn("Function 'pogobot_timer_init' is not implemented yet!");
}

int32_t pogobot_timer_get_remaining_microseconds( time_reference_t *timer ) {
    glogger->warn("Function 'pogobot_timer_get_remaining_microseconds' is not implemented yet!");
    return 0;
}

bool pogobot_timer_has_expired( time_reference_t *timer ) {
    glogger->warn("Function 'pogobot_timer_has_expired' is not implemented yet!");
    return false;
}

void pogobot_timer_wait_for_expiry( time_reference_t *timer ) {
    glogger->warn("Function 'pogobot_timer_wait_for_expiry' is not implemented yet!");
}

void pogobot_timer_offset_origin_microseconds( time_reference_t *timer, int32_t microseconds_offset ) {
    glogger->warn("Function 'pogobot_timer_offset_origin_microseconds' is not implemented yet!");
}

#pragma GCC diagnostic pop


/************* General API *************/ // {{{1

void msleep(int milliseconds) {
    //std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    glogger->debug("{} Sleeping for {} ms", log_current_robot(), milliseconds);
    if (milliseconds <= 0) return;
    for (auto* sw : current_robot->stop_watches) {
        sw->add_elapsed_microseconds(static_cast<uint64_t>(milliseconds) * 1000);
    }
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
