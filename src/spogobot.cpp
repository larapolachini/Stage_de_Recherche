
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>
#include <sstream>
#include <string>
#include <cstdint>

#include "spogobot.h"
#include "pogosim.h"
#include "simulator.h"


/************* GLOBALS *************/ // {{{1

Robot* current_robot;
//std::chrono::time_point<std::chrono::system_clock> sim_starting_time;
uint64_t sim_starting_time_microseconds;

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


/************* SIMULATED ROBOTS *************/ // {{{1

Robot::Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius, b2WorldId worldId)
        : id(_id), radius(_radius) {
    data = malloc(_userdatasize);
    create_body(worldId, x, y);
}

//Robot::~Robot() {
//    free(data);
//    data = nullptr;
//}


void Robot::create_body(b2WorldId worldId, float x, float y) {
    // Create the body
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = {x / VISUALIZATION_SCALE, y / VISUALIZATION_SCALE};
    bodyDef.linearDamping = 1000.0f;
    bodyDef.angularDamping = 1000.0f;
    bodyDef.isBullet = false; // Enable bullet mode
    bodyId = b2CreateBody(worldId, &bodyDef);

    // Create the circle shape
    b2Circle circle;
    circle.center = {0.0f, 0.0f};
    circle.radius = radius / VISUALIZATION_SCALE; // Scaled radius

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 10.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 0.5f; // Bounciness
    shapeDef.enablePreSolveEvents = true; // Enable CCD
    shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);

    // Assign initial velocity
    //b2Vec2 velocity = {0.0, 0.0}; // {(std::rand() % 200 - 100) / 20.0f, (std::rand() % 200 - 100) / 20.0f};
    b2Vec2 velocity = {1.0, 1.0}; // {(std::rand() % 200 - 100) / 20.0f, (std::rand() % 200 - 100) / 20.0f};
    b2Body_SetLinearVelocity(bodyId, velocity);
}


void Robot::render(SDL_Renderer* renderer, [[maybe_unused]] b2WorldId worldId) const {
    // Get robot's position in the physics world
    b2Vec2 position = b2Body_GetPosition(bodyId);

    // Convert to screen coordinates
    float screenX = position.x * VISUALIZATION_SCALE;
    float screenY = position.y * VISUALIZATION_SCALE;

    // Draw circle representing the robot's body
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255); // Gray color for robot body
    SDL_RenderDrawCircle(renderer, static_cast<int>(screenX), static_cast<int>(screenY), radius);

    // Get the robot's orientation as a rotation (cosine/sine pair)
    b2Rot rotation = b2Body_GetRotation(bodyId);
    float cosAngle = rotation.c;
    float sinAngle = rotation.s;

    // Draw line indicating robot orientation with increased width
    float orientationX = screenX + cosAngle * radius * 2.0; // Increase length of orientation line
    float orientationY = screenY + sinAngle * radius * 2.0;
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 150); // Blue/transparent color for orientation line
    for (int offset = -2; offset <= 2; ++offset) {
        SDL_RenderDrawLine(renderer, screenX + offset, screenY, orientationX + offset, orientationY);
        SDL_RenderDrawLine(renderer, screenX, screenY + offset, orientationX, orientationY + offset);
    }

    // Define relative positions for LEDs around the robot based on orientation
    std::vector<b2Vec2> ledOffsets = {
        {0, 0},                        // Above the robot center
        {0, -radius / 2},              // Up
        {radius / 2, 0},               // Right
        {0, radius / 2},               // Down
        {-radius / 2, 0}               // Left
    };

    // Rotate LED offsets based on robot orientation
    std::vector<b2Vec2> rotatedLedOffsets;
    for (const auto& offset : ledOffsets) {
        float rotatedX = cosAngle * offset.x - sinAngle * offset.y;
        float rotatedY = sinAngle * offset.x + cosAngle * offset.y;
        rotatedLedOffsets.push_back({rotatedX, rotatedY});
    }

    // Draw each LED
    for (size_t i = 0; i < leds.size() && i < rotatedLedOffsets.size(); ++i) {
        const color_t& ledColor = leds[i]; // Get LED color

        // Convert LED color to SDL color
        //SDL_SetRenderDrawColor(renderer, ledColor.r / 25.0f * 255.0f, ledColor.g / 25.0f * 255.0f, ledColor.b / 25.0f * 255.0f, 255);
        //SDL_SetRenderDrawColor(renderer, ledColor.r, ledColor.g, ledColor.b, 255);
        SDL_SetRenderDrawColor(renderer,
                ledColor.r > 25 ? 255 : static_cast<float>(ledColor.r) / 25.0f * 255.0f,
                ledColor.g > 25 ? 255 : static_cast<float>(ledColor.g) / 25.0f * 255.0f,
                ledColor.b > 25 ? 255 : static_cast<float>(ledColor.b) / 25.0f * 255.0f, 255);

        // Calculate screen coordinates for the LED
        float ledScreenX = screenX + rotatedLedOffsets[i].x * 2.0;
        float ledScreenY = screenY + rotatedLedOffsets[i].y * 2.0;

        // Draw LED as a small circle
        if (i == 0) {
            SDL_RenderDrawCircle(renderer, static_cast<int>(ledScreenX), static_cast<int>(ledScreenY), radius - 2);
        } else {
            SDL_RenderDrawCircle(renderer, static_cast<int>(ledScreenX), static_cast<int>(ledScreenY), radius / 2.5);
        }
    }
}



void Robot::set_motor(motor_id motor, int speed) {
    // Update motors speed
    if (motor == motorL) {
        left_motor_speed = speed;
    } else if (motor == motorR) {
        right_motor_speed = speed;
    }
    glogger->debug("set motor: {} {}", left_motor_speed, right_motor_speed);

    // No damping
    b2Body_SetLinearDamping(bodyId, 0.0);
    b2Body_SetAngularDamping(bodyId, 0.0);

    // Update linear velocity of the agent
    b2Rot const rot = b2Body_GetRotation(bodyId);
    float const v = 1.0f * (left_motor_speed / static_cast<float>(motorFull) + right_motor_speed / static_cast<float>(motorFull)) / 2.0f;
    b2Vec2 const linear_velocity = {rot.c * v, rot.s * v};
    //b2Vec2 const velocity = {left_motor_speed / VISUALIZATION_SCALE, right_motor_speed / VISUALIZATION_SCALE};
    b2Body_SetLinearVelocity(bodyId, linear_velocity);
    //float const angular_velocity = 1.0f / (motorFull * 0.5) * (right_motor_speed - left_motor_speed);
    float const angular_velocity = 1.0f / (static_cast<float>(motorFull) * 0.5) * (left_motor_speed - right_motor_speed);
    b2Body_SetAngularVelocity(bodyId, angular_velocity);
}


void Robot::launch_user_step() {
    update_time();
    enable_stop_watches();
    //user_step();
    pogo_main_loop_step(user_step);
    disable_stop_watches();
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

b2Vec2 Robot::get_position() const {
    return b2Body_GetPosition(bodyId);
}


void Robot::send_to_neighbors(short_message_t *const message) {
    // Reconstruct a long message from the short message
    message_t m; 
    m.header._packet_type = message->header._packet_type;
    m.header._emitting_power_list = 0; // power all to 0 shouldn't emit something
    m.header._sender_id = 0xFF; 
    m.header._sender_ir_index = 0xF; // index not possible
    m.header._receiver_ir_index = 0;
    m.header.payload_length = message->header.payload_length;
    memcpy( m.payload, message->payload, m.header.payload_length);

    send_to_neighbors(&m);
}

void Robot::send_to_neighbors(message_t *const message) {
    for (Robot* robot : neighbors) {
        //glogger->debug("MESSAGE !! {} -> {}", message->header._sender_id, robot->id);
        if (robot->messages.size() < 100) { // XXX Maxsize should be an option
            robot->messages.push(*message);
        }
    }
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
    glogger->warn("Function 'pogobot_motor_power_set' is not implemented yet!");
}


void pogobot_motor_set ( motor_id motor, uint16_t value ) {
    glogger->debug("{} Motor {} set to speed {}", log_current_robot(), static_cast<uint8_t>(motor), value);
    current_robot->set_motor(motor, value);
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
    glogger->warn("Function 'pogobot_helper_print_version' is not implemented yet!");
}



/**
 * ## Time API
 */

void pli_timer_sleep_stopwatch_init( void ) {
    glogger->warn("Function 'pli_timer_sleep_stopwatch_init' is not implemented yet!");
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
