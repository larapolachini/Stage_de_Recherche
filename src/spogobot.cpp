
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>
#include <sstream>
#include <string>

#include "spogobot.h"
#include "pogosim.h"


/************* GLOBALS *************/ // {{{1

Robot* current_robot;
std::shared_ptr<spdlog::logger> glogger; // XXX Move
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
    bodyDef.linearDamping = 0.0f;
    bodyDef.isBullet = true; // Enable bullet mode
    bodyId = b2CreateBody(worldId, &bodyDef);

    // Create the circle shape
    b2Circle circle;
    circle.center = {0.0f, 0.0f};
    circle.radius = radius / VISUALIZATION_SCALE; // Scaled radius

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.friction = 0.3f;
    shapeDef.restitution = 10.8f; // Bounciness
    shapeDef.enablePreSolveEvents = true; // Enable CCD
    shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);

    // Assign initial velocity
    b2Vec2 velocity = {0.0, 0.0}; // {(std::rand() % 200 - 100) / 20.0f, (std::rand() % 200 - 100) / 20.0f};
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
        SDL_SetRenderDrawColor(renderer, ledColor.r, ledColor.g, ledColor.b, 255);

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



void Robot::set_motor(const char* motor, int speed) {
    // Update motors speed
    if (strcmp(motor, motorL) == 0) {
        left_motor_speed = speed;
    } else if (strcmp(motor, motorR) == 0) {
        right_motor_speed = speed;
    }
    glogger->debug("set motor: {} {}", left_motor_speed, right_motor_speed);

    // Update linear velocity of the agent
    b2Rot const rot = b2Body_GetRotation(bodyId);
    float const v = 1.0f * (left_motor_speed / motorFull + right_motor_speed / motorFull) / 2.0f;
    b2Vec2 const linear_velocity = {rot.c * v, rot.s * v};
    //b2Vec2 const velocity = {left_motor_speed / VISUALIZATION_SCALE, right_motor_speed / VISUALIZATION_SCALE};
    b2Body_SetLinearVelocity(bodyId, linear_velocity);
    //float const angular_velocity = 1.0f / (motorFull * 0.5) * (right_motor_speed - left_motor_speed);
    float const angular_velocity = 1.0f / (motorFull * 0.5) * (left_motor_speed - right_motor_speed);
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

void pogobot_led_setColor(const uint8_t r, const uint8_t g, const uint8_t b) {
    glogger->debug("{} LED Color {} set to R:{} G:{} B:{}", log_current_robot(), 0, r, g, b);
    current_robot->leds[0] = {.r=r, .g=g, .b=b};
}

void pogobot_led_setColors(const uint8_t r, const uint8_t g, const uint8_t b, uint8_t id) {
    glogger->debug("{} LED Color {} set to R:{} G:{} B:{}", log_current_robot(), id, r, g, b);
    current_robot->leds[id] = {.r=r, .g=g, .b=b};
}

void pogobot_motor_set(const char* motor, int speed) {
    glogger->debug("{} Motor {} set to speed {}", log_current_robot(), motor, speed);
    current_robot->set_motor(motor, speed);
}

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
