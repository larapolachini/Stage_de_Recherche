
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>
#include <sstream>
#include <string>
#include <cstdint>
#include "SDL2_gfxPrimitives.h"

#include "robot.h"
#include "spogobot.h"
#include "pogosim.h"
#include "simulator.h"


/************* GLOBALS *************/ // {{{1

Robot* current_robot;
//std::chrono::time_point<std::chrono::system_clock> sim_starting_time;
uint64_t sim_starting_time_microseconds;



/************* SIMULATED ROBOTS *************/ // {{{1

Robot::Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius, b2WorldId worldId, float _msg_success_rate)
        : id(_id), radius(_radius), msg_success_rate(_msg_success_rate) {
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


// Inline function to calculate the normalized color values
inline uint8_t adjust_color(uint8_t const value) {
    if (value == 0) {
        return 0; // Use 0 for value 0
    } else if (value <= 25) {
        // Map values from 1-25 to 100-210
        return static_cast<uint8_t>(100 + (static_cast<float>(value - 1) / 24.0f * 110.0f));
    } else {
        return 210; // Use 210 for values > 25
    }
}

void Robot::render(SDL_Renderer* renderer, [[maybe_unused]] b2WorldId worldId) const {
    // Get robot's position in the physics world
    b2Vec2 position = b2Body_GetPosition(bodyId);

    // Convert to screen coordinates
    float screenX = position.x * VISUALIZATION_SCALE;
    float screenY = position.y * VISUALIZATION_SCALE;

    // Draw circle representing the robot's body
//    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255); // Gray color for robot body
//    SDL_RenderDrawCircle(renderer, static_cast<int>(screenX), static_cast<int>(screenY), radius);
    auto const circle_pos = visualization_position(screenX, screenY);
    circleRGBA(renderer, circle_pos.x, circle_pos.y, radius * mm_to_pixels, 0, 0, 0, 255);

    // Get the robot's orientation as a rotation (cosine/sine pair)
    b2Rot rotation = b2Body_GetRotation(bodyId);
    float cosAngle = rotation.c;
    float sinAngle = rotation.s;

    // Draw line indicating robot orientation with increased width
    float const orientationX = screenX + cosAngle * radius * 2.0; // Increase length of orientation line
    float const orientationY = screenY + sinAngle * radius * 2.0;
//    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 150); // Blue/transparent color for orientation line
//    for (int offset = -2; offset <= 2; ++offset) {
//        SDL_RenderDrawLine(renderer, screenX + offset, screenY, orientationX + offset, orientationY);
//        SDL_RenderDrawLine(renderer, screenX, screenY + offset, orientationX, orientationY + offset);
//    }
    auto const orientation_pos = visualization_position(orientationX, orientationY);
    thickLineRGBA(renderer, circle_pos.x, circle_pos.y, orientation_pos.x, orientation_pos.y, 4, 0, 0, 255, 150);

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
        color_t const& ledColor = leds[i]; // Get LED color
        uint8_t const r = adjust_color(ledColor.r);
        uint8_t const g = adjust_color(ledColor.g);
        uint8_t const b = adjust_color(ledColor.b);

        //// Convert LED color to SDL color
        ////SDL_SetRenderDrawColor(renderer, ledColor.r / 25.0f * 255.0f, ledColor.g / 25.0f * 255.0f, ledColor.b / 25.0f * 255.0f, 255);
        ////SDL_SetRenderDrawColor(renderer, ledColor.r, ledColor.g, ledColor.b, 255);
        //SDL_SetRenderDrawColor(renderer,
        //        ledColor.r > 25 ? 230 : static_cast<float>(ledColor.r) / 25.0f * 210.0f,
        //        ledColor.g > 25 ? 230 : static_cast<float>(ledColor.g) / 25.0f * 210.0f,
        //        ledColor.b > 25 ? 230 : static_cast<float>(ledColor.b) / 25.0f * 210.0f, 255);

        // Calculate screen coordinates for the LED
        float ledScreenX = screenX + rotatedLedOffsets[i].x * 2.0;
        float ledScreenY = screenY + rotatedLedOffsets[i].y * 2.0;

        // Draw LED as a small circle
        auto const led_pos = visualization_position(ledScreenX, ledScreenY);
        if (i == 0) {
            //SDL_RenderDrawCircle(renderer, static_cast<int>(ledScreenX), static_cast<int>(ledScreenY), radius - 2);
            filledCircleRGBA(renderer, led_pos.x, led_pos.y, (radius - 2) * mm_to_pixels, r, g, b, 255);
        } else {
            //SDL_RenderDrawCircle(renderer, static_cast<int>(ledScreenX), static_cast<int>(ledScreenY), radius / 2.5);
            filledCircleRGBA(renderer, led_pos.x, led_pos.y, (radius / 2.5) * mm_to_pixels, r, g, b, 255);
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
    //glogger->debug("set motor: {} {}", left_motor_speed, right_motor_speed);

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
    // Define a uniform real distribution between 0.0 and 1.0
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);

    for (Robot* robot : neighbors) {
        float const prob = dis(rnd_gen);
        //glogger->debug("MESSAGE !! with prob {} / {}: {} -> {}", prob, msg_success_rate, message->header._sender_id, robot->id);
        if (prob <= msg_success_rate && robot->messages.size() < 100) { // XXX Maxsize should be an option
            robot->messages.push(*message);
        }
    }
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
