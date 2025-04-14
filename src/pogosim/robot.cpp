
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdarg>
#include <sstream>
#include <string>
#include <cstdint>
#include <cmath>
#include "SDL2_gfxPrimitives.h"

#include "robot.h"
#include "spogobot.h"
#include "pogosim.h"
#include "simulator.h"
#include "colormaps.h"


/************* GLOBALS *************/ // {{{1

//Robot* current_robot;
PogobotObject* current_robot;

//std::chrono::time_point<std::chrono::system_clock> sim_starting_time;
uint64_t sim_starting_time_microseconds;


/************* MsgSuccessRate *************/ // {{{1

DynamicMsgSuccessRate::DynamicMsgSuccessRate(double alpha, double beta, double gamma, double delta)
    : alpha_(alpha), beta_(beta), gamma_(gamma), delta_(delta) {}

double DynamicMsgSuccessRate::operator()(double msg_size, double p_send, double cluster_size) const {
    return 1.0 / (1.0 + (alpha_ * std::pow(msg_size, beta_) *
                                  std::pow(p_send, gamma_) *
                                  std::pow(cluster_size, delta_)));
}

ConstMsgSuccessRate::ConstMsgSuccessRate(double value)
    : const_value_(value) {}

double ConstMsgSuccessRate::operator()(double /*msg_size*/, double /*p_send*/, double /*cluster_size*/) const {
    return const_value_;
}


MsgSuccessRate* msg_success_rate_factory(Configuration const& config) {
    std::string type = to_lowercase(config["type"].get(std::string("realistic")));
    if (type == "realistic") {
        float const alpha  = 0.000004f;
        float const beta   = 2.8096f;
        float const gamma  = 2.3807f;
        float const delta  = 1.2457f;
        return new DynamicMsgSuccessRate(alpha, beta, gamma, delta);
    } else if (type == "static") {
        float const rate = config["rate"].get(0.9);
        return new ConstMsgSuccessRate(rate);
    } else if (type == "dynamic") {
        float const alpha  = config["alpha"].get(0.000004f);
        float const beta   = config["beta"].get(2.8096f);
        float const gamma  = config["gamma"].get(2.3807f);
        float const delta  = config["delta"].get(1.2457f);
        return new DynamicMsgSuccessRate(alpha, beta, gamma, delta);
    } else {
        throw std::runtime_error("Unknown msg_success_rate type '" + type + "'.");
    }
}


/************* SIMULATED ROBOTS *************/ // {{{1

Robot::Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius,
             b2WorldId worldId, std::unique_ptr<MsgSuccessRate> _msg_success_rate,
             float _linearDamping, float _angularDamping,
             float _density, float _friction, float _restitution,
             ShapeType _shapeType, float _yRadius,
             const std::vector<b2Vec2>& _polygonVertices,
             float _linear_noise_stddev, float _angular_noise_stddev,
             float _temporal_noise_stddev)
    : id(_id), radius(_radius), msg_success_rate(std::move(_msg_success_rate)), linearDamping(_linearDamping), angularDamping(_angularDamping),
        linear_noise_stddev(_linear_noise_stddev), angular_noise_stddev(_angular_noise_stddev), temporal_noise_stddev(_temporal_noise_stddev) {
    data = malloc(_userdatasize);
    create_body(worldId, x, y,
                _density, _friction, _restitution, _shapeType, _yRadius, _polygonVertices);
}


//Robot::~Robot() {
//    free(data);
//    data = nullptr;
//}


void Robot::create_body(b2WorldId worldId, float x, float y,
                        float density, float friction, float restitution,
                        ShapeType shapeType, float yRadius,
                        const std::vector<b2Vec2>& polygonVertices) {
    // Create the body definition.
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = { x / VISUALIZATION_SCALE, y / VISUALIZATION_SCALE };
    bodyDef.linearDamping = 1000.0f;
    bodyDef.angularDamping = 1000.0f;
    bodyDef.isBullet = false;
    bodyId = b2CreateBody(worldId, &bodyDef);

    // Set up a shape definition with common physical properties.
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = density;
    shapeDef.friction = friction;
    shapeDef.restitution = restitution;
    shapeDef.enablePreSolveEvents = true;

    // Create the shape based on the specified shape type.
    switch (shapeType) {
        case ShapeType::Circle: {
            b2Circle circle;
            circle.center = { 0.0f, 0.0f };
            circle.radius = radius / VISUALIZATION_SCALE;
            shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);
            break;
        }
        case ShapeType::Ellipse: {
            // For an ellipse, use _yRadius if provided; otherwise default to the circle's radius.
            float effectiveYRadius = (yRadius > 0.0f ? yRadius : radius);
            const int vertexCount = 16; // Number of vertices to approximate the ellipse.
            std::vector<b2Vec2> ellipseVertices;
            ellipseVertices.reserve(vertexCount);
            for (int i = 0; i < vertexCount; i++) {
                float angle = (2.0f * 3.14159265f * i) / vertexCount;
                float vx = (radius * std::cos(angle)) / VISUALIZATION_SCALE;
                float vy = (effectiveYRadius * std::sin(angle)) / VISUALIZATION_SCALE;
                ellipseVertices.push_back({ vx, vy });
            }
            // Create a temporary b2Polygon and fill it with the ellipse vertices.
            b2Polygon poly;
            poly.count = static_cast<int32_t>(ellipseVertices.size());
            for (int i = 0; i < poly.count; ++i) {
                poly.vertices[i] = ellipseVertices[i];
            }
            shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &poly);
            break;
        }
        case ShapeType::Polygon: {
            if (!polygonVertices.empty()) {
                // Create a temporary b2Polygon and fill it with the provided vertices.
                b2Polygon poly;
                poly.count = static_cast<int32_t>(polygonVertices.size());
                for (int i = 0; i < poly.count; ++i) {
                    poly.vertices[i] = polygonVertices[i];
                }
                shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &poly);
            } else {
                // Fallback: if no vertices are provided, default to a circle.
                b2Circle circle;
                circle.center = { 0.0f, 0.0f };
                circle.radius = radius / VISUALIZATION_SCALE;
                shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);
            }
            break;
        }
        default: {
            // Fallback to circle for any unrecognized shape type.
            b2Circle circle;
            circle.center = { 0.0f, 0.0f };
            circle.radius = radius / VISUALIZATION_SCALE;
            shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);
            break;
        }
    }

    // Assign an initial velocity.
    b2Vec2 velocity = { 1.0f, 1.0f };
    b2Body_SetLinearVelocity(bodyId, velocity);

    // Identify the level of temporal noise on this robot
    if (temporal_noise_stddev > 0) {
        //std::normal_distribution<float> dist(0.0f, temporal_noise_stddev);
        std::uniform_real_distribution<float> dist(0.0f, temporal_noise_stddev);
        temporal_noise = dist(rnd_gen);
    }
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


void Robot::render(SDL_Renderer* renderer, [[maybe_unused]] b2WorldId worldId, bool show_comm, bool show_lateral_leds) const {
    // Get robot's position in the physics world
    b2Vec2 position = b2Body_GetPosition(bodyId);

    // Convert to screen coordinates
    float screenX = position.x * VISUALIZATION_SCALE;
    float screenY = position.y * VISUALIZATION_SCALE;

    // Get the robot's orientation as a rotation (cosine/sine pair)
    b2Rot rotation = b2Body_GetRotation(bodyId);
    float cosAngle = rotation.c;
    float sinAngle = rotation.s;

    auto const circle_pos = visualization_position(screenX, screenY);

    // Define relative positions for LEDs around the robot based on orientation.
    // For the lateral LEDs we want them exactly on the border, so use full 'radius'.
    std::vector<b2Vec2> ledOffsets;
    if (show_lateral_leds) {
        ledOffsets = {
            {0, 0},            // Center LED remains at the center
            {0, -radius},      // Top (in simulation units)
            {radius, 0},       // Right
            {0, radius},       // Bottom
            {-radius, 0}       // Left
        };

        // Apply a 45° clockwise rotation to the lateral LEDs (skip index 0)
        const float angle = M_PI / 4;  // 45 degrees in radians
        const float cos45 = cos(angle);
        const float sin45 = sin(angle);
        for (size_t i = 1; i < ledOffsets.size(); ++i) {
            float originalX = ledOffsets[i].x;
            float originalY = ledOffsets[i].y;
            ledOffsets[i].x = originalX * cos45 + originalY * sin45;
            ledOffsets[i].y = -originalX * sin45 + originalY * cos45;
        }
    } else {
        ledOffsets = {
            {0, 0}             // Only the center LED is used
        };
    }

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

        // Calculate screen coordinates for the LED.
        // Here, we scale the rotated offset from simulation units to pixels.
        float ledScreenX = screenX + rotatedLedOffsets[i].x  * 0.95;
        float ledScreenY = screenY + rotatedLedOffsets[i].y  * 0.95;

        auto const led_pos = visualization_position(ledScreenX, ledScreenY);
        if (i == 0) {
            // Center LED is drawn as a full circle.
            filledCircleRGBA(renderer, led_pos.x, led_pos.y, (radius - 2) * mm_to_pixels, r, g, b, 255);
        } else {
            // For lateral LEDs, only draw the half-disk that lies inside the robot's body.
            // Compute the angle from the LED center toward the robot center.
            float angleToCenter = atan2(screenY - ledScreenY, screenX - ledScreenX);
            // Convert the angle to degrees (SDL2_gfx expects degrees)
            float angleDeg = angleToCenter * 180.0f / M_PI;
            float startAngle = angleDeg - 90;  // start of 180° arc
            float endAngle   = angleDeg + 90;  // end of 180° arc

            // Draw the half-disk border
            filledPieRGBA(renderer, led_pos.x, led_pos.y,
                          (radius / 2.5 + 2) * mm_to_pixels, startAngle, endAngle,
                          255, 255, 255, 150);

            // Draw a half-disk for the LED.
            // (Assumes SDL2_gfx's filledPieRGBA is available.)
            filledPieRGBA(renderer, led_pos.x, led_pos.y,
                          (radius / 2.5) * mm_to_pixels, startAngle, endAngle,
                          r, g, b, 255);
        }
    }

    // Draw the main robot body (outline)
    circleRGBA(renderer, circle_pos.x, circle_pos.y, radius * mm_to_pixels, 0, 0, 0, 255);

    // Draw arrow indicating orientation
    float arrowLength = radius * 1.0;
    float arrowHeadSize = radius * 0.4;

    // Get opposite color of main LED for arrow
    uint8_t arrowR = 255, arrowG = 255, arrowB = 160;  // Default yellow arrow
    if (!leds.empty()) {
        color_t const& mainLed = leds[0];
        arrowR = 255 - adjust_color(mainLed.r);
        arrowG = 255 - adjust_color(mainLed.g);
        arrowB = 255 - adjust_color(mainLed.b);
        if (arrowR < 100 && arrowG < 100 && arrowB < 100) {
            arrowR = std::min(255, arrowR + 100);
            arrowG = std::min(255, arrowG + 100);
            arrowB = std::min(255, arrowB + 100);
        }
    }

    float endX = screenX + cosAngle * arrowLength;
    float endY = screenY + sinAngle * arrowLength;
    auto const end_pos = visualization_position(endX, endY);
    thickLineRGBA(renderer, circle_pos.x, circle_pos.y, end_pos.x, end_pos.y, 4, arrowR, arrowG, arrowB, 255);

    float arrowLeft = endX - arrowHeadSize * (cosAngle * 0.7 + sinAngle * 0.5);
    float arrowRight = endX - arrowHeadSize * (cosAngle * 0.7 - sinAngle * 0.5);
    float arrowTopY = endY - arrowHeadSize * (sinAngle * 0.7 - cosAngle * 0.5);
    float arrowBottomY = endY - arrowHeadSize * (sinAngle * 0.7 + cosAngle * 0.5);
    auto const arrow_left = visualization_position(arrowLeft, arrowTopY);
    auto const arrow_right = visualization_position(arrowRight, arrowBottomY);
    filledTrigonRGBA(renderer, end_pos.x, end_pos.y,
                      arrow_left.x, arrow_left.y,
                      arrow_right.x, arrow_right.y,
                      arrowR, arrowG, arrowB, 255);

    // Draw communication channels if needed
    if (show_comm) {
        for (int i = 0; i < IR_RX_COUNT; i++ ) {
            // Find IR emitter position
            //b2Vec2 ir_position = get_IR_emitter_position((ir_direction)i, 1.f / VISUALIZATION_SCALE);
            b2Vec2 ir_position = get_position();
            auto const ir_pos = visualization_position(ir_position.x * VISUALIZATION_SCALE, ir_position.y * VISUALIZATION_SCALE);
            // Get color from colormap
            uint8_t r, g, b;
            qualitative_colormap(i, &r, &g, &b);
            // Draw communication channels
            for (Robot* robot : neighbors[i]) {
                b2Vec2 const r_pos = robot->get_position();
                auto const r_circle_pos = visualization_position(r_pos.x * VISUALIZATION_SCALE, r_pos.y * VISUALIZATION_SCALE);
                thickLineRGBA(renderer, ir_pos.x, ir_pos.y, r_circle_pos.x, r_circle_pos.y, 4, r, g, b, 150);
            }
        }
    }
}



void Robot::set_motor(motor_id motor, int speed) {
    // Update motor speeds
    if (motor == motorL) {
        left_motor_speed = speed;
    } else if (motor == motorR) {
        right_motor_speed = speed;
    }
    // glogger->debug("set motor: {} {}", left_motor_speed, right_motor_speed);

    // Set damping values using those provided during construction.
    b2Body_SetLinearDamping(bodyId, linearDamping);
    b2Body_SetAngularDamping(bodyId, angularDamping);

    // Compute the desired linear velocity based on motor speeds.
    b2Rot const rot = b2Body_GetRotation(bodyId);
    float const v = 1.0f * (left_motor_speed / static_cast<float>(motorFull) +
                            right_motor_speed / static_cast<float>(motorFull)) / 2.0f;
    b2Vec2 linear_velocity = {rot.c * v, rot.s * v};

    // Add Gaussian noise to linear velocity if the standard deviation is greater than 0.0.
    if (linear_noise_stddev > 0.0f) {
        // Use a static generator so that it persists across calls.
        std::normal_distribution<float> dist(0.0f, linear_noise_stddev);
        linear_velocity.x += dist(rnd_gen);
        linear_velocity.y += dist(rnd_gen);
    }
    b2Body_SetLinearVelocity(bodyId, linear_velocity);

    // Compute the desired angular velocity based on motor speed difference.
    float angular_velocity = 1.0f / (static_cast<float>(motorFull) * 0.5f) *
                             (left_motor_speed - right_motor_speed);

    // Add Gaussian noise to angular velocity if the standard deviation is greater than 0.0.
    if (angular_noise_stddev > 0.0f) {
        std::normal_distribution<float> dist(0.0f, angular_noise_stddev);
        angular_velocity += dist(rnd_gen);
    }
    b2Body_SetAngularVelocity(bodyId, angular_velocity);

    //glogger->debug("velocity: lin={},{}  ang={}  noise={},{}", linear_velocity.x, linear_velocity.y, angular_velocity, linear_noise_stddev, angular_noise_stddev);
}


void Robot::launch_user_step() {
    update_time();
    enable_stop_watches();
    //user_step();
    pogo_main_loop_step(user_step);
    disable_stop_watches();
}

void Robot::update_time() {
    current_time_microseconds += temporal_noise;
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

b2Vec2 Robot::get_IR_emitter_position(ir_direction dir) const {
    b2Vec2 pos = b2Body_GetPosition(bodyId);

    // Get the robot's orientation as a rotation (cosine/sine pair)
    b2Rot rotation = b2Body_GetRotation(bodyId);
    float cosAngle = rotation.c;
    float sinAngle = rotation.s;

    std::vector<b2Vec2> irOffsets = {
        {0, -radius},      // Front (in simulation units)
        {radius, 0},       // Right
        {0, radius},       // Back
        {-radius, 0},      // Left
        {0, 0}             // Middle
    };

    // Rotate IR offset based on robot orientation
    auto const& offset = irOffsets[dir];
    pos.x += cosAngle * offset.x / VISUALIZATION_SCALE - sinAngle * offset.y / VISUALIZATION_SCALE;
    pos.y += sinAngle * offset.x / VISUALIZATION_SCALE + cosAngle * offset.y / VISUALIZATION_SCALE;
    return pos;
}

float Robot::get_angle() const {
    b2Rot const rotation = b2Body_GetRotation(bodyId);
    return std::atan2(rotation.s, rotation.c);
}


void Robot::send_to_neighbors(ir_direction dir, short_message_t *const message) {
    // Reconstruct a long message from the short message
    message_t m;
    m.header._packet_type = message->header._packet_type;
    m.header._emitting_power_list = 0; // power all to 0 shouldn't emit something
    m.header._sender_id = 0xFF;
    m.header._sender_ir_index = 0xF; // index not possible
    m.header._receiver_ir_index = 0;
    m.header.payload_length = message->header.payload_length;
    memcpy( m.payload, message->payload, m.header.payload_length);

    send_to_neighbors(dir, &m);
}

void Robot::send_to_neighbors(ir_direction dir, message_t *const message) {
    // Define a uniform real distribution between 0.0 and 1.0
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);

    double const payload_size = static_cast<double>(message->header.payload_length); 
    double const msg_size = payload_size + (message->header._packet_type == ir_t_short ? sizeof(message_short_header_t) : sizeof(message_header_t));
    double const p_send = static_cast<double>(percent_msgs_sent_per_ticks) / 100.0;
    double const cluster_size = static_cast<double>(neighbors[dir].size() + 1);

    for (Robot* robot : neighbors[dir]) {
        float const prob = dis(rnd_gen);
        //glogger->debug("MESSAGE !! with prob {} / {}: {} -> {}", prob, msg_success_rate, message->header._sender_id, robot->id);
        if (prob <= (*msg_success_rate)(msg_size, p_send, cluster_size) && robot->messages.size() < 100) {
            robot->messages.push(*message);
        }
    }
}

void Robot::sleep_µs(uint64_t microseconds) {
    if (microseconds <= 0) return;
    current_time_microseconds += microseconds;
}


/************* Pogobot Objects *************/ // {{{1


PogobotObject::PogobotObject(uint16_t _id, float _x, float _y,
       ObjectGeometry& geom, b2WorldId world_id,
       size_t _userdatasize,
       float _communication_radius,
       std::unique_ptr<MsgSuccessRate> _msg_success_rate,
       float _temporal_noise_stddev,
       float _linear_damping, float _angular_damping,
       float _density, float _friction, float _restitution,
       float _linear_noise_stddev, float _angular_noise_stddev)
    : PhysicalObject(_id, _x, _y, geom, world_id,
      _temporal_noise_stddev, _linear_damping, _angular_damping,
      _density, _friction, _restitution),
    communication_radius(_communication_radius), msg_success_rate(std::move(_msg_success_rate)),
    linear_noise_stddev(_linear_noise_stddev), angular_noise_stddev(_angular_noise_stddev) {
    data = malloc(_userdatasize);
    create_robot_body(world_id);
}

PogobotObject::PogobotObject(uint16_t _id, float _x, float _y,
       b2WorldId world_id, size_t _userdatasize, Configuration const& config) : PhysicalObject(_id, _x, _y, world_id, config) {
    parse_configuration(config);
    data = malloc(_userdatasize);
    create_robot_body(world_id);
}

void PogobotObject::parse_configuration(Configuration const& config) {
    PhysicalObject::parse_configuration(config);
    msg_success_rate.reset(msg_success_rate_factory(config["msg_success_rate"]));
    communication_radius = config["communication_radius"].get(80.0f);
    linear_noise_stddev  = config["linear_noise_stddev"].get(0.0f);
    angular_noise_stddev = config["angular_noise_stddev"].get(0.0f);
}


void PogobotObject::create_robot_body([[maybe_unused]] b2WorldId world_id) {
    // Assign an initial velocity.
    b2Vec2 velocity = { 1.0f, 1.0f };
    b2Body_SetLinearVelocity(body_id, velocity);

    // Ensure the geometry is a disk, and extract radius
    DiskGeometry* g = dynamic_cast<DiskGeometry*>(geom);
    if (!g)
        throw std::runtime_error("Pogobots can only have a geometry of type 'disk'.");
    radius = g->get_radius();
}


void PogobotObject::launch_user_step() {
    update_time();
    enable_stop_watches();
    //user_step();
    pogo_main_loop_step(user_step);
    disable_stop_watches();
}

void PogobotObject::register_stop_watch(time_reference_t* sw) {
    stop_watches.insert(sw);
}

void PogobotObject::enable_stop_watches() {
    for (auto* sw : stop_watches) {
        sw->enable();
    }
}

void PogobotObject::disable_stop_watches() {
    for (auto* sw : stop_watches) {
        sw->disable();
    }
}


void PogobotObject::render(SDL_Renderer* renderer, [[maybe_unused]] b2WorldId worldId) const {
    // Get robot's position in the physics world
    b2Vec2 position = b2Body_GetPosition(body_id);

    // Convert to screen coordinates
    float screenX = position.x * VISUALIZATION_SCALE;
    float screenY = position.y * VISUALIZATION_SCALE;
    auto const circle_pos = visualization_position(screenX, screenY);

    // Get the robot's orientation as a rotation (cosine/sine pair)
    b2Rot rotation = b2Body_GetRotation(body_id);
    float cosAngle = rotation.c;
    float sinAngle = rotation.s;

    // Define relative positions for LEDs around the robot based on orientation.
    // For the lateral LEDs we want them exactly on the border, so use full 'radius'.
    std::vector<b2Vec2> ledOffsets;
    if (show_lateral_leds) {
        ledOffsets = {
            {0, 0},            // Center LED remains at the center
            {0, -radius},      // Top (in simulation units)
            {radius, 0},       // Right
            {0, radius},       // Bottom
            {-radius, 0}       // Left
        };

        // Apply a 45° clockwise rotation to the lateral LEDs (skip index 0)
        const float angle = M_PI / 4;  // 45 degrees in radians
        const float cos45 = cos(angle);
        const float sin45 = sin(angle);
        for (size_t i = 1; i < ledOffsets.size(); ++i) {
            float originalX = ledOffsets[i].x;
            float originalY = ledOffsets[i].y;
            ledOffsets[i].x = originalX * cos45 + originalY * sin45;
            ledOffsets[i].y = -originalX * sin45 + originalY * cos45;
        }
    } else {
        ledOffsets = {
            {0, 0}             // Only the center LED is used
        };
    }

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

        // Calculate screen coordinates for the LED.
        // Here, we scale the rotated offset from simulation units to pixels.
        float ledScreenX = screenX + rotatedLedOffsets[i].x  * 0.95;
        float ledScreenY = screenY + rotatedLedOffsets[i].y  * 0.95;

        auto const led_pos = visualization_position(ledScreenX, ledScreenY);
        if (i == 0) {
            // Center LED is drawn as a full circle.
            filledCircleRGBA(renderer, led_pos.x, led_pos.y, (radius - 2) * mm_to_pixels, r, g, b, 255);
        } else {
            // For lateral LEDs, only draw the half-disk that lies inside the robot's body.
            // Compute the angle from the LED center toward the robot center.
            float angleToCenter = atan2(screenY - ledScreenY, screenX - ledScreenX);
            // Convert the angle to degrees (SDL2_gfx expects degrees)
            float angleDeg = angleToCenter * 180.0f / M_PI;
            float startAngle = angleDeg - 90;  // start of 180° arc
            float endAngle   = angleDeg + 90;  // end of 180° arc

            // Draw the half-disk border
            filledPieRGBA(renderer, led_pos.x, led_pos.y,
                          (radius / 2.5 + 2) * mm_to_pixels, startAngle, endAngle,
                          255, 255, 255, 150);

            // Draw a half-disk for the LED.
            // (Assumes SDL2_gfx's filledPieRGBA is available.)
            filledPieRGBA(renderer, led_pos.x, led_pos.y,
                          (radius / 2.5) * mm_to_pixels, startAngle, endAngle,
                          r, g, b, 255);
        }
    }

    // Draw the main robot body (outline)
    circleRGBA(renderer, circle_pos.x, circle_pos.y, radius * mm_to_pixels, 0, 0, 0, 255);

    // Draw arrow indicating orientation
    float arrowLength = radius * 1.0;
    float arrowHeadSize = radius * 0.4;

    // Get opposite color of main LED for arrow
    uint8_t arrowR = 255, arrowG = 255, arrowB = 160;  // Default yellow arrow
    if (!leds.empty()) {
        color_t const& mainLed = leds[0];
        arrowR = 255 - adjust_color(mainLed.r);
        arrowG = 255 - adjust_color(mainLed.g);
        arrowB = 255 - adjust_color(mainLed.b);
        if (arrowR < 100 && arrowG < 100 && arrowB < 100) {
            arrowR = std::min(255, arrowR + 100);
            arrowG = std::min(255, arrowG + 100);
            arrowB = std::min(255, arrowB + 100);
        }
    }

    float endX = screenX + cosAngle * arrowLength;
    float endY = screenY + sinAngle * arrowLength;
    auto const end_pos = visualization_position(endX, endY);
    thickLineRGBA(renderer, circle_pos.x, circle_pos.y, end_pos.x, end_pos.y, 4, arrowR, arrowG, arrowB, 255);

    float arrowLeft = endX - arrowHeadSize * (cosAngle * 0.7 + sinAngle * 0.5);
    float arrowRight = endX - arrowHeadSize * (cosAngle * 0.7 - sinAngle * 0.5);
    float arrowTopY = endY - arrowHeadSize * (sinAngle * 0.7 - cosAngle * 0.5);
    float arrowBottomY = endY - arrowHeadSize * (sinAngle * 0.7 + cosAngle * 0.5);
    auto const arrow_left = visualization_position(arrowLeft, arrowTopY);
    auto const arrow_right = visualization_position(arrowRight, arrowBottomY);
    filledTrigonRGBA(renderer, end_pos.x, end_pos.y,
                      arrow_left.x, arrow_left.y,
                      arrow_right.x, arrow_right.y,
                      arrowR, arrowG, arrowB, 255);

    // Draw communication channels if needed
    if (show_comm) {
        for (int i = 0; i < IR_RX_COUNT; i++ ) {
            // Find IR emitter position
            //b2Vec2 ir_position = get_IR_emitter_position((ir_direction)i, 1.f / VISUALIZATION_SCALE);
            b2Vec2 ir_position = get_position();
            auto const ir_pos = visualization_position(ir_position.x * VISUALIZATION_SCALE, ir_position.y * VISUALIZATION_SCALE);
            // Get color from colormap
            uint8_t r, g, b;
            qualitative_colormap(i, &r, &g, &b);
            // Draw communication channels
            for (PogobotObject* robot : neighbors[i]) {
                b2Vec2 const r_pos = robot->get_position();
                auto const r_circle_pos = visualization_position(r_pos.x * VISUALIZATION_SCALE, r_pos.y * VISUALIZATION_SCALE);
                thickLineRGBA(renderer, ir_pos.x, ir_pos.y, r_circle_pos.x, r_circle_pos.y, 4, r, g, b, 150);
            }
        }
    }
}

void PogobotObject::set_motor(motor_id motor, int speed) {
    // Update motor speeds
    if (motor == motorL) {
        left_motor_speed = speed;
    } else if (motor == motorR) {
        right_motor_speed = speed;
    }
    // glogger->debug("set motor: {} {}", left_motor_speed, right_motor_speed);

    // Set damping values using those provided during construction.
    b2Body_SetLinearDamping(body_id, linear_damping);
    b2Body_SetAngularDamping(body_id, angular_damping);

    // Compute the desired linear velocity based on motor speeds.
    b2Rot const rot = b2Body_GetRotation(body_id);
    float const v = 1.0f * (left_motor_speed / static_cast<float>(motorFull) +
                            right_motor_speed / static_cast<float>(motorFull)) / 2.0f;
    b2Vec2 linear_velocity = {rot.c * v, rot.s * v};

    // Add Gaussian noise to linear velocity if the standard deviation is greater than 0.0.
    if (linear_noise_stddev > 0.0f) {
        // Use a static generator so that it persists across calls.
        std::normal_distribution<float> dist(0.0f, linear_noise_stddev);
        linear_velocity.x += dist(rnd_gen);
        linear_velocity.y += dist(rnd_gen);
    }
    b2Body_SetLinearVelocity(body_id, linear_velocity);

    // Compute the desired angular velocity based on motor speed difference.
    float angular_velocity = 1.0f / (static_cast<float>(motorFull) * 0.5f) *
                             (left_motor_speed - right_motor_speed);

    // Add Gaussian noise to angular velocity if the standard deviation is greater than 0.0.
    if (angular_noise_stddev > 0.0f) {
        std::normal_distribution<float> dist(0.0f, angular_noise_stddev);
        angular_velocity += dist(rnd_gen);
    }
    b2Body_SetAngularVelocity(body_id, angular_velocity);

    //glogger->debug("velocity: lin={},{}  ang={}  noise={},{}", linear_velocity.x, linear_velocity.y, angular_velocity, linear_noise_stddev, angular_noise_stddev);
}


b2Vec2 PogobotObject::get_IR_emitter_position(ir_direction dir) const {
    b2Vec2 pos = b2Body_GetPosition(body_id);

    // Get the robot's orientation as a rotation (cosine/sine pair)
    b2Rot rotation = b2Body_GetRotation(body_id);
    float cosAngle = rotation.c;
    float sinAngle = rotation.s;

    std::vector<b2Vec2> irOffsets = {
        {0, -radius},      // Front (in simulation units)
        {radius, 0},       // Right
        {0, radius},       // Back
        {-radius, 0},      // Left
        {0, 0}             // Middle
    };

    // Rotate IR offset based on robot orientation
    auto const& offset = irOffsets[dir];
    pos.x += cosAngle * offset.x / VISUALIZATION_SCALE - sinAngle * offset.y / VISUALIZATION_SCALE;
    pos.y += sinAngle * offset.x / VISUALIZATION_SCALE + cosAngle * offset.y / VISUALIZATION_SCALE;
    return pos;
}

void PogobotObject::send_to_neighbors(ir_direction dir, short_message_t *const message) {
    // Reconstruct a long message from the short message
    message_t m;
    m.header._packet_type = message->header._packet_type;
    m.header._emitting_power_list = 0; // power all to 0 shouldn't emit something
    m.header._sender_id = 0xFF;
    m.header._sender_ir_index = 0xF; // index not possible
    m.header._receiver_ir_index = 0;
    m.header.payload_length = message->header.payload_length;
    memcpy( m.payload, message->payload, m.header.payload_length);

    send_to_neighbors(dir, &m);
}

void PogobotObject::send_to_neighbors(ir_direction dir, message_t *const message) {
    // Define a uniform real distribution between 0.0 and 1.0
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);

    double const payload_size = static_cast<double>(message->header.payload_length); 
    double const msg_size = payload_size + (message->header._packet_type == ir_t_short ? sizeof(message_short_header_t) : sizeof(message_header_t));
    double const p_send = static_cast<double>(percent_msgs_sent_per_ticks) / 100.0;
    double const cluster_size = static_cast<double>(neighbors[dir].size() + 1);

    for (PogobotObject* robot : neighbors[dir]) {
        float const prob = dis(rnd_gen);
        //glogger->debug("MESSAGE !! with prob {} / {}: {} -> {}", prob, msg_success_rate, message->header._sender_id, robot->id);
        if (prob <= (*msg_success_rate)(msg_size, p_send, cluster_size) && robot->messages.size() < 100) {
            robot->messages.push(*message);
        }
    }
}

void PogobotObject::sleep_µs(uint64_t microseconds) {
    if (microseconds <= 0) return;
    current_time_microseconds += microseconds;
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
