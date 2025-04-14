#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <cmath>
#include <set>
#include <queue>
#include <chrono>
#include <SDL2/SDL.h>
#include <box2d/box2d.h>

#include "utils.h"
#include "render.h"
#include "colormaps.h"
#include "spogobot.h"
#include "objects.h"

/**
 * @brief Returns a log string for the current robot.
 *
 * This function gathers and returns a string containing relevant log information
 * for the currently active robot.
 *
 * @return std::string A string representing the current robot's log.
 */
std::string log_current_robot();


/**
 * @brief Enumeration to select the shape type of the robot's body.
 */
enum class ShapeType {
    Circle,   /**< Use a circular shape. */
    Ellipse,  /**< Use an elliptical shape (approximated by a polygon). */
    Polygon   /**< Use an arbitrary polygon shape defined by vertices. */
};


/**
 * @brief Abstract base class for message success rate calculations.
 *
 * This class defines the interface for calculating message success rates.
 */
class MsgSuccessRate {
public:
    /**
     * @brief Computes the message success rate.
     *
     * @param msg_size The message size.
     * @param p_send The sending parameter.
     * @param cluster_size The cluster size.
     * @return The computed success rate.
     */
    virtual double operator()(double msg_size, double p_send, double cluster_size) const = 0;

    /**
     * @brief Virtual destructor.
     */
    virtual ~MsgSuccessRate() = default;
};

/**
 * @brief Class for dynamic message success rate calculation using a configurable formula.
 *
 * The success rate is calculated as:
 * \f[
 * \text{success rate} = \frac{1}{1 + (\alpha \cdot \text{msg\_size}^{\beta} \cdot \text{p\_send}^{\gamma} \cdot \text{cluster\_size}^{\delta})}
 * \f]
 */
class DynamicMsgSuccessRate : public MsgSuccessRate {
public:
    /**
     * @brief Constructs a new DynamicMsgSuccessRate object.
     *
     * @param alpha The multiplicative constant (default is 0.000004).
     * @param beta The exponent for msg_size (default is 2.8096).
     * @param gamma The exponent for p_send (default is 2.3807).
     * @param delta The exponent for cluster_size (default is 1.2457).
     */
    DynamicMsgSuccessRate(double alpha = 0.000004, double beta = 2.8096, double gamma = 2.3807, double delta = 1.2457);

    /**
     * @brief Computes the dynamic success rate.
     *
     * @param msg_size The msg size.
     * @param p_send The sending parameter.
     * @param cluster_size The cluster size.
     * @return The computed success rate.
     */
    double operator()(double msg_size, double p_send, double cluster_size) const override;

private:
    double alpha_; ///< The multiplicative constant.
    double beta_;  ///< Exponent for msg size.
    double gamma_; ///< Exponent for sending parameter.
    double delta_; ///< Exponent for cluster size.
};

/**
 * @brief Class for constant message success rate calculation.
 *
 * This class always returns a constant success rate value regardless of the inputs.
 */
class ConstMsgSuccessRate : public MsgSuccessRate {
public:
    /**
     * @brief Constructs a new ConstMsgSuccessRate object.
     *
     * @param value The constant success rate value.
     */
    explicit ConstMsgSuccessRate(double value);

    /**
     * @brief Returns the constant success rate.
     *
     * @param msg_size The msg size (ignored).
     * @param p_send The sending parameter (ignored).
     * @param cluster_size The cluster size (ignored).
     * @return The constant success rate.
     */
    double operator()(double msg_size, double p_send, double cluster_size) const override;

private:
    double const_value_; ///< The constant success rate value.
};

/**
 * @brief Factory of MsgSuccessRate, from a given configuration
 *
 * @param config Configuration entry describing the object properties.
 */
MsgSuccessRate* msg_success_rate_factory(Configuration const& config);


/**
 * @brief Class representing a simulated robot.
 *
 * The Robot class encapsulates the properties and behaviors of a simulated robot,
 * including physical properties, timing, LED control, messaging, and rendering.
 */
class Robot {

public:

    /**
     * @brief Constructs a Robot object.
     *
     * Initializes a new robot with the specified identifier, user data size, initial position,
     * radius, associated Box2D world, and message success rate. It also allows customization
     * of the body's physical properties (linear and angular damping, density, friction, and restitution)
     * as well as the choice of body shape. The shape can be a circle, an ellipse (using a secondary radius),
     * or an arbitrary polygon defined by a vector of vertices.
     *
     * @param _id Unique robot identifier.
     * @param _userdatasize Size of the memory block allocated for user data.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param _radius The primary radius of the robot.
     * @param worldId The Box2D world identifier.
     * @param _msg_success_rate std::unique_ptr<MsgSuccessRate> describing the probability of successfully sending a message.
     * @param _linearDamping Linear damping value for the physical body (default is 0.0f).
     * @param _angularDamping Angular damping value for the physical body (default is 0.0f).
     * @param _density Density of the body shape (default is 10.0f).
     * @param _friction Friction coefficient of the body shape (default is 0.3f).
     * @param _restitution Restitution (bounciness) of the body shape (default is 0.5f).
     * @param _shapeType Type of shape to use for the physical body (default is ShapeType::Circle).
     * @param _yRadius Secondary radius for an ellipse; if 0, defaults to _radius (only used for ShapeType::Ellipse).
     * @param _polygonVertices Vector of vertices defining the polygon shape (only used for ShapeType::Polygon).
     * @param _linear_noise_stddev Standard deviation of the gaussian noise to apply to linear velocity, or 0.0 for deterministic velocity
     * @param _angular_noise_stddev Standard deviation of the gaussian noise to apply to angular velocity, or 0.0 for deterministic velocity
     * @param _temporal_noise_stddev Standard deviation of the gaussian noise to apply to time on each robot, or 0.0 for deterministic time
     */
    Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius,
          b2WorldId worldId, std::unique_ptr<MsgSuccessRate> _msg_success_rate = std::make_unique<ConstMsgSuccessRate>(0.5),
          float _linearDamping = 0.0f, float _angularDamping = 0.0f,
          float _density = 10.0f, float _friction = 0.3f, float _restitution = 0.5f,
          ShapeType _shapeType = ShapeType::Circle, float _yRadius = 0.0f,
          const std::vector<b2Vec2>& _polygonVertices = std::vector<b2Vec2>(),
          float _linear_noise_stddev = 0.0f, float _angular_noise_stddev = 0.0f,
          float _temporal_noise_stddev = 0.0f);


    //virtual ~Robot();

    // Base info
    uint16_t id;                         ///< Robot identifier.
    void* data = nullptr;                ///< Pointer to user data.
    void (*user_init)(void) = nullptr;   ///< Pointer to a user-defined initialization function.
    void (*user_step)(void) = nullptr;   ///< Pointer to a user-defined step function.
    void (*callback_create_data_schema)(void) = nullptr; ///< Callback to create a data schema.
    void (*callback_export_data)(void) = nullptr;        ///< Callback to export data.

    /**
     * @brief Launches the user-defined step function.
     *
     * Updates the robot's time, enables all registered stop watches, executes the user step
     * function via pogo_main_loop_step, and then disables the stop watches.
     */
    void launch_user_step();

    //std::chrono::time_point<std::chrono::system_clock> current_time; ///< Current system time.
    uint64_t current_time_microseconds = 0LL;                        ///< Current time in microseconds.

    // C-code accessible values
    uint32_t pogobot_ticks = 0;                 ///< Simulation ticks counter.
    uint8_t main_loop_hz = 60;                  ///< Main loop frequency in Hz.
    uint8_t max_nb_processed_msg_per_tick = 3;  ///< Maximum number of messages processed per tick.
    void (*msg_rx_fn)(message_t*) = nullptr;    ///< Function pointer for message reception.
    bool (*msg_tx_fn)(void) = nullptr;          ///< Function pointer for message transmission.
    int8_t error_codes_led_idx = 3;             ///< LED index for error codes.
    time_reference_t _global_timer;             ///< Global timer reference.
    time_reference_t timer_main_loop;           ///< Main loop timer reference.
    uint32_t _current_time_milliseconds = 0;    ///< Current time in milliseconds.
    uint32_t _error_code_initial_time = 0;      ///< Initial time for error code reporting.
    uint8_t percent_msgs_sent_per_ticks = 20;   ///< Percentage of messages sent per tick.
    uint32_t nb_msgs_sent = 0;                  ///< Counter for messages sent.
    uint32_t nb_msgs_recv = 0;                  ///< Counter for messages received.

    // Time-related utilities
    std::set<time_reference_t*> stop_watches;   ///< Set of registered stop watches.

    /**
     * @brief Updates the robot's current time.
     *
     * Computes the current time in microseconds relative to the simulation start.
     */
    void update_time();

    /**
     * @brief Registers a stop watch with the robot.
     *
     * Adds the given stop watch pointer to the set of stop watches.
     *
     * @param sw Pointer to the stop watch to register.
     */
    void register_stop_watch(time_reference_t* sw);

    /**
     * @brief Enables all registered stop watches.
     *
     * Iterates through all registered stop watches and enables them.
     */
    void enable_stop_watches();

    /**
     * @brief Disables all registered stop watches.
     *
     * Iterates through all registered stop watches and disables them.
     */
    void disable_stop_watches();

    // Physical information
    b2BodyId bodyId;       ///< Box2D body identifier.
    b2ShapeId shapeId;     ///< Box2D shape identifier.
    float radius = 5;      ///< Physical radius of the robot.
    float left_motor_speed  = 0; ///< Current speed of the left motor.
    float right_motor_speed = 0; ///< Current speed of the right motor.


    /**
     * @brief Renders the robot on the given SDL renderer.
     *
     * Draws the robot's body as a circle, its orientation as a thick line, and its LEDs at rotated positions.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param worldId The Box2D world identifier (unused in rendering).
     * @param show_comm Whether to show communication channels from this robot to its neighbors
     * @param show_lateral_leds Whether to show communication lateral LEDs
     */
    void render(SDL_Renderer* renderer, b2WorldId worldId, bool show_comm = false, bool show_lateral_leds = false) const;

    /**
     * @brief Updates the motor speed of the robot and recalculates its velocities.
     *
     * This method sets the motor speed (left or right) and then computes the robot's
     * linear and angular velocities. It applies the damping values
     * that were provided in the constructor. If the noise standard deviations
     * (for linear or angular velocities) are greater than 0.0, a Gaussian noise component is added
     * to the respective velocity.
     *
     * @param motor The identifier of the motor to update.
     * @param speed The new speed value for the selected motor.
     */
    void set_motor(motor_id motor, int speed);

    /**
     * @brief Retrieves the robot's current position.
     *
     * Returns the position of the robot's physical body as a Box2D vector.
     *
     * @return b2Vec2 The current position.
     */
    b2Vec2 get_position() const;

    /**
     * @brief Retrieves the IR emitters current positions
     *
     * Returns the position of one of the robot's IR emitter as a Box2D vector.
     *
     * @return b2Vec2 The current position.
     */
    b2Vec2 get_IR_emitter_position(ir_direction dir) const;

    /**
     * @brief Retrieves the robot's current orientation angle.
     *
     * Computes and returns the orientation angle (in radians) of the robot's body.
     *
     * @return float The orientation angle.
     */
    float get_angle() const;

    // LED control
    std::vector<color_t> leds = std::vector<color_t>(5, {0, 0, 0}); ///< LED colors for the robot.

    // Neighbors and messaging
    std::vector<std::vector<Robot*>> neighbors{ir_all+1};  ///< Pointers to neighboring robots.
    std::queue<message_t> messages; ///< Queue of incoming messages.

    /**
     * @brief Sends a short message to neighboring robots.
     *
     * Converts a short message to a full message and forwards it to all neighboring robots.
     *
     * @param dir Direction in which the message is sent (i.e. the number of the IR emitter)
     * @param message Pointer to the short_message_t to send.
     */
    void send_to_neighbors(ir_direction dir, short_message_t *const message);

    /**
     * @brief Sends a message to neighboring robots.
     *
     * Iterates over neighboring robots and, based on a random probability and the message success rate,
     * enqueues the message in each neighbor's message queue.
     *
     * @param dir Direction in which the message is sent (i.e. the number of the IR emitter)
     * @param message Pointer to the message_t to send.
     */
    void send_to_neighbors(ir_direction dir, message_t *const message);


    /**
     * @brief Simulate a sleep on a single robot.
     *
     * @param microseconds Number of microseconds to sleep for
     */
    void sleep_µs(uint64_t microseconds);

    // Message success rate
    std::unique_ptr<MsgSuccessRate> msg_success_rate; ///< Probability of successfully sending a message.

    float temporal_noise = 0;

private:

    /**
     * @brief Creates the robot's physical body in the simulation.
     *
     * Constructs a dynamic body in the Box2D world at the specified position, defines its shape
     * based on the provided shape type, and assigns an initial velocity. Depending on the shape type,
     * the body can be created as a circle, an ellipse (approximated by a polygon), or an arbitrary polygon.
     *
     * @param worldId The Box2D world identifier.
     * @param x The x-coordinate for the body's position.
     * @param y The y-coordinate for the body's position.
     * @param density Density of the shape.
     * @param friction Friction coefficient of the shape.
     * @param restitution Restitution (bounciness) of the shape.
     * @param shapeType The type of shape to create (Circle, Ellipse, or Polygon).
     * @param yRadius Secondary radius for an ellipse; if 0, defaults to the circle's radius (used for Ellipse).
     * @param polygonVertices Vector of vertices for the polygon shape (used for Polygon).
     */
    void create_body(b2WorldId worldId, float x, float y,
                     float density, float friction, float restitution,
                     ShapeType shapeType, float yRadius,
                     const std::vector<b2Vec2>& polygonVertices);

    float linearDamping;
    float angularDamping;
    float linear_noise_stddev;
    float angular_noise_stddev;
    float temporal_noise_stddev;
};



/**
 * @brief Class representing a simulated robot.
 *
 * The PogobotObject class encapsulates the properties and behaviors of a simulated Pogobot robot,
 * including physical properties, timing, LED control, messaging, and rendering.
 */
class PogobotObject : public PhysicalObject {

public:

    /**
     * Initializes a new Pogobot robot with the specified identifier, user data size, initial position,
     * radius, associated Box2D world, and message success rate. It also allows customization
     * of the body's physical properties (linear and angular damping, density, friction, and restitution).
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param geom Object's geometry.
     * @param world_id The Box2D world identifier.
     * @param _userdatasize Size of the memory block allocated for user data.
     * @param _communication_radius communication radius of each IR emitter
     * @param _msg_success_rate std::unique_ptr<MsgSuccessRate> describing the probability of successfully sending a message.
     * @param _temporal_noise_stddev Standard deviation of the gaussian noise to apply to time on each object, or 0.0 for deterministic time
     * @param _linear_damping Linear damping value for the physical body (default is 0.0f).
     * @param _angular_damping Angular damping value for the physical body (default is 0.0f).
     * @param _density Density of the body shape (default is 10.0f).
     * @param _friction Friction coefficient of the body shape (default is 0.3f).
     * @param _restitution Restitution (bounciness) of the body shape (default is 0.5f).
     * @param _linear_noise_stddev Standard deviation of the gaussian noise to apply to linear velocity, or 0.0 for deterministic velocity
     * @param _angular_noise_stddev Standard deviation of the gaussian noise to apply to angular velocity, or 0.0 for deterministic velocity
     */
    PogobotObject(uint16_t _id, float _x, float _y,
           ObjectGeometry& geom, b2WorldId world_id,
           size_t _userdatasize,
           float _communication_radius = 80.0f,
           std::unique_ptr<MsgSuccessRate> _msg_success_rate = std::make_unique<ConstMsgSuccessRate>(0.5),
           float _temporal_noise_stddev = 0.0f,
           float _linear_damping = 0.0f, float _angular_damping = 0.0f,
           float _density = 10.0f, float _friction = 0.3f, float _restitution = 0.5f,
           float _linear_noise_stddev = 0.0f, float _angular_noise_stddev = 0.0f);

    /**
     * @brief Constructs a PogobotObject from a configuration entry.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param world_id The Box2D world identifier.
     * @param _userdatasize Size of the memory block allocated for user data.
     * @param config Configuration entry describing the object properties.
     */
    PogobotObject(uint16_t _id, float _x, float _y,
           b2WorldId world_id, size_t _userdatasize, Configuration const& config);


    //virtual ~Robot();

    // Base info
    void* data = nullptr;                ///< Pointer to user data.
    void (*user_init)(void) = nullptr;   ///< Pointer to a user-defined initialization function.
    void (*user_step)(void) = nullptr;   ///< Pointer to a user-defined step function.
    void (*callback_create_data_schema)(void) = nullptr; ///< Callback to create a data schema.
    void (*callback_export_data)(void) = nullptr;        ///< Callback to export data.

    /**
     * @brief Launches the user-defined step function.
     *
     * Updates the object's time, enables all registered stop watches, executes the user step
     * function via pogo_main_loop_step, and then disables the stop watches.
     */
    virtual void launch_user_step() override;

    // C-code accessible values
    uint32_t pogobot_ticks = 0;                 ///< Simulation ticks counter.
    uint8_t main_loop_hz = 60;                  ///< Main loop frequency in Hz.
    uint8_t max_nb_processed_msg_per_tick = 3;  ///< Maximum number of messages processed per tick.
    void (*msg_rx_fn)(message_t*) = nullptr;    ///< Function pointer for message reception.
    bool (*msg_tx_fn)(void) = nullptr;          ///< Function pointer for message transmission.
    int8_t error_codes_led_idx = 3;             ///< LED index for error codes.
    time_reference_t _global_timer;             ///< Global timer reference.
    time_reference_t timer_main_loop;           ///< Main loop timer reference.
    uint32_t _current_time_milliseconds = 0;    ///< Current time in milliseconds.
    uint32_t _error_code_initial_time = 0;      ///< Initial time for error code reporting.
    uint8_t percent_msgs_sent_per_ticks = 20;   ///< Percentage of messages sent per tick.
    uint32_t nb_msgs_sent = 0;                  ///< Counter for messages sent.
    uint32_t nb_msgs_recv = 0;                  ///< Counter for messages received.

    // Time-related utilities
    std::set<time_reference_t*> stop_watches;   ///< Set of registered stop watches.

    /**
     * @brief Registers a stop watch with the robot.
     *
     * Adds the given stop watch pointer to the set of stop watches.
     *
     * @param sw Pointer to the stop watch to register.
     */
    void register_stop_watch(time_reference_t* sw);

    /**
     * @brief Enables all registered stop watches.
     *
     * Iterates through all registered stop watches and enables them.
     */
    void enable_stop_watches();

    /**
     * @brief Disables all registered stop watches.
     *
     * Iterates through all registered stop watches and disables them.
     */
    void disable_stop_watches();

    // Physical information
    float radius;                   ///< Radius of this robot.
    float left_motor_speed  = 0;    ///< Current speed of the left motor.
    float right_motor_speed = 0;    ///< Current speed of the right motor.
    bool show_comm = false;         ///< Whether to render communication channels.
    bool show_lateral_leds = false; ///< Whether to render lateral LEDs.


    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     */
    virtual void render(SDL_Renderer*, b2WorldId) const override;

    /**
     * @brief Updates the motor speed of the robot and recalculates its velocities.
     *
     * This method sets the motor speed (left or right) and then computes the robot's
     * linear and angular velocities. It applies the damping values
     * that were provided in the constructor. If the noise standard deviations
     * (for linear or angular velocities) are greater than 0.0, a Gaussian noise component is added
     * to the respective velocity.
     *
     * @param motor The identifier of the motor to update.
     * @param speed The new speed value for the selected motor.
     */
    void set_motor(motor_id motor, int speed);

    /**
     * @brief Retrieves the IR emitters current positions
     *
     * Returns the position of one of the robot's IR emitter as a Box2D vector.
     *
     * @return b2Vec2 The current position.
     */
    b2Vec2 get_IR_emitter_position(ir_direction dir) const;

    // LED control
    std::vector<color_t> leds = std::vector<color_t>(5, {0, 0, 0}); ///< LED colors for the robot.

    // Neighbors and messaging
    std::vector<std::vector<PogobotObject*>> neighbors{ir_all+1};  ///< Pointers to neighboring robots.
    std::queue<message_t> messages; ///< Queue of incoming messages.
    float communication_radius;                       ///< Communication radius of each IR emitter.
    std::unique_ptr<MsgSuccessRate> msg_success_rate; ///< Probability of successfully sending a message.

    /**
     * @brief Sends a short message to neighboring robots.
     *
     * Converts a short message to a full message and forwards it to all neighboring robots.
     *
     * @param dir Direction in which the message is sent (i.e. the number of the IR emitter)
     * @param message Pointer to the short_message_t to send.
     */
    void send_to_neighbors(ir_direction dir, short_message_t *const message);

    /**
     * @brief Sends a message to neighboring robots.
     *
     * Iterates over neighboring robots and, based on a random probability and the message success rate,
     * enqueues the message in each neighbor's message queue.
     *
     * @param dir Direction in which the message is sent (i.e. the number of the IR emitter)
     * @param message Pointer to the message_t to send.
     */
    void send_to_neighbors(ir_direction dir, message_t *const message);


    /**
     * @brief Simulate a sleep on a single robot.
     *
     * @param microseconds Number of microseconds to sleep for
     */
    void sleep_µs(uint64_t microseconds);


protected:
    /**
     * @brief Creates the object's physical body in the simulation.
     *
     * @param world_id The Box2D world identifier.
     */
    void create_robot_body(b2WorldId world_id);

    // Physical information
    float linear_noise_stddev;
    float angular_noise_stddev;

    /**
     * @brief Parse a provided configuration and set associated members values.
     *
     * @param config Configuration entry describing the object properties.
     */
    virtual void parse_configuration(Configuration const& config) override;
};


//extern Robot* current_robot;
extern PogobotObject* current_robot;
extern int UserdataSize;
extern void* mydata;
extern uint64_t sim_starting_time_microseconds;


/**
 * @brief Retrieves the current time in microseconds.
 *
 * Computes and returns the current simulation time in microseconds.
 *
 * @return uint64_t Current time in microseconds.
 */
uint64_t get_current_time_microseconds();

#endif


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
