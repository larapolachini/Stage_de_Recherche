#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <set>
#include <queue>
#include <chrono>
#include <SDL2/SDL.h>
#include <box2d/box2d.h>

#include "utils.h"
#include "render.h"
#include "colormaps.h"
#include "spogobot.h"

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
     * @param _msg_success_rate Probability (0.0 to 1.0) of successfully sending a message (default is 0.5).
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
     */
    Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius,
          b2WorldId worldId, float _msg_success_rate = 0.5,
          float _linearDamping = 0.0f, float _angularDamping = 0.0f,
          float _density = 10.0f, float _friction = 0.3f, float _restitution = 0.5f,
          ShapeType _shapeType = ShapeType::Circle, float _yRadius = 0.0f,
          const std::vector<b2Vec2>& _polygonVertices = std::vector<b2Vec2>(),
          float _linear_noise_stddev = 0.0f, float _angular_noise_stddev = 0.0f);


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

    std::chrono::time_point<std::chrono::system_clock> current_time; ///< Current system time.
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
     */
    void render(SDL_Renderer* renderer, b2WorldId worldId, bool show_comm = false) const;

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
    std::vector<Robot*> neighbors;  ///< Pointers to neighboring robots.
    std::queue<message_t> messages; ///< Queue of incoming messages.

    /**
     * @brief Sends a short message to neighboring robots.
     *
     * Converts a short message to a full message and forwards it to all neighboring robots.
     *
     * @param message Pointer to the short_message_t to send.
     */
    void send_to_neighbors(short_message_t *const message);

    /**
     * @brief Sends a message to neighboring robots.
     *
     * Iterates over neighboring robots and, based on a random probability and the message success rate,
     * enqueues the message in each neighbor's message queue.
     *
     * @param message Pointer to the message_t to send.
     */
    void send_to_neighbors(message_t *const message);

    // Message success rate
    float msg_success_rate = 0.5; ///< Probability of successfully sending a message.

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
};


extern Robot* current_robot;
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
