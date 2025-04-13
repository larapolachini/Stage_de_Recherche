#ifndef OBJECTS_H
#define OBJECTS_H


#include "utils.h"
#include "configuration.h"
#include "render.h"
#include "colormaps.h"


/**
 * @brief Geometry of an object.
 *
 */
class ObjectGeometry {
public:

    /**
     * @brief Construct an ObjectGeometry.
     */
    ObjectGeometry() {}

    /**
     * @brief Destructor
     */
    virtual ~ObjectGeometry();

    /**
     * @brief Create Box2D shape based on this geometry
     */
    virtual void create_box2d_shape(b2BodyId body_id, b2ShapeDef& shape_def) = 0;

    /**
     * @brief Return Box2D shape_id
     */
    b2ShapeId get_shape_id() const { return shape_id; }

    /**
     * @brief Exports a boolean 2D grid showing which bins are covered by the geometry.
     *
     * @param num_bins_x Number of bins along the X-axis.
     * @param num_bins_y Number of bins along the Y-axis.
     * @param bin_width Width (size) of each bin.
     * @param bin_height Height (size) of each bin.
     * @param obj_x The x-coordinate of the object (geometry center).
     * @param obj_y The y-coordinate of the object (geometry center).
     * @return A 2D vector of booleans. True in a given cell indicates that the geometry covers that bin.
     */
    virtual std::vector<std::vector<bool>> export_geometry_grid(size_t num_bins_x,
                                                                  size_t num_bins_y,
                                                                  float bin_width,
                                                                  float bin_height,
                                                                  float obj_x,
                                                                  float obj_y) const = 0;

    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     * @param x X coordinate
     * @param y Y coordinate
     * @param r Red color component
     * @param g Green color component
     * @param b Blue color component
     * @param alpha Alpha color component
     */
    virtual void render(SDL_Renderer* renderer, b2WorldId world_id, float x, float y, uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255) const = 0;

protected:
    bool shape_created = false;
    b2ShapeId shape_id;     ///< Box2D shape identifier.
};

/**
 * @brief Disk-shaped geometry
 *
 */
class DiskGeometry : public ObjectGeometry {
public:

    /**
     * @brief Construct an ObjectGeometry.
     */
    DiskGeometry(float _radius) : radius(_radius) {}

    /**
     * @brief Create Box2D shape based on this geometry.
     */
    virtual void create_box2d_shape(b2BodyId body_id, b2ShapeDef& shape_def) override;

    /**
     * @brief Return radius of the disk.
     */
    float get_radius() const { return radius; }

    /**
     * @brief Exports a boolean 2D grid showing which bins are covered by the geometry.
     *
     * @param num_bins_x Number of bins along the X-axis.
     * @param num_bins_y Number of bins along the Y-axis.
     * @param bin_width Width (size) of each bin.
     * @param bin_height Height (size) of each bin.
     * @param obj_x The x-coordinate of the object (geometry center).
     * @param obj_y The y-coordinate of the object (geometry center).
     * @return A 2D vector of booleans. True in a given cell indicates that the geometry covers that bin.
     */
    virtual std::vector<std::vector<bool>> export_geometry_grid(size_t num_bins_x,
                                                                  size_t num_bins_y,
                                                                  float bin_width,
                                                                  float bin_height,
                                                                  float obj_x,
                                                                  float obj_y) const override;

    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     * @param x X coordinate
     * @param y Y coordinate
     * @param r Red color component
     * @param g Green color component
     * @param b Blue color component
     * @param alpha Alpha color component
     */
    virtual void render(SDL_Renderer* renderer, b2WorldId world_id, float x, float y, uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255) const override;

protected:
    float radius;           ///< Radius of the disk
};

class RectangleGeometry : public ObjectGeometry {
public:
    /**
     * @brief Construct a RectangleGeometry.
     * @param _width The width of the rectangle.
     * @param _height The height of the rectangle.
     */
    RectangleGeometry(float _width, float _height) : width(_width), height(_height) {}

    /**
     * @brief Create a Box2D shape based on this geometry.
     */
    virtual void create_box2d_shape(b2BodyId body_id, b2ShapeDef& shape_def) override;

    /**
     * @brief Exports a boolean 2D grid showing which bins are covered by the rectangle.
     *
     * @param num_bins_x Number of bins along the X-axis.
     * @param num_bins_y Number of bins along the Y-axis.
     * @param bin_width Width (size) of each bin.
     * @param bin_height Height (size) of each bin.
     * @param obj_x The x-coordinate of the object (geometry center).
     * @param obj_y The y-coordinate of the object (geometry center).
     * @return A 2D vector of booleans. True in a given cell indicates that the geometry covers that bin.
     */
    virtual std::vector<std::vector<bool>> export_geometry_grid(size_t num_bins_x,
                                                                  size_t num_bins_y,
                                                                  float bin_width,
                                                                  float bin_height,
                                                                  float obj_x,
                                                                  float obj_y) const override;

    /**
     * @brief Renders the rectangle on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     * @param x X coordinate of the rectangle center.
     * @param y Y coordinate of the rectangle center.
     * @param r Red color component.
     * @param g Green color component.
     * @param b Blue color component.
     * @param alpha Alpha color component.
     */
    virtual void render(SDL_Renderer* renderer, b2WorldId world_id, float x, float y,
                        uint8_t r, uint8_t g, uint8_t b, uint8_t alpha = 255) const override;

    /**
     * @brief Returns the width of the rectangle.
     */
    float get_width() const { return width; }

    /**
     * @brief Returns the height of the rectangle.
     */
    float get_height() const { return height; }

protected:
    float width;   ///< Width of the rectangle.
    float height;  ///< Height of the rectangle.
};


/**
 * @brief Geometry representing the entire simulation.
 *
 */
class GlobalGeometry : public ObjectGeometry {
public:
    /**
     * @brief Construct an ObjectGeometry.
     */
    GlobalGeometry() {}

    /**
     * @brief Create Box2D shape based on this geometry.
     */
    virtual void create_box2d_shape(b2BodyId, b2ShapeDef&) override {};

    /**
     * @brief Exports a boolean 2D grid showing which bins are covered by the geometry.
     *
     * @param num_bins_x Number of bins along the X-axis.
     * @param num_bins_y Number of bins along the Y-axis.
     * @param bin_width Width (size) of each bin.
     * @param bin_height Height (size) of each bin.
     * @param obj_x The x-coordinate of the object (geometry center).
     * @param obj_y The y-coordinate of the object (geometry center).
     * @return A 2D vector of booleans. True in a given cell indicates that the geometry covers that bin.
     */
    virtual std::vector<std::vector<bool>> export_geometry_grid(size_t num_bins_x,
                                                                  size_t num_bins_y,
                                                                  float bin_width,
                                                                  float bin_height,
                                                                  float obj_x,
                                                                  float obj_y) const override;

    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     * @param x X coordinate
     * @param y Y coordinate
     * @param r Red color component
     * @param g Green color component
     * @param b Blue color component
     * @param alpha Alpha color component
     */
    virtual void render(SDL_Renderer*, b2WorldId, float, float, uint8_t, uint8_t, uint8_t, uint8_t = 255) const override {}
};


/**
 * @brief A discretized 2D grid representing light intensities over a simulation area.
 */
class LightLevelMap {
public:
    /**
     * @brief Construct a LightLevelMap.
     * @param num_bins_x Number of bins along the x-axis.
     * @param num_bins_y Number of bins along the y-axis.
     * @param bin_width Physical width of each bin.
     * @param bin_height Physical height of each bin.
     */
    LightLevelMap(size_t num_bins_x, size_t num_bins_y, float bin_width, float bin_height);

    /// Destructor.
    ~LightLevelMap();

    /// Returns the light level stored at the given bin (bin_x, bin_y).
    float get_light_level(size_t bin_x, size_t bin_y) const;

    /// Sets the light level at the given bin (bin_x, bin_y).
    void set_light_level(size_t bin_x, size_t bin_y, int16_t value);

    /// Adds a given value to the light level at the given bin (bin_x, bin_y).
    void add_light_level(size_t bin_x, size_t bin_y, int16_t value);

    /// Resets all bins to 0.
    void clear();

    /// Accessors for the grid properties.
    size_t get_num_bins_x() const;
    size_t get_num_bins_y() const;
    float get_bin_width() const;
    float get_bin_height() const;

private:
    size_t num_bins_x_;
    size_t num_bins_y_;
    float bin_width_;
    float bin_height_;
    std::vector<std::vector<int16_t>> levels_;
};


/**
 * @brief Base class of any object contained within the simulation.
 *
 */
class Object {
public:
    /**
     * @brief Constructs an Object.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param geom Object's geometry.
     * @param _temporal_noise_stddev Standard deviation of the gaussian noise to apply to time on each object, or 0.0 for deterministic time
     */
    Object(uint16_t _id, float _x, float _y,
           ObjectGeometry& _geom,
           float _temporal_noise_stddev = 0.0f);

    /**
     * @brief Constructs an Object from a configuration entry.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param config Configuration entry describing the object properties.
     */
    Object(uint16_t _id, float _x, float _y, Configuration const& config);

    /**
     * @brief Destructor
     */
    virtual ~Object();


    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     */
    virtual void render(SDL_Renderer* renderer, b2WorldId world_id) const = 0;

    /**
     * @brief Launches the user-defined step function.
     *
     * Updates the object's time, enables all registered stop watches, executes the user step
     * function via pogo_main_loop_step, and then disables the stop watches.
     */
    virtual void launch_user_step();

    /**
     * @brief Updates the object's current time.
     *
     * Computes the current time in microseconds relative to the simulation start.
     */
    virtual void update_time();

protected:
    /**
     * @brief Parse a provided configuration and set associated members values.
     *
     * @param config Configuration entry describing the object properties.
     */
    virtual void parse_configuration(Configuration const& config);

    /**
     * @brief Initialize time-related operations
     */
    void initialize_time();

    // Base info
    uint16_t id;                         ///< Object identifier.

    // Physical information
    float x;                             ///< X position
    float y;                             ///< Y position
    ObjectGeometry* geom;                ///< Geometry of the object.

    // Temporal information
    uint64_t current_time_microseconds = 0LL;                        ///< Current time in microseconds.
    float temporal_noise = 0;
    float temporal_noise_stddev;
};



/**
 * @brief StaticLightObject that contributes light to the simulation's light level map.
 */
class StaticLightObject : public Object {
public:

    /**
     * @brief Constructs a StaticLightObject object.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param geom Object's geometry.
     * @param light_map Pointer to the global light level map.
     * @param value Light value between 0 (min) and 32767 (max).
     * @param photo_start_at Change the light value at the specified time to trigger the synchronised photo start of the robots.
     * @param photo_start_duration Amount of time to stay in the photo start stage.
     * @param photo_start_value Light value during the photo start stage.
     * @param _temporal_noise_stddev Standard deviation of the gaussian noise to apply to time on each object, or 0.0 for deterministic time.
     */
    StaticLightObject(uint16_t _id, float _x, float _y,
           ObjectGeometry& _geom, LightLevelMap* light_map,
           int16_t _value, float _photo_start_at = 1.0f, float _photo_start_duration = 1.0f, int16_t _photo_start_value = 32767,
           float _temporal_noise_stddev = 0.0f);

    /**
     * @brief Constructs a StaticLightObject object from a configuration entry.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param light_map Pointer to the global light level map.
     * @param config Configuration entry describing the object properties.
     */
    StaticLightObject(uint16_t _id, float _x, float _y,
            LightLevelMap* light_map, Configuration const& config);

    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     */
    virtual void render(SDL_Renderer*, b2WorldId) const override {}

    /// Updates the object's contribution to the light level map.
    virtual void update_light_map();

protected:
    /**
     * @brief Parse a provided configuration and set associated members values.
     *
     * @param config Configuration entry describing the object properties.
     */
    virtual void parse_configuration(Configuration const& config) override;

    int16_t value;
    LightLevelMap* light_map;  ///< Pointer to the global light level map.
    float photo_start_at;
    float photo_start_duration;
    int16_t photo_start_value;
};


/**
 * @brief A physical object, i.e. with physics properties (e.g. collisions) modelled by Box2D
 *
 */
class PhysicalObject : public Object {
public:

    /**
     * @brief Constructs a PhysicalObject.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param geom Object's geometry.
     * @param world_id The Box2D world identifier.
     * @param _temporal_noise_stddev Standard deviation of the gaussian noise to apply to time on each object, or 0.0 for deterministic time
     * @param _linear_damping Linear damping value for the physical body (default is 0.0f).
     * @param _angular_damping Angular damping value for the physical body (default is 0.0f).
     * @param _density Density of the body shape (default is 10.0f).
     * @param _friction Friction coefficient of the body shape (default is 0.3f).
     * @param _restitution Restitution (bounciness) of the body shape (default is 0.5f).
     */
    PhysicalObject(uint16_t _id, float _x, float _y,
           ObjectGeometry& geom, b2WorldId world_id,
           float _temporal_noise_stddev = 0.0f,
           float _linear_damping = 0.0f, float _angular_damping = 0.0f,
           float _density = 10.0f, float _friction = 0.3f, float _restitution = 0.5f);

    /**
     * @brief Constructs a PhysicalObject from a configuration entry.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param world_id The Box2D world identifier.
     * @param config Configuration entry describing the object properties.
     */
    PhysicalObject(uint16_t _id, float _x, float _y,
           b2WorldId world_id, Configuration const& config);

    /**
     * @brief Retrieves the object's current position.
     *
     * Returns the position of the object's physical body as a Box2D vector.
     *
     * @return b2Vec2 The current position.
     */
    b2Vec2 get_position() const;

    /**
     * @brief Retrieves the object's current orientation angle.
     *
     * Computes and returns the orientation angle (in radians) of the object's body.
     *
     * @return float The orientation angle.
     */
    float get_angle() const;

    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     */
    virtual void render(SDL_Renderer* renderer, b2WorldId world_id) const = 0;


protected:
    /**
     * @brief Parse a provided configuration and set associated members values.
     *
     * @param config Configuration entry describing the object properties.
     */
    virtual void parse_configuration(Configuration const& config) override;

    /**
     * @brief Creates the object's physical body in the simulation.
     *
     * Constructs a dynamic body in the Box2D world at the specified position, defines its shape
     * based on the provided geometry.
     *
     * @param world_id The Box2D world identifier.
     */
    virtual void create_body(b2WorldId world_id);

    // Physical information
    float linear_damping;
    float angular_damping;
    float density;
    float friction;
    float restitution;
    b2BodyId body_id;      ///< Box2D body identifier.
};


/**
 * @brief Physical object without user code (i.e. passive). Can still interact with other objects (e.g. collisions, etc).
 *
 */
class PassiveObject : public PhysicalObject {
public:

    /**
     * @brief Constructs a PassiveObject.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param geom Object's geometry.
     * @param world_id The Box2D world identifier.
     * @param _temporal_noise_stddev Standard deviation of the gaussian noise to apply to time on each object, or 0.0 for deterministic time
     * @param _linear_damping Linear damping value for the physical body (default is 0.0f).
     * @param _angular_damping Angular damping value for the physical body (default is 0.0f).
     * @param _density Density of the body shape (default is 10.0f).
     * @param _friction Friction coefficient of the body shape (default is 0.3f).
     * @param _restitution Restitution (bounciness) of the body shape (default is 0.5f).
     * @param _colormap Name of the colormap to use to set the color of the object
     */
    PassiveObject(uint16_t _id, float _x, float _y,
           ObjectGeometry& geom, b2WorldId world_id,
           float _temporal_noise_stddev = 0.0f,
           float _linear_damping = 0.0f, float _angular_damping = 0.0f,
           float _density = 10.0f, float _friction = 0.3f, float _restitution = 0.5f,
           std::string _colormap = "rainbow");

    /**
     * @brief Constructs a PassiveObject from a configuration entry.
     *
     * @param _id Unique object identifier.
     * @param x Initial x-coordinate in the simulation.
     * @param y Initial y-coordinate in the simulation.
     * @param world_id The Box2D world identifier.
     * @param config Configuration entry describing the object properties.
     */
    PassiveObject(uint16_t _id, float _x, float _y,
           b2WorldId world_id, Configuration const& config);

    /**
     * @brief Renders the object on the given SDL renderer.
     *
     * @param renderer Pointer to the SDL_Renderer.
     * @param world_id The Box2D world identifier (unused in rendering).
     */
    void render(SDL_Renderer* renderer, b2WorldId world_id) const override;

protected:
    std::string colormap;

    /**
     * @brief Parse a provided configuration and set associated members values.
     *
     * @param config Configuration entry describing the object properties.
     */
    virtual void parse_configuration(Configuration const& config) override;
};


/**
 * @brief Factory of ObjectGeometries
 *
 * @param config Configuration entry describing the object properties.
 */
ObjectGeometry* object_geometry_factory(Configuration const& config);


/**
 * @brief Factory of simulation Objects. Return a constructed object from configuration.
 *
 * @param world_id The Box2D world identifier (unused in rendering).
 * @param config Configuration entry describing the object properties.
 * @param light_map Pointer to the global light level map.
 */
Object* object_factory(uint16_t id, float x, float y, b2WorldId world_id, Configuration const& config, LightLevelMap* light_map);

/**
 * @brief Interface to colormaps
 *
 * @param name Name of the colormap.
 * @param value Value to determine the color in this colormap
 * @param r Red color component
 * @param g Green color component
 * @param b Blue color component
 */
void get_cmap_val(std::string const name, uint8_t const value, uint8_t* r, uint8_t* g, uint8_t* b);

#endif


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
