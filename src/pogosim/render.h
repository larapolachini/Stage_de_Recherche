#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <box2d/box2d.h>

float const VISUALIZATION_SCALE = 100.0f; // 1 Box2D unit = 100 pixels
extern float mm_to_pixels;
extern float visualization_x;
extern float visualization_y;

void adjust_mm_to_pixels(float delta);
b2Vec2 visualization_position(float x, float y);
b2Vec2 visualization_position(b2Vec2 pos);

/**
 * @brief Reads polygons from a CSV file and scales them to match a specified surface area.
 *
 * This function loads polygons from a CSV file where each line contains the x and y
 * coordinates separated by a comma. An empty line indicates the end of a polygon.
 * Once the polygons are loaded, the function normalizes all points into the [0,1] range
 * using the overall bounding box of the data. The effective area is then computed as the
 * area of the first (main) polygon minus the sum of the areas of any subsequent polygons
 * (considered holes). A uniform scaling factor is determined so that when applied to the
 * normalized polygons, the effective area becomes equal to the specified total_surface.
 *
 * @param filename The path to the CSV file containing the polygon vertices.
 * @param total_surface The desired surface area for the main polygon after subtracting the holes.
 * @return std::vector<std::vector<b2Vec2>> A vector containing the scaled polygons, where each
 *         polygon is represented as a vector of b2Vec2 points.
 *
 * @throw std::runtime_error If the file cannot be opened, no polygons are loaded, or the
 *         effective area (main polygon area minus holes area) is non-positive.
 */
std::vector<std::vector<b2Vec2>> read_poly_from_csv(const std::string& filename, float total_surface);

std::vector<b2Vec2> offset_polygon(const std::vector<b2Vec2>& polygon, float offset);
b2Vec2 generate_random_point_within_polygon(const std::vector<b2Vec2>& polygon);
//b2Vec2 generate_random_point_within_polygon_safe(const std::vector<b2Vec2>& polygon, float minDistance);
//b2Vec2 generate_random_point_within_polygon_safe(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance);
std::vector<b2Vec2> generate_random_points_within_polygon_safe(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance, unsigned int N);
bool is_point_within_polygon(const std::vector<b2Vec2>& polygon, float x, float y);


/**
 * @brief Computes the width and height of a polygon.
 *
 * This function calculates the dimensions of a polygon by determining the minimum
 * and maximum x and y coordinates of its vertices. The width is computed as the
 * difference between the maximum and minimum x values, and the height is the difference
 * between the maximum and minimum y values.
 *
 * @param polygon A vector of b2Vec2 points representing the vertices of the polygon.
 * @return std::pair<float, float> A pair where the first element is the width and the second element is the height of the polygon.
 */
std::pair<float, float> compute_polygon_dimensions(const std::vector<b2Vec2>& polygon);

/**
 * @brief Computes the area of a polygon using the shoelace formula.
 *
 * This function calculates the area of a polygon defined by a sequence of
 * points (b2Vec2) using the shoelace algorithm. The formula sums the cross
 * products of consecutive vertices and returns half of the absolute value.
 *
 * @param poly A vector of b2Vec2 points representing the vertices of the polygon.
 * @return float The computed area of the polygon.
 */
float compute_polygon_area(const std::vector<b2Vec2>& poly);

b2Vec2 polygon_centroid(const std::vector<b2Vec2>& polygon);
float point_to_line_segment_distance(const b2Vec2& p, const b2Vec2& a, const b2Vec2& b);
std::vector<b2Vec2> generate_regular_disk_points_in_polygon(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance, unsigned int N);

void draw_polygon(SDL_Renderer* renderer, const std::vector<b2Vec2>& polygon);
void save_window_to_png(SDL_Renderer* renderer, SDL_Window* window, const std::string& filename);

#endif // RENDER_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
