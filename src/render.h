#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <box2d/box2d.h>

float const VISUALIZATION_SCALE = 100.0f; // 1 Box2D unit = 100 pixels


//std::vector<b2Vec2> read_poly_from_csv(const std::string& filename, float window_width, float window_height);
std::vector<std::vector<b2Vec2>> read_poly_from_csv(const std::string& filename, float window_width, float window_height);

std::vector<b2Vec2> offset_polygon(const std::vector<b2Vec2>& polygon, float offset);
b2Vec2 generate_random_point_within_polygon(const std::vector<b2Vec2>& polygon);
//b2Vec2 generate_random_point_within_polygon_safe(const std::vector<b2Vec2>& polygon, float minDistance);
//b2Vec2 generate_random_point_within_polygon_safe(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance);
std::vector<b2Vec2> generate_random_points_within_polygon_safe(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance, unsigned int N);
bool is_point_within_polygon(const std::vector<b2Vec2>& polygon, float x, float y);
void draw_polygon(SDL_Renderer* renderer, const std::vector<b2Vec2>& polygon);

void save_window_to_png(SDL_Renderer* renderer, SDL_Window* window, const std::string& filename);

#endif // RENDER_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
