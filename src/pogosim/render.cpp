#include <SDL2/SDL.h>
#include <box2d/box2d.h>
#include <vector>
#include <random>
#include <iostream>

#include "utils.h"
#include "distances.h"
#include "render.h"
#include "spogobot.h"
#include "fpng.h"


float mm_to_pixels = 1.0f;
float visualization_x = 0.0f;
float visualization_y = 0.0f;


void adjust_mm_to_pixels(float delta) {
    mm_to_pixels += delta;
    if (mm_to_pixels <= 0.50) {
        mm_to_pixels = 0.50;
    } else if (mm_to_pixels >= 10.0) {
        mm_to_pixels = 10.0;
    }
}

b2Vec2 visualization_position(float x, float y) {
    b2Vec2 res = {.x = (x + visualization_x) * mm_to_pixels, .y = (y + visualization_y) * mm_to_pixels};
    return res;
}

b2Vec2 visualization_position(b2Vec2 pos) {
    b2Vec2 res = {.x = (pos.x + visualization_x) * mm_to_pixels, .y = (pos.y + visualization_y) * mm_to_pixels};
    return res;
}

std::vector<std::vector<b2Vec2>> read_poly_from_csv(const std::string& filename, float window_width, float window_height) {
    std::vector<std::vector<b2Vec2>> polygons;
    std::ifstream file(filename);
    if (!file.is_open()) {
        glogger->error("Error: Unable to open file {}", filename);
        throw std::runtime_error("Unable to open arena file");
        //return polygons;
    }

    float const width = (window_width - 2 * 30);  // Adjusted width
    float const height = (window_height - 2 * 30);  // Adjusted height

    std::vector<b2Vec2> currentPolygon;
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty()) {
            // Empty line indicates end of current polygon
            if (!currentPolygon.empty()) {
                polygons.push_back(currentPolygon);
                currentPolygon.clear();
            }
            continue;
        }

        std::istringstream ss(line);
        std::string xStr, yStr;
        if (std::getline(ss, xStr, ',') && std::getline(ss, yStr)) {
            float x = std::stof(xStr);
            float y = std::stof(yStr);
            currentPolygon.emplace_back(x, y);
        }
    }
    file.close();

    // Add the last polygon if it exists
    if (!currentPolygon.empty()) {
        polygons.push_back(currentPolygon);
    }

    // Compute min and max for x and y
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    for (auto const& poly : polygons) {
        for (auto const& point : poly) {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }
    }

    // Normalize and scale points
    std::vector<std::vector<b2Vec2>> normalized_polygons;
    std::vector<b2Vec2> normalizedPoints;
    for (auto const& poly : polygons) {
        std::vector<b2Vec2> currentPolygon;
        for (const auto& point : poly) {
            float normX = (point.x - minX) / (maxX - minX); // Min-max normalization for X
            float normY = (point.y - minY) / (maxY - minY); // Min-max normalization for Y
            //normalizedPoints.emplace_back(normX * width, normY * height); // Scale by width and height
            currentPolygon.emplace_back(normX * width, normY * height); // Scale by width and height
        }
        normalized_polygons.push_back(currentPolygon);
    }

    return normalized_polygons;
}


b2Vec2 generate_random_point_within_polygon(const std::vector<b2Vec2>& polygon) {
    if (polygon.size() < 3) {
        throw std::runtime_error("Polygon must have at least 3 points to define a valid area.");
    }

    // Calculate the bounding box of the polygon
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    for (const auto& point : polygon) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(minX, maxX);
    std::uniform_real_distribution<float> disY(minY, maxY);

    // Generate random points until one is inside the polygon
    while (true) {
        float x = disX(gen);
        float y = disY(gen);

        if (is_point_within_polygon(polygon, x, y)) {
            return b2Vec2(x, y);
        }
    }
}

bool is_point_within_polygon(const std::vector<b2Vec2>& polygon, float x, float y) {
    int n = polygon.size();
    if (n < 3) {
        std::cerr << "Error: Polygon must have at least 3 points." << std::endl;
        return false;
    }

    bool isInside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        float xi = polygon[i].x, yi = polygon[i].y;
        float xj = polygon[j].x, yj = polygon[j].y;

        // Check if point is within edge bounds
        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / (yj - yi) + xi);

        if (intersect) {
            isInside = !isInside;
        }
    }
    return isInside;
}

std::vector<b2Vec2> offset_polygon(const std::vector<b2Vec2>& polygon, float offset) {
    std::vector<b2Vec2> offsetPolygon;

    int n = polygon.size();
    if (n < 3) {
        throw std::runtime_error("Polygon must have at least 3 points to offset.");
    }

    for (int i = 0; i < n; ++i) {
        // Get the previous, current, and next points
        b2Vec2 prev = polygon[(i - 1 + n) % n];
        b2Vec2 curr = polygon[i];
        b2Vec2 next = polygon[(i + 1) % n];

        // Calculate vectors for the current edge and the previous edge
        b2Vec2 edge1 = curr - prev;
        b2Vec2 edge2 = next - curr;

        // Normalize and find perpendiculars
        b2Vec2 norm1 = b2Vec2(-edge1.y, edge1.x);
        b2Vec2 norm2 = b2Vec2(-edge2.y, edge2.x);

        norm1 *= (offset / b2Distance(b2Vec2(0, 0), edge1));
        norm2 *= (offset / b2Distance(b2Vec2(0, 0), edge2));

        // Compute the inward offset point using normals
        b2Vec2 offsetPoint = curr + 0.5f * (norm1 + norm2);
        offsetPolygon.push_back(offsetPoint);
    }

    return offsetPolygon;
}

//b2Vec2 generate_random_point_within_polygon_safe(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance) {
//    for (auto const& poly : polygons) {
//        if (poly.size() < 3) {
//            throw std::runtime_error("Polygon must have at least 3 points to define a valid area.");
//        }
//    }
//
//    // Calculate the bounding box of the polygons
//    float minX = std::numeric_limits<float>::max();
//    float maxX = std::numeric_limits<float>::lowest();
//    float minY = std::numeric_limits<float>::max();
//    float maxY = std::numeric_limits<float>::lowest();
//    //std::vector<b2Vec2> innerPolygon0 = offset_polygon(polygons[0], -minDistance);
//    std::vector<b2Vec2> innerPolygon0 = polygons[0];
//    for (const auto& point : innerPolygon0) {
//        minX = std::min(minX, point.x);
//        maxX = std::max(maxX, point.x);
//        minY = std::min(minY, point.y);
//        maxY = std::max(maxY, point.y);
//    }
//    minX += minDistance;
//    minY += minDistance;
//    maxX -= minDistance;
//    maxY -= minDistance;
//
//    // Random number generator
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<float> disX(minX, maxX);
//    std::uniform_real_distribution<float> disY(minY, maxY);
//
//    // Generate random points within the polygon
//    while (true) {
//        float x = disX(gen);
//        float y = disY(gen);
//
//        if (is_point_within_polygon(innerPolygon0, x, y)) {
//            bool within = true;
//            for (auto const& poly : std::span(polygons.begin() + 1, polygons.end())) {
//                if (is_point_within_polygon(poly, x, y)) {
//                    within = false;
//                    break;
//                }
//            }
//            if (within)
//                return b2Vec2(x, y);
//        }
//    }
//}

std::vector<b2Vec2> generate_random_points_within_polygon_safe(const std::vector<std::vector<b2Vec2>>& polygons, float minDistance, unsigned int N) {
    for (const auto& poly : polygons) {
        if (poly.size() < 3) {
            throw std::runtime_error("Polygon must have at least 3 points to define a valid area.");
        }
    }

    // Calculate the bounding box of the primary polygon
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    std::vector<b2Vec2> innerPolygon0 = polygons[0];
    for (const auto& point : innerPolygon0) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }
    minX += minDistance;
    minY += minDistance;
    maxX -= minDistance;
    maxY -= minDistance;

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(minX, maxX);
    std::uniform_real_distribution<float> disY(minY, maxY);

    std::vector<b2Vec2> points; // Store the generated points

    // Generate N random points
    uint32_t attempts = 0;
    while (points.size() < N) {
        float x = disX(gen);
        float y = disY(gen);

        // Check if the point is within the main polygon and outside exclusion polygons
        if (is_point_within_polygon(innerPolygon0, x, y)) {
            bool valid = true;

            // Ensure the point is outside all other polygons
            for (const auto& poly : std::span(polygons.begin() + 1, polygons.end())) {
                if (is_point_within_polygon(poly, x, y)) {
                    valid = false;
                    break;
                }
            }

            // Ensure the point does not overlap with any existing point
            if (valid) {
                for (const auto& existingPoint : points) {
                    if (euclidean_distance(existingPoint, b2Vec2(x, y)) < 2 * minDistance) {
                        valid = false;
                        break;
                    }
                }
            }

            // If the point is valid, add it to the list
            if (valid) {
                points.emplace_back(x, y);
                attempts = 0;
            } else {
                attempts++;
            }
        }

        // If too many attempts are made, maybe the problem is impossible
        if (attempts >= 1000) {
            throw std::runtime_error("Impossible to create random points within polygon: number of points is too high.");
        }
    }

    return points;
}

void draw_polygon(SDL_Renderer* renderer, const std::vector<b2Vec2>& polygon) {
    if (polygon.size() < 3) {
        std::cerr << "Error: Polygon must have at least 3 points to be drawable." << std::endl;
        return;
    }

    // Set the drawing color for the polygon
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red color with full opacity

    // Draw lines between consecutive points
    for (size_t i = 0; i < polygon.size() - 1; ++i) {
        auto const orig_pos = visualization_position(polygon[i].x, polygon[i].y);
        auto const dest_pos = visualization_position(polygon[i+1].x, polygon[i+1].y);
        SDL_RenderDrawLine(renderer,
                           static_cast<int>(orig_pos.x),
                           static_cast<int>(orig_pos.y),
                           static_cast<int>(dest_pos.x),
                           static_cast<int>(dest_pos.y));
    }

    // Connect the last point to the first to close the polygon
    auto const back_pos  = visualization_position(polygon.back().x, polygon.back().y);
    auto const front_pos = visualization_position(polygon.front().x, polygon.front().y);
    SDL_RenderDrawLine(renderer,
                       static_cast<int>(back_pos.x),
                       static_cast<int>(back_pos.y),
                       static_cast<int>(front_pos.x),
                       static_cast<int>(front_pos.y));
}


void save_window_to_png(SDL_Renderer* renderer, SDL_Window* window, const std::string& filename) {
    int width, height;
    SDL_GetWindowSize(window, &width, &height);

    ensure_directories_exist(filename);

    std::vector<Uint8> pixels(width * height * 4); // RGBA format
    if (SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_RGBA32, pixels.data(), width * 4) != 0) {
        std::cerr << "Error reading pixels: " << SDL_GetError() << std::endl;
        return;
    }

//    // Flip the image vertically (SDL's origin is top-left, PNG's is bottom-left)
//    std::vector<Uint8> flippedPixels(width * height * 4);
//    for (int row = 0; row < height; ++row) {
//        std::copy_n(
//            &pixels[(height - row - 1) * width * 4], // Source row (flipped)
//            width * 4,                               // Row size
//            &flippedPixels[row * width * 4]          // Destination row
//        );
//    }

    // Save to PNG
    if (!fpng::fpng_encode_image_to_file(filename.c_str(), pixels.data(), width, height, 4)) {
        glogger->warn("Error writing PNG file '{}'", filename);
    } else {
        glogger->debug("Saved window content to '{}'", filename);
    }
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
