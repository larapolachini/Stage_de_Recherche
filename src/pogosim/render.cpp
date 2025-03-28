#include <SDL2/SDL.h>
#include <box2d/box2d.h>
#include <vector>
#include <random>
#include <iostream>
#include <fstream>
#include <sstream>

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
    if (mm_to_pixels <= 0.10) {
        mm_to_pixels = 0.10;
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


std::vector<std::vector<b2Vec2>> read_poly_from_csv(const std::string& filename, float total_surface) {
    std::vector<std::vector<b2Vec2>> polygons;
    std::ifstream file(filename);
    if (!file.is_open()) {
        glogger->error("Error: Unable to open file {}", filename);
        throw std::runtime_error("Unable to open arena file");
    }

    std::vector<b2Vec2> currentPolygon;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
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
            currentPolygon.push_back({x, y});
        }
    }
    file.close();
    if (!currentPolygon.empty()) {
        polygons.push_back(currentPolygon);
    }

    // Compute the overall bounding box.
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    for (const auto& poly : polygons) {
        for (const auto& point : poly) {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }
    }

    // Normalize all polygons into a [0,1] range.
    std::vector<std::vector<b2Vec2>> normalized_polygons;
    for (const auto& poly : polygons) {
        std::vector<b2Vec2> normPoly;
        for (const auto& point : poly) {
            float normX = (point.x - minX) / (maxX - minX);
            float normY = (point.y - minY) / (maxY - minY);
            normPoly.push_back({normX, normY});
        }
        normalized_polygons.push_back(normPoly);
    }

    if (normalized_polygons.empty()) {
        throw std::runtime_error("No polygons loaded from file.");
    }

    // Compute effective area in normalized space:
    // effective_area = (area of main polygon) - (sum of areas of holes)
    float mainArea = compute_polygon_area(normalized_polygons[0]);
    float holesArea = 0.0f;
    for (size_t i = 1; i < normalized_polygons.size(); i++) {
        holesArea += compute_polygon_area(normalized_polygons[i]);
    }
    float effectiveArea = mainArea - holesArea;
    if (effectiveArea <= 0) {
        throw std::runtime_error("Effective area of polygons is non-positive.");
    }

    // Determine the scale factor s so that:
    // (normalized effective area) * s^2 = total_surface
    float scale = std::sqrt(total_surface / effectiveArea);

    // Apply uniform scaling to all normalized polygons.
    std::vector<std::vector<b2Vec2>> scaled_polygons;
    for (const auto& poly : normalized_polygons) {
        std::vector<b2Vec2> scaledPoly;
        for (const auto& point : poly) {
            scaledPoly.push_back({point.x * scale, point.y * scale});
        }
        scaled_polygons.push_back(scaledPoly);
    }

    return scaled_polygons;
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
            return b2Vec2{x, y};
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
        b2Vec2 norm1 = b2Vec2{-edge1.y, edge1.x};
        b2Vec2 norm2 = b2Vec2{-edge2.y, edge2.x};

        norm1 *= (offset / b2Distance(b2Vec2{0, 0}, edge1));
        norm2 *= (offset / b2Distance(b2Vec2{0, 0}, edge2));

        // Compute the inward offset point using normals
        b2Vec2 offsetPoint = curr + 0.5f * (norm1 + norm2);
        offsetPolygon.push_back(offsetPoint);
    }

    return offsetPolygon;
}


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
            for (size_t i = 1; i < polygons.size(); ++i) {
                const auto& poly = polygons[i];
                if (is_point_within_polygon(poly, x, y)) {
                    valid = false;
                    break;
                }
            }

            // Ensure the point does not overlap with any existing point
            if (valid) {
                for (const auto& existingPoint : points) {
                    if (euclidean_distance(existingPoint, b2Vec2{x, y}) < 2 * minDistance) {
                        valid = false;
                        break;
                    }
                }
            }

            // If the point is valid, add it to the list
            if (valid) {
                points.push_back({x, y});
                attempts = 0;
            } else {
                attempts++;
            }
        }

        // If too many attempts are made, maybe the problem is impossible
        if (attempts >= 1000000LL) {
            throw std::runtime_error("Impossible to create random points within polygon: number of points is too high.");
        }
    }

    return points;
}

std::pair<float, float> compute_polygon_dimensions(const std::vector<b2Vec2>& polygon) {
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

    float width = maxX - minX;
    float height = maxY - minY;
    return std::make_pair(width, height);
}


float compute_polygon_area(const std::vector<b2Vec2>& poly) {
    float area = 0.0f;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        const b2Vec2& p1 = poly[i];
        const b2Vec2& p2 = poly[(i + 1) % n];
        area += p1.x * p2.y - p2.x * p1.y;
    }
    return 0.5f * std::abs(area);
}


// Simple polygon centroid function (using the "shoelace" formula):
b2Vec2 polygon_centroid(const std::vector<b2Vec2>& polygon) {
    // Expecting polygon to be non-empty and closed (first == last) is optional.
    double signedArea = 0.0;
    double cx = 0.0, cy = 0.0;
    for (size_t i = 0; i < polygon.size(); i++) {
        const b2Vec2& p0 = polygon[i];
        const b2Vec2& p1 = polygon[(i+1) % polygon.size()];
        double cross = (p0.x * p1.y) - (p1.x * p0.y);
        signedArea += cross;
        cx += (p0.x + p1.x) * cross;
        cy += (p0.y + p1.y) * cross;
    }
    signedArea *= 0.5;
    cx /= (6.0 * signedArea);
    cy /= (6.0 * signedArea);
    return b2Vec2{(float)cx, (float)cy};
}

// Distance from point to line segment (used for computing min-dist from centroid to edges):
float point_to_line_segment_distance(const b2Vec2& p,
                                     const b2Vec2& a,
                                     const b2Vec2& b) {
    // Vector AP and AB
    float vx = p.x - a.x;
    float vy = p.y - a.y;
    float ux = b.x - a.x;
    float uy = b.y - a.y;
    // Compute the dot product AP·AB
    float dot = vx * ux + vy * uy;
    // Compute squared length of AB
    float len2 = ux * ux + uy * uy;
    // Parameter t along AB to project point P
    float t = (len2 == 0.0f ? 0.0f : dot / len2);
    // Clamp t to [0,1] so we stay in segment
    t = std::max(0.0f, std::min(1.0f, t));
    // Projection point on AB
    float projx = a.x + t * ux;
    float projy = a.y + t * uy;
    // Distance from P to projection
    float dx = p.x - projx;
    float dy = p.y - projy;
    return std::sqrt(dx*dx + dy*dy);
}

std::vector<b2Vec2> generate_regular_disk_points_in_polygon(
        const std::vector<std::vector<b2Vec2>>& polygons,
        float minDistance,
        unsigned int N) {
    // 1. Basic validity checks
    if (polygons.empty()) {
        throw std::runtime_error("No polygons provided.");
    }
    const auto& mainPolygon = polygons.front();
    if (mainPolygon.size() < 3) {
        throw std::runtime_error("Polygon must have at least 3 vertices.");
    }

    // 2. Compute centroid of the *main* polygon
    b2Vec2 center = polygon_centroid(mainPolygon);

    // 3. Find approximate largest inscribed circle radius
    //    by taking min distance from centroid to each edge minus minDistance.
    float minEdgeDist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < mainPolygon.size(); ++i) {
        const b2Vec2& a = mainPolygon[i];
        const b2Vec2& b = mainPolygon[(i + 1) % mainPolygon.size()];
        float dist = point_to_line_segment_distance(center, a, b);
        if (dist < minEdgeDist) {
            minEdgeDist = dist;
        }
    }
    float radius = minEdgeDist - minDistance;
    if (radius <= 0.0f) {
        throw std::runtime_error("Polygon is too small or minDistance too large to fit any disk.");
    }

    // 4. We will place points in a series of concentric "rings" from r=0 to r=radius.
    //    We'll increment radius by ~minDistance each ring to ensure no overlap.
    //    For each ring, we compute the angle increment to maintain ~minDistance spacing along the arc.
    //    If we manage to collect N points, we return them; if we exhaust r > radius without success, we throw.

    std::vector<b2Vec2> result;
    result.reserve(N);

    // First, place the center point if it is valid (sometimes it might be near a hole boundary):
    // We also check the holes to ensure the center is not in a hole.
    if (is_point_within_polygon(mainPolygon, center.x, center.y)) {
        bool inHole = false;
        for (size_t h = 1; h < polygons.size(); ++h) {
            if (is_point_within_polygon(polygons[h], center.x, center.y)) {
                inHole = true;
                break;
            }
        }
        if (!inHole) {
            result.push_back(center);
        }
    }

    // If the center was valid, we might have 1 point placed already.
    // We'll keep going until we have N or we run out of radius.

    float currentRadius = 0.0f;
    const float stepR = minDistance; // radial increment
    // (You can tweak stepR to a fraction of minDistance if you want a denser ring approach.)

    while (result.size() < N && currentRadius <= radius + 1e-6f) {
        currentRadius += stepR;
        if (currentRadius > radius) break; // we've exceeded the inscribed circle

        // On a ring of radius = currentRadius, the arc length between adjacent points is about 2*minDistance
        // so that points along the ring won't overlap with each other.
        // arc_length = r * dTheta => dTheta = arc_length / r.
        // We'll choose arc_length ~ (2*minDistance). You might want smaller arcs if you want to
        // be extra safe about not missing valid placements, but that can slow performance.
        float arcLength = 2.0f * minDistance;
        float dTheta = arcLength / currentRadius;
        if (currentRadius < 1e-6f) {
            // avoid division by zero if very close to center
            dTheta = 2.0f * float(M_PI);
        }

        // We'll place angles from [0, 2π). We have about 2π / dTheta points on this ring.
        // If the ring is extremely small, e.g. near zero radius, we handle that separately above.
        for (float theta = 0.0f; theta < 2.0f * float(M_PI); theta += dTheta) {
            float px = center.x + currentRadius * std::cos(theta);
            float py = center.y + currentRadius * std::sin(theta);

            // 5. Check if this point is inside main polygon & not in a hole
            if (!is_point_within_polygon(mainPolygon, px, py)) continue;
            bool inHole = false;
            for (size_t h = 1; h < polygons.size(); ++h) {
                if (is_point_within_polygon(polygons[h], px, py)) {
                    inHole = true;
                    break;
                }
            }
            if (inHole) continue;

            // 6. Ensure this new point is at least 2*minDistance from all existing points
            //    (so circles of radius minDistance/2 won't overlap).
            bool tooClose = false;
            for (auto& p : result) {
                if (euclidean_distance(p, b2Vec2{px, py}) < 2.0f * minDistance) {
                    tooClose = true;
                    break;
                }
            }
            if (tooClose) continue;

            // 7. Accept this point
            result.push_back({px, py});
            if (result.size() == N) break;
        }
    }

    // If we exit the loop and haven't collected N points, we must throw.
    if (result.size() < N) {
        throw std::runtime_error(
            "Impossible to place N points inside the polygon with the given minDistance constraints."
        );
    }

    return result;
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
