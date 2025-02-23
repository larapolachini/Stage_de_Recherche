#include "distances.h"

#include <box2d/box2d.h>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <set>
#include <iostream>


float euclidean_distance(const b2Vec2& a, const b2Vec2& b) {
    float const dx = a.x - b.x;
    float const dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Represents a cell in a spatial grid.
 *
 * A GridCell is defined by its integer coordinates (x, y) and is used in spatial
 * hashing to partition a 2D space into discrete cells.
 */
struct GridCell {
    int x, y; ///< The x and y coordinates of the grid cell.

    /**
     * @brief Compares two GridCell objects for equality.
     *
     * @param other The other GridCell to compare against.
     * @return true if both the x and y coordinates are equal.
     * @return false otherwise.
     */
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief Hash functor for GridCell.
 *
 * This structure provides a hash function for GridCell objects, allowing them to be used
 * as keys in unordered associative containers.
 */
struct GridCellHash {
    /**
     * @brief Computes a hash value for a GridCell.
     *
     * Combines the hash of the x and y coordinates.
     *
     * @param cell The GridCell to hash.
     * @return std::size_t The computed hash value.
     */
    std::size_t operator()(const GridCell& cell) const {
        // Simple hash combining for x and y.
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

/**
 * @brief Precomputed offsets for neighbor grid cells.
 *
 * This constant array contains the relative offsets for a cell's neighbors in a 3x3 grid,
 * including the cell itself. It is used to quickly access adjacent cells during spatial queries.
 */
constexpr std::array<GridCell, 9> precomputedNeighborCells{
    GridCell{-1, -1}, GridCell{-1,  0}, GridCell{-1,  1},
    GridCell{ 0, -1}, GridCell{ 0,  0}, GridCell{ 0,  1},
    GridCell{ 1, -1}, GridCell{ 1,  0}, GridCell{ 1,  1},
};

/**
 * @brief Converts a 2D position to a grid cell index.
 *
 * This inline function maps the provided (x, y) coordinates into a GridCell based on
 * the specified cell size. It uses std::floor to determine the appropriate cell index.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param cellSize The size of a single grid cell.
 * @return GridCell The corresponding grid cell for the given position.
 */
inline GridCell getGridCell(float x, float y, float cellSize) {
    return {static_cast<int>(std::floor(x / cellSize)),
            static_cast<int>(std::floor(y / cellSize))};
}

void find_neighbors(std::vector<Robot>& robots, float maxDistance) {
    const float cellSize = maxDistance;
    const float maxDistSq = maxDistance * maxDistance;
    const size_t N = robots.size();

    // 1) Build separate x and y arrays (SoA layout).
    //    This avoids calling get_position() repeatedly and also helps cache locality.
    std::vector<float> xs(N), ys(N);
    for (size_t i = 0; i < N; i++) {
        b2Vec2 pos = robots[i].get_position();
        xs[i] = pos.x;
        ys[i] = pos.y;
    }

    // 2) Build a spatial hash from GridCell -> list of robot indices.
    std::unordered_map<GridCell, std::vector<size_t>, GridCellHash> spatialHash;
    spatialHash.reserve(N); // optional, to reduce rehashes if you know approximate N

    for (size_t i = 0; i < N; i++) {
        GridCell cell = getGridCell(xs[i], ys[i], cellSize);
        spatialHash[cell].push_back(i);
    }

    // 3) For each robot, find neighbors in the same or adjacent cells.
    for (size_t i = 0; i < N; i++) {
        // Clear the old neighbor list
        robots[i].neighbors.clear();

        GridCell cell = getGridCell(xs[i], ys[i], cellSize);

        for (const auto& offset : precomputedNeighborCells) {
            GridCell neighborCell{ cell.x + offset.x, cell.y + offset.y };
            auto it = spatialHash.find(neighborCell);
            if (it != spatialHash.end()) {
                // For each candidate robot in this neighbor cell
                for (size_t otherIdx : it->second) {
                    if (otherIdx == i) {
                        continue; // skip self
                    }
                    // 4) Compare squared distances to avoid sqrt.
                    float dx = xs[i] - xs[otherIdx];
                    float dy = ys[i] - ys[otherIdx];
                    float distSq = dx*dx + dy*dy;
                    if (distSq <= maxDistSq) {
                        // Robot i and otherIdx are neighbors
                        robots[i].neighbors.push_back(&robots[otherIdx]);
                    }
                }
            }
        }
    }
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
