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

// Spatial hashing helpers
struct GridCell {
    int x, y;
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        // Simple hash combining
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

// Precompute the 9 offsets
constexpr std::array<GridCell, 9> precomputedNeighborCells{
    GridCell{-1, -1}, GridCell{-1,  0}, GridCell{-1,  1},
    GridCell{ 0, -1}, GridCell{ 0,  0}, GridCell{ 0,  1},
    GridCell{ 1, -1}, GridCell{ 1,  0}, GridCell{ 1,  1},
};

// Convert a position to a grid cell index
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
