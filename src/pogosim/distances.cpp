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

// Spatial hashing functions remain unchanged
struct GridCell {
    int x, y;

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

//GridCell getGridCell(const b2Vec2& position, float cellSize) {
//    return {static_cast<int>(std::floor(position.x / cellSize)),
//            static_cast<int>(std::floor(position.y / cellSize))};
//}
//
//std::vector<GridCell> getNeighborCells(const GridCell& cell) {
//    std::vector<GridCell> neighbors;
//    for (int dx = -1; dx <= 1; ++dx) {
//        for (int dy = -1; dy <= 1; ++dy) {
//            neighbors.push_back({cell.x + dx, cell.y + dy});
//        }
//    }
//    return neighbors;
//}
//
////// Find neighbors for Robot objects
//void find_neighbors(std::vector<Robot>& robots, float maxDistance) {
//    float cellSize = maxDistance; // Cell size based on max neighbor distance
//    std::unordered_map<GridCell, std::vector<const Robot*>, GridCellHash> spatialHash;
//
//    // Assign robots to grid cells
//    for (const auto& robot : robots) {
//        GridCell cell = getGridCell(robot.get_position(), cellSize);
//        spatialHash[cell].push_back(&robot);
//    }
//
//    // Compute neighbors
//    for (auto& robot : robots) {
//        GridCell cell = getGridCell(robot.get_position(), cellSize);
//
//        // Check this cell and its neighbors
//        std::vector<GridCell> neighborCells = getNeighborCells(cell);
//        std::set<uint32_t> neighbors;
//
//        for (const auto& neighborCell : neighborCells) {
//            if (spatialHash.find(neighborCell) != spatialHash.end()) {
//                for (const auto& otherRobot : spatialHash[neighborCell]) {
//                    // Compute distance only for nearby robots
//                    if (robot.id != otherRobot->id) { // Skip itself
//                        float dist = euclidean_distance(robot.get_position(), otherRobot->get_position());
//                        if (dist <= maxDistance) {
//                            neighbors.insert(otherRobot->id);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Set current robot neighbors
//        robot.neighbors.clear();
//        for (const auto& id : neighbors) {
//            robot.neighbors.push_back(&robots[id]);
//        }
//    }
//}


// Precompute neighbor cells to avoid frequent dynamic allocation
constexpr std::array<GridCell, 9> precomputedNeighborCells{
    GridCell{-1, -1}, GridCell{-1, 0}, GridCell{-1, 1},
    GridCell{0, -1}, GridCell{0, 0}, GridCell{0, 1},
    GridCell{1, -1}, GridCell{1, 0}, GridCell{1, 1},
};

GridCell getGridCell(const b2Vec2& position, float cellSize) {
    return {static_cast<int>(std::floor(position.x / cellSize)),
            static_cast<int>(std::floor(position.y / cellSize))};
}

// Find neighbors for Robot objects
void find_neighbors(std::vector<Robot>& robots, float maxDistance) {
    float cellSize = maxDistance; // Cell size based on max neighbor distance

    // Use a pre-allocated map to minimize dynamic allocation during insertion
    std::unordered_map<GridCell, std::vector<Robot*>, GridCellHash> spatialHash;

    // Assign robots to grid cells
    for (auto& robot : robots) {
        GridCell cell = getGridCell(robot.get_position(), cellSize);
        spatialHash[cell].push_back(&robot);
    }

    // Compute neighbors
    for (auto& robot : robots) {
        GridCell cell = getGridCell(robot.get_position(), cellSize);

        // Set current robot neighbors
        robot.neighbors.clear();

        for (const auto& offset : precomputedNeighborCells) {
            GridCell neighborCell{cell.x + offset.x, cell.y + offset.y};

            auto it = spatialHash.find(neighborCell);
            if (it != spatialHash.end()) {
                for (const auto& otherRobot : it->second) {
                    if (robot.id != otherRobot->id) { // Skip itself
                        float dist = euclidean_distance(robot.get_position(), otherRobot->get_position());
                        if (dist <= maxDistance) {
                            robot.neighbors.push_back(otherRobot); // Store directly in neighbors
                        }
                    }
                }
            }
        }
    }
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
