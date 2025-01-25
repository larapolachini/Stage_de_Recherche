#include "distances.h"

#include <box2d/box2d.h>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <set>
#include <iostream>


float euclidean_distance(const b2Vec2& a, const b2Vec2& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
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

GridCell getGridCell(const b2Vec2& position, float cellSize) {
    return {static_cast<int>(std::floor(position.x / cellSize)),
            static_cast<int>(std::floor(position.y / cellSize))};
}

std::vector<GridCell> getNeighborCells(const GridCell& cell) {
    std::vector<GridCell> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            neighbors.push_back({cell.x + dx, cell.y + dy});
        }
    }
    return neighbors;
}

//// Find neighbors for Robot objects
//std::unordered_map<uint32_t, std::vector<uint32_t>> findNeighbors(
//    const std::vector<Robot>& robots,
//    float maxDistance
//) {
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
//    std::unordered_map<uint32_t, std::vector<uint32_t>> neighborsMap;
//
//    for (const auto& robot : robots) {
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
//        neighborsMap[robot.id] = std::vector<uint32_t>(neighbors.begin(), neighbors.end());
//    }
//
//    return neighborsMap;
//}


void find_neighbors(std::vector<Robot>& robots, float maxDistance) {
    float cellSize = maxDistance; // Cell size based on max neighbor distance
    std::unordered_map<GridCell, std::vector<const Robot*>, GridCellHash> spatialHash;

    // Assign robots to grid cells
    for (const auto& robot : robots) {
        GridCell cell = getGridCell(robot.get_position(), cellSize);
        spatialHash[cell].push_back(&robot);
    }

    // Compute neighbors
    std::unordered_map<uint32_t, std::vector<uint32_t>> neighborsMap;

    for (auto& robot : robots) {
        GridCell cell = getGridCell(robot.get_position(), cellSize);

        // Check this cell and its neighbors
        std::vector<GridCell> neighborCells = getNeighborCells(cell);
        std::set<uint32_t> neighbors;

        for (const auto& neighborCell : neighborCells) {
            if (spatialHash.find(neighborCell) != spatialHash.end()) {
                for (const auto& otherRobot : spatialHash[neighborCell]) {
                    // Compute distance only for nearby robots
                    if (robot.id != otherRobot->id) { // Skip itself
                        float dist = euclidean_distance(robot.get_position(), otherRobot->get_position());
                        if (dist <= maxDistance) {
                            neighbors.insert(otherRobot->id);
                        }
                    }
                }
            }
        }

        // Set current robot neighbors
        robot.neighbors.clear();
        for (const auto& id : neighbors) {
            robot.neighbors.push_back(&robots[id]);
        }

        //neighborsMap[robot.id] = std::vector<uint32_t>(neighbors.begin(), neighbors.end());
    }

    //return neighborsMap;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
