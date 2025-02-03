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

//// Find neighbors for Robot objects
//void find_neighbors(std::vector<Robot>& robots, float maxDistance) {
//    float cellSize = maxDistance; // Cell size based on max neighbor distance
//
//    // Use a pre-allocated map to minimize dynamic allocation during insertion
//    std::unordered_map<GridCell, std::vector<Robot*>, GridCellHash> spatialHash;
//
//    // Assign robots to grid cells
//    for (auto& robot : robots) {
//        GridCell cell = getGridCell(robot.get_position(), cellSize);
//        spatialHash[cell].push_back(&robot);
//    }
//
//    // Compute neighbors
//    for (auto& robot : robots) {
//        GridCell cell = getGridCell(robot.get_position(), cellSize);
//
//        // Set current robot neighbors
//        robot.neighbors.clear();
//
//        for (const auto& offset : precomputedNeighborCells) {
//            GridCell neighborCell{cell.x + offset.x, cell.y + offset.y};
//
//            auto it = spatialHash.find(neighborCell);
//            if (it != spatialHash.end()) {
//                for (const auto& otherRobot : it->second) {
//                    if (robot.id != otherRobot->id) { // Skip itself
//                        float dist = euclidean_distance(robot.get_position(), otherRobot->get_position());
//                        if (dist <= maxDistance) {
//                            robot.neighbors.push_back(otherRobot); // Store directly in neighbors
//                        }
//                    }
//                }
//            }
//        }
//    }
//}


void find_neighbors(std::vector<Robot>& robots, float maxDistance) {
    float cellSize = maxDistance;
    
    // 1) Cache positions for every robot
    std::vector<b2Vec2> positions;
    positions.reserve(robots.size());
    for (auto const& robot : robots) {
        positions.push_back(robot.get_position());
    }

    // 2) Build the grid: use the cached positions for hashing
    std::unordered_map<GridCell, std::vector<int>, GridCellHash> spatialHash;
    for (int i = 0; i < (int)robots.size(); i++) {
        GridCell cell = getGridCell(positions[i], cellSize);
        spatialHash[cell].push_back(i);
    }

    // 3) For each robot, look up neighbors
    for (int i = 0; i < (int)robots.size(); i++) {
        // Clear the neighbor list
        robots[i].neighbors.clear();
        
        GridCell cell = getGridCell(positions[i], cellSize);

        // Check the 9 neighbor cells
        for (auto const &offset : precomputedNeighborCells) {
            GridCell neighborCell{cell.x + offset.x, cell.y + offset.y};

            auto it = spatialHash.find(neighborCell);
            if (it != spatialHash.end()) {
                // For each candidate index
                for (auto const& otherIdx : it->second) {
                    if (i == otherIdx) { 
                        continue; // skip self
                    }
                    // Now do distance check with *cached* positions
                    float dist = euclidean_distance(positions[i], positions[otherIdx]);
                    if (dist <= maxDistance) {
                        // Add a pointer to that Robot
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
