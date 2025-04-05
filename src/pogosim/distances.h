#ifndef DISTANCES_H
#define DISTANCES_H

#include <box2d/box2d.h>
#include <unordered_map>
#include <vector>

#include "utils.h"
#include "spogobot.h"
#include "robot.h"

/**
 * @brief Computes the Euclidean distance between two 2D points.
 *
 * This function calculates the Euclidean distance between the points represented by
 * the Box2D vectors @a a and @a b.
 *
 * @param a The first point as a b2Vec2.
 * @param b The second point as a b2Vec2.
 * @return float The Euclidean distance between @a a and @a b.
 */
float euclidean_distance(const b2Vec2& a, const b2Vec2& b);

/**
 * @brief Finds neighboring robots within a specified maximum distance.
 *
 * This function uses spatial hashing to efficiently determine the neighbors for each robot.
 * It partitions the 2D space into grid cells of size @a maxDistance and assigns each robot
 * to a cell. For every robot, the function then checks the same cell and all adjacent cells
 * for other robots, and updates the robot's neighbor list if the Euclidean distance (squared)
 * is within the allowed maximum distance.
 *
 * @param dir Direction in which messages are sent (i.e. the ID number of the IR emitter)
 * @param robots A vector of Robot objects. Each Robot must implement a method get_position()
 *               returning a b2Vec2, and contain a public member "neighbors" (e.g., a vector)
 *               that can store pointers to neighboring Robot objects.
 * @param maxDistance The maximum distance within which two robots are considered neighbors.
 */
void find_neighbors(ir_direction dir, std::vector<Robot>& robots, float maxDistance);

#endif // DISTANCES_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
