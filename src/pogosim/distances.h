#ifndef DISTANCES_H
#define DISTANCES_H

#include <box2d/box2d.h>
#include <unordered_map>
#include <vector>

#include "utils.h"
#include "spogobot.h"

float euclidean_distance(const b2Vec2& a, const b2Vec2& b);

//std::unordered_map<uint32_t, std::vector<uint32_t>> findNeighbors(
//    const std::vector<Robot&>& robots,
//    float maxDistance);
void find_neighbors(std::vector<Robot>& robots, float maxDistance);

#endif // DISTANCES_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
