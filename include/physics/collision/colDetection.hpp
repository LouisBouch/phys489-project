#pragma once

#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include <eigen3/Eigen/Dense>

namespace physics::collision {
/**
 * @brief Finds collisions between element in the environment.
 *
 * @param env Environment where collisions will occur.
 *
 */
void findCollisions(env::Environment& env);

/**
 * @brief Test whether a collision is happening between polygons.
 *
 * @param p1 First polygon.
 * @param p2 Second polygon.
 *
 * @return Whether a collision occured.
 *
 */
bool testCollision(const env::bodies::Polygon& p1,
                   const env::bodies::Polygon& p2);
} // namespace physics::collision
