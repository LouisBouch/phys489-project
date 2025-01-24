#pragma once
#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"

namespace physics {
/**
 * @brief Steps the environment forward in time.
 *
 * @param env Environment to advance in time.
 * @param dt Amount of time to step forward. (In seconds)
 */
void stepEnvironment(env::Environment& env, double dt);

/**
 * @brief Steps a single polygon in time.
 *
 * @param polygon Polygon to advance in time.
 * @param dt Amount of time to step forward. (In seconds)
 */
void stepPolygon(env::bodies::Polygon& polygon, double dt);
} // namespace physics
