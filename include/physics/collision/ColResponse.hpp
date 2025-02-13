#pragma once

#include "physics/collision/Collision.hpp"
#include <eigen3/Eigen/Dense>

namespace physics::collision {
class ColResponse {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param nbIterations Maximum number of times the resolver will iterate over
   * the collisions
   * @param coupled Whether or not to treat the collision manifold as a coupled.
   */
  ColResponse(int nbIterations = 8, bool coupled = false);

  /**
   * @brief Resolves all collisions in the list.
   *
   * @param collisions List of collisions to resolve.
   */
  void resolveCollisions(const std::vector<Collision>& collisions);

private:
  bool coupled; //< Whether or not to treat the collision manifold as a coupled
                // system.
  int nbIterations; //< Maximum number of times the resolver will iterate over
                    // the collisions in order to get a better collision
                    // response.

  /**
   * @brief Given a collision and whether to solve the contact manifold as a
   * coupled system or not, find the impulse magnitude for each contact.
   * j = -v_rel.dot(n) /
   * (1/m_a + 1/m_b + (r_a.cross(n))²/I_a + (r_b.cross(n))²/I_b)
   * Where v_rel = (v_b + w_b.cross(r_b)) - (v_a + w_a.cross(r_a))
   *
   * @param collision The collision containing the contact manifold.
   *
   * @return An impulse magnitude for each contact point on the contact
   * manifold.
   */
  std::vector<double> findImpulseMagnitude(const Collision& collision);
  /**
   * @brief Resolves a single collision.
   *
   * @param collision The collision to be resolved.
   *
   */
  void resolveCollision(const Collision& collision);
};
} // namespace physics::collision
