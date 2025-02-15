#pragma once

#include "env/bodies/Polygon.hpp"
#include "physics/collision/Collision.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

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
   * @param dt Time delta of the physics engine.
   */
  void resolveCollisions(std::vector<Collision>& collisions, double dt);

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
   * @param c Contact point for which to find magnitude.
   * @param targetVel Target velocity of the constraint. (Positive value to pull
   * objects apart)
   *
   * @return An impulse magnitude for the specified contact point.
   */
  double findImpulseMagnitude(Collision& collision, int c,
                              double targetVel = 0);
  /**
   * @brief Resolves a single collision velocity constraint.
   *
   * @param collision The collision to be resolved.
   *
   */
  void enforceVelConstraint(Collision& collision);

  /**
   * @brief Applies impulse to contact point.
   *
   * @param impulse Impulse magnitude.
   * @param n Normal of collision.
   * @param p1 First polygon colliding.
   * @param p2 Second polygon colliding.
   * @param r1 Vector from centroid of first polygon to contact point.
   * @param r2 Vector from centroid of second polygon to contact point.
   *
   */
  void applyImpulse(double impulse, const Eigen::Vector2d& n,
                    env::bodies::Polygon& p1, env::bodies::Polygon& p2,
                    const Eigen::Vector2d& r1, const Eigen::Vector2d& r2);

  /**
   * @brief Applies impulse of contact point to position directly.
   *
   * @param impulse Impulse magnitude.
   * @param n Normal of collision.
   * @param p1 First polygon colliding.
   * @param p2 Second polygon colliding.
   * @param r1 Vector from centroid of first polygon to contact point.
   * @param r2 Vector from centroid of second polygon to contact point.
   * @param dt Time delta of the physics engine.
   * @param collision Collision object affected by changed in position.
   *
   */
  void applyImpulseDirectly(double impulse, const Eigen::Vector2d& n,
                            env::bodies::Polygon& p1, env::bodies::Polygon& p2,
                            const Eigen::Vector2d& r1,
                            const Eigen::Vector2d& r2, double dt,
                            Collision& collision);
  /**
   * @brief Resolves a single collision position constraint.
   *
   * @param collision The collision to be resolved.
   * @param dt Time delta of the physics engine.
   * @param steerConst The steering constant. Says how much of the error to
   * correct.
   */
  void enforcePosConstraint(Collision& collision, double dt, double steerConst);
};
} // namespace physics::collision
