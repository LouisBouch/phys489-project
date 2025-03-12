#pragma once

#include "env/bodies/Polygon.hpp"
#include "physics/collision/ColDetector.hpp"
#include "physics/collision/Collision.hpp"
#include <eigen3/Eigen/Dense>
#include <memory>

namespace physics::collision {
class ColResponse {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param nbIterations Maximum number of times the resolver will iterate over
   * the collisions for impulse velocities.
   * @param nbPosIt Maximum number of times the resolver will iterate over
   * the collisions for pseudo impulses.
   * @param minRelVel Minimum relaxation of velocity for the last iteration.
   * @param minRelFric Minimum relaxation of friction for the last iteration.
   */
  ColResponse(int nbIterations = 8, int nbPosIt = 4, double minRelVel = 0.95,
              double minRelFric = 0.85);
  /**
   * @brief Resolves all collisions in the list.
   *
   * @param collisions List of collisions to resolve.
   * @param dt Time delta of the physics engine.
   */
  void resolveCollisions(
      std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>,
                         PairHash>& collisions,
      double dt);
  /**
   * @brief Applies warmstarting to a collision.
   *
   * @param collision Collision to warm start.
   */
  void warmStart(Collision& collision);

private:
  bool coupled; //< Whether or not to treat the collision manifold as a coupled
                // system.
  int nbVelIt;  //< Maximum number of times the resolver will iterate over
                // the collisions in order to get a better velocity response.
  int nbPosIt;  //< Maximum number of times the resolver will iterate over
                // the collisions in order to get a better position response.
  double relaxVel;   //< Relaxation factor for velocity.
  double relaxFric;  // Relaxation factor for friction.
  double minRelVel;  //< Minimum relaxation of velocity for the last iteration.
  double minRelFric; //< Minimum relaxation of friction for the last iteration.
  double relVelR;    //< Ratio of velocity relaxation factor to keep after each
                     // iteration.
  double relFricR;   //< Ratio of friction relaxation factor to keep after each
                     // iteration. Tuned in order to reach minRelFric after all
                     // the iterations.

  /**
   * @brief Given a collision, find the impulse magnitude for each contact.
   * j = -v_rel.dot(n) /
   * (1/m_a + 1/m_b + (r_a.cross(n))²/I_a + (r_b.cross(n))²/I_b)
   * Where v_rel = (v_b + w_b.cross(r_b)) - (v_a + w_a.cross(r_a))
   *
   * This comes from solving:
   * J^T (V + M^-1 J * j) = V_target
   *
   * @param collision The collision containing the contact manifold.
   * @param n Vector along which the velocity contraint must be met.
   * @param c Contact point for which to find magnitude.
   * @param targetVel Target velocity of the constraint. (Positive value to pull
   * objects apart)
   * @param resitution When not negative, overrides targetVel to be a fraction
   * of the relative velocity at the contat point. Must be between 0 and 1.
   *
   * @return An impulse magnitude for the specified contact point.
   */
  double findImpulseMagnitude(Collision& collision, const Eigen::Vector2d& n,
                              int c, double targetVel = 0,
                              double restitution = -1);
  /**
   * @brief Given a collsion, find the impulse as a coupled system.
   *
   * @param collision The collision containing the contact manifold.
   * @param n Vector along which the velocity contraint must be met.
   * @param targetVel Target velocity of the constraint. (Positive value to pull
   * objects apart)
   *
   * Solves the system:
   *
   *                    1                     2                  3
   * (J_1^T V + J_1^T M^-1 J_1 * j_1 + J_1^T M^-1 J_2 * j_2) = V_target1
   * (J_2^T V + J_2^T M^-1 J_1 * j_1 + J_2^T M^-1 J_2 * j_2) = V_target2
   *
   * |J_1^T V| + |J_1^T M^-1 J_1  J_1^T M^-1 J_2| |j_1| = V_target1
   * |J_2^T V| + |J_2^T M^-1 J_1  J_2^T M^-1 J_2| |j_2| = V_target2
   *
   * m represents rows.
   *
   * Where
   * V^T = [V_a^T, W_a^T, V_b^T, W_b^T]
   * J^T = [-n^T, -(r_a x n)^T, n^T, (r_b x n)^T] (Jacobian for a given contact
   * point) M = diagonal 4x4 with value [1/m_a, 1/I_a, 1/m_b, 1/I_b] on the
   * diagonal.
   *
   * 1. The relative velocity at contact point m caused by impulse 1.
   * 2. The relative velocity at contact point m caused by impulse 2.
   * 3. Target relative velocity at contact point m.
   *
   * @return An impulse magnitude for the contact manifold.
   */
  // TODO: Find a way to stabilize ill conditioned matrices without introducing
  // errors.
  std::vector<double>
  findImpulseMagnitudeCoupled(Collision& collision, const Eigen::Vector2d& n,
                              const Eigen::Vector2d& targetVel = {0, 0});
  /**
   * @brief Given a collsion contact, find the impulse as a coupled system.
   *
   * @param collision The collision containing the contact manifold.
   * @param n1 Vector along which the first contraint must be met.
   * @param n2 Vector along which the second contraint must be met.
   * @param c The point on the contact manifold.
   * @param targetVel Target velocity of the constraint. (First value relates to
   * first direction vector)
   *
   * @return An impulse magnitude for the contact manifold.
   */
  // TODO: Enforce complementary restrictions (Convert system to LCP). Right now
  // it does not take them into account which yields tthe wrong answer under
  // certain conditions.
  std::vector<double>
  findImpulseMagnitudeCoupledSingle(Collision& collision,
                                    const Eigen::Vector2d& n1,
                                    const Eigen::Vector2d& n2, int c,
                                    const Eigen::Vector2d& targetVel = {0, 0});
  /**
   * @brief Resolves a single collision velocity constraint.
   *
   * @param collision The collision to be resolved.
   *
   */
  void enforceVelConstraint(Collision& collision);

  /**
   * @brief Resolves a single collision velocity and friction constraint.
   * (Coupled)
   *
   * @param collision The collision to be resolved.
   *
   */
  void enforceVelFricConstraint(Collision& collision);

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
                            const Eigen::Vector2d& r2, double dt);
  /**
   * @brief Resolves a single collision position constraint.
   *
   * @param collision The collision to be resolved.
   * @param dt Time delta of the physics engine.
   * @param steerConst The steering constant. Says how much of the error to
   * correct.
   */
  void enforcePosConstraint(Collision& collision, double dt, double steerConst);

  /**
   * @brief Resolves a single collision friction constraint.
   *
   * @param collision The collision to be resolved.
   */
  void enforceFrictionConstraint(Collision& collision);
};
} // namespace physics::collision
