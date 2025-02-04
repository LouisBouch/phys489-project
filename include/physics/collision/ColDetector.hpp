#pragma once

#define SAT true
#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/collision/Collision.hpp"
#include <eigen3/Eigen/Dense>

namespace physics::collision {
class ColDetector {
public:
  /**
   * @brief Default constructor.
   *
   * @param env Environment in which to detect collisions.
   */
  ColDetector(env::Environment* env = nullptr);

  /**
   * @brief Finds collisions between element in the environment.
   *
   * @param env Environment where collisions will occur.
   *
   */
  std::vector<physics::collision::Collision>& findCollisions();

  /**
   * @brief Gets the list of collisions in the environment.
   *
   * @return List of collisions.
   */
  const std::vector<Collision>& getCollisions() const;

  /**
   * @brief Sets the environment the class will work with.
   *
   * @param env Environment.
   */
  void setEnvironment(env::Environment* env);

private:
  env::Environment* env; //< Environment in which collisions will take place.
  std::vector<Collision> collisions; //< List of collisions in the environment.

  /**
   * @brief Test whether a collision is happening between polygons using
   * edge-edge detection.
   *
   * @param p1 First polygon.
   * @param p2 Second polygon.
   *
   * @return Whether a collision occured.
   *
   */
  bool testEdgeCollisions(env::bodies::Polygon& p1, env::bodies::Polygon& p2);
  /**
   * @brief Test whether a collision is happening between polygons using SAT.
   *
   * @param p1 First polygon.
   * @param p2 Second polygon.
   *
   * @return Whether a collision occured.
   *
   */
  bool testSATConcave(env::bodies::Polygon& p1, env::bodies::Polygon& p2);

  /**
   * @brief Test whether a collision is happening between triangles using SAT.
   * If so, add it to the list of collisions. Use valid edge as reference edge.
   * (Valid implies vertices forming the separating axis form an outside edge.)
   *
   * @param p1 First polygon.
   * @param p2 Second polygon.
   * @param t1 First triangle indices.
   * @param t2 Second triangle indices.
   * @param ps1 Points of the first triangle.
   * @param ps2 Points of the second triangle.
   *
   * @return True if a collision is present.
   */
  bool subtestSATTria(env::bodies::Polygon& p1, env::bodies::Polygon& p2,
                      const Eigen::Vector3i& t1, const Eigen::Vector3i& t2,
                      const Eigen::Matrix2Xd& ps1, const Eigen::Matrix2Xd& ps2);

  /**
   * @brief Helper function for subtestSATTria. Does half the SAT test.
   *
   * @param minDepth Minimum penetration depth between outside edge and triangle
   * points.
   * @param ps1 Vertices of polygon triangle on which SA are chosen.
   * @param ps2 Vertices of second polygon triangle to check penetration.
   * @param n Normal of collision vector.
   * @param refEdge Valid edge where collision is occuring.
   * @param vs
   * @param t Reference triangle indices.
   *
   * @return Whether a SA was found.
   *
   */
  bool SATHelper(double& minDepth, const Eigen::Matrix2Xd& ps1, const Eigen::Matrix2Xd& ps2,
                 Eigen::Vector2d& n, std::array<int, 2>& refEdge,
                 const Eigen::Matrix2Xd& vs, const Eigen::Vector3i& t);
};

} // namespace physics::collision
