#pragma once

#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/collision/Collision.hpp"
#include <eigen3/Eigen/Dense>
#include <memory>
#include <unordered_map>
#include <utility>

namespace physics::collision {
// Hash for a pair of ints.
struct PairHash {
  std::size_t operator()(const std::pair<int, int>& p) const {
    std::size_t h1 = std::hash<int>{}(p.first);
    std::size_t h2 = std::hash<int>{}(p.second);
    return h1 ^ (31 * h2);
  }
};
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
  std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>, PairHash>&
  findCollisions();

  /**
   * @brief Gets the list of collisions in the environment.
   *
   * @return List of collisions.
   */
  std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>, PairHash>&
  getCollisions();

  /**
   * @brief Gets the list of collisions in the environment at the previous
   * timestep.
   *
   * @return List of old collisions.
   */
  std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>, PairHash>&
  getCollisionsOld();

  /**
   * @brief Switches the collision buffer. Old becoems new and new becoems old.
   */
  void switchColBuffer();

  /**
   * @brief Sets the environment the class will work with.
   *
   * @param env Environment.
   */
  void setEnvironment(env::Environment* env);

private:
  env::Environment* env; //< Environment in which collisions will take place.
  std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>, PairHash>
      collisions0; //< List of collisions in the environment. (First buffer)
  std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>, PairHash>
      collisions1;  //< List of collisions in the environment. (Second buffer)
  int curColBuffer; //< Current collision buffer.
  std::pair<int, int> lastAddedCol; //< Key of the last added collision.

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
   * @param subP1I First sub polygon.
   * @param subP2I Second sub polygon.
   * @param ps1 Points of the first triangle.
   * @param ps2 Points of the second triangle.
   *
   * @return True if a collision is present.
   */
  bool subtestSATTria(env::bodies::Polygon& p1, env::bodies::Polygon& p2,
                      int subP1I, int subP2I, const Eigen::Matrix2Xd& ps1,
                      const Eigen::Matrix2Xd& ps2);

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
   * @param subP Reference sub polygon indices.
   *
   * @return Whether a SA was found.
   *
   */
  bool SATHelper(double& minDepth, const Eigen::Matrix2Xd& ps1,
                 const Eigen::Matrix2Xd& ps2, Eigen::Vector2d& n,
                 std::array<int, 2>& refEdge, const Eigen::Matrix2Xd& vs,
                 const std::vector<int>& subP);
};

} // namespace physics::collision
