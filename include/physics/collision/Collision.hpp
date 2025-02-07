#pragma once

#include "eigen3/Eigen/Dense"
#include "env/bodies/Polygon.hpp"
#include <optional>
namespace physics::collision {
class Collision {
public:
  /**
   * @brief Copy constructor.
   *
   * @param col Collision to copy
   */
  Collision(const Collision& col);
  /**
   * @brief Factory for collisions.
   *
   * @param p1 First colliding polygon. (Reference)
   * @param p2 Second colliding polygon. (Incident)
   * @param t1 Triangle on the reference polygon where collision occured.
   * @param t2 Triangle on the incident polygon where collision occured.
   * @param n Collision normal. (On reference polygon)
   * @param depth Collision depth.
   * @param refEdgeTi Reference edge where collision occured.
   *
   * @return Valid collision if one occurs given the parameters.
   */
  static std::optional<Collision>
  create(const env::bodies::Polygon& p1, const Eigen::Vector3i& t1,
         const env::bodies::Polygon& p2, const Eigen::Vector3i& t2,
         const Eigen::Vector2d& n, double depth, std::array<int, 2> refEdgeTi);
  /**
   * @brief Gets the first polygon involved in the collision.
   *
   * @return First polygon in the collision.
   */
  const env::bodies::Polygon& getFirstPolygon() const;
  /**
   * @brief Gets the second polygon involved in the collision.
   *
   * @return Second polygon in the collision.
   */
  const env::bodies::Polygon& getSecondPolygon() const;
  /**
   * @brief Gets the collision normal.
   *
   * @return Collision normal.
   */
  double getDepth() const;
  /**
   * @brief Gets the collision normal.
   *
   * @return Collision normal.
   */
  const Eigen::Vector2d& getNormal() const;
  /**
   * @brief Gets the contact manifold of the collision.
   *
   * @return Array containing all contact points.
   */
  const std::vector<Eigen::Vector2d> getManifold() const;

private:
  std::array<int, 2>
      refEdgeTi; //< Indices of triangle vertices of edge where collision
                 // occured on reference polygon. Actual polygon vertex index
                 // can be obtained with t1[refEdge[0]]. (Along the ccw
                 // direction.)
  const env::bodies::Polygon& p1; //< First colliding polygon. (Reference)
  const env::bodies::Polygon& p2; //< Second colliding polygon. (Incident)
  const Eigen::Vector3i&
      t1; //< Triangle on the reference plygon where collision occured.
  const Eigen::Vector3i&
      t2; //< Triangle on the incident plygon where collision occured.
  Eigen::Vector2d n; //< Collision normal. (Lies on p1)
  double depth;      //< Collision depth.
  std::vector<Eigen::Vector2d>
      manifold; //< Contact manifold of the collision. (Can have at most 2
                // points)

  /**
   * @brief Represents a collision between two polygons.
   *
   * @param p1 First colliding polygon. (Reference)
   * @param p2 Second colliding polygon. (Incident)
   * @param t1 Triangle on the reference polygon where collision occured.
   * @param t2 Triangle on the incident polygon where collision occured.
   * @param n Collision normal. (On reference polygon)
   * @param depth Collision depth.
   * @param refEdgeTi Reference edge where collision occured.
   */
  Collision(const env::bodies::Polygon& p1, const Eigen::Vector3i& t1,
            const env::bodies::Polygon& p2, const Eigen::Vector3i& t2,
            const Eigen::Vector2d& n, double depth, std::array<int, 2> refEdgeTi,
            std::vector<Eigen::Vector2d> manifold);
  /**
   * @brief Obtains the incident edge of the collision.
   *
   * @return Valid incident edge of collision if one exists. (Invalid if inside
   * edge of polygon)
   */
  static std::optional<std::vector<Eigen::Vector2d>>
  findIncEdge(const env::bodies::Polygon& p1, const Eigen::Vector3i& t1,
              const env::bodies::Polygon& p2, const Eigen::Vector3i& t2,
              const Eigen::Vector2d& n, double depth, std::array<int, 2> refEdgeTi);
  /**
   * @brief Obtains the contact manifold of the intersecting polygons.
   *
   * @return Contact manifold of the collision.
   */
  static std::optional<std::vector<Eigen::Vector2d>>
  findManifold(const env::bodies::Polygon& p1, const Eigen::Vector3i& t1,
               const env::bodies::Polygon& p2, const Eigen::Vector3i& t2,
               const Eigen::Vector2d& n, double depth, std::array<int, 2> refEdgeTi);
};
} // namespace physics::collision
