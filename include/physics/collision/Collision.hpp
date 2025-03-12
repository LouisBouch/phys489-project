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
   * @param subP1I Sub polygon on the reference polygon where collision occured.
   * @param subP2I Sub polygon on the incident polygon where collision occured.
   * @param n Collision normal. (On reference polygon)
   * @param refEdgePi Reference edge where collision occured.
   *
   * @return Valid collision if one occurs given the parameters.
   */
  static std::optional<Collision> create(env::bodies::Polygon& p1, int subP1I,
                                         env::bodies::Polygon& p2, int subP2I,
                                         const Eigen::Vector2d& n,
                                         std::array<int, 2> refEdgePi);
  /**
   * @brief Gets the first polygon involved in the collision.
   *
   * @return First polygon in the collision.
   */
  env::bodies::Polygon& getFirstPolygon();
  /**
   * @brief Gets the second polygon involved in the collision.
   *
   * @return Second polygon in the collision.
   */
  env::bodies::Polygon& getSecondPolygon();
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

  /**
   * @brief Gets the sub polygon on first polygon where collision occured.
   *
   * @return Sub polygon.
   */
  const int getSubP1I() const;

  /**
   * @brief Gets vertices on the sub polygon that form the reference ege.
   *
   * @return Reference edge indices on sub polygon.
   */
  const std::array<int, 2>& getRefEdgePI() const;

  /**
   * @brief Gets the current accumulated impulse for each contact point in the
   * normal direction.
   *
   * @return Impulse of each contact point.
   */
  const std::vector<double>& getAccNormalImpulse() const;

  /**
   * @brief Gets the current accumulated impulse for each contact point in the
   * tangent direction.
   *
   * @return Impulse of each contact point.
   */
  const std::vector<double>& getAccTangentImpulse() const;

  /**
   * @brief Gets the current accumulated pseudo impulse for each contact point.
   *
   * @return Pseudo impulse of each contact point.
   */
  const std::vector<double>& getAccPseudoImpulse() const;

  /**
   * @brief Adds normal impulse to contact in manifold.
   *
   * @param impulse Magnitude of impulse.
   * @param contactPoint Which of the contact point on the manifold to add an
   * impulse to.
   */
  void addNormalImpulse(double impulse, int contactPoint);

  /**
   * @brief Sets the normal impulse to a specific value.
   *
   * @param impulse Magnitude of impulse.
   */
  void setNormalImpulse(const std::vector<double>& impulse);

  /**
   * @brief Sets the tangent impulse to a specific value.
   *
   * @param impulse Magnitude of impulse.
   */
  void setTangentImpulse(const std::vector<double>& impulse);

  /**
   * @brief Adds tangent impulse to contact in manifold.
   *
   * @param impulse Magnitude of impulse.
   * @param contactPoint Which of the contact point on the manifold to add an
   * impulse to.
   */
  void addTangentImpulse(double impulse, int contactPoint);

  /**
   * @brief Adds pseudo impulse to contact in manifold.
   *
   * @param impulse Magnitude of pseudo impulse.
   * @param contactPoint Which of the contact point on the manifold to add an
   * impulse to.
   */
  void addPseudoImpulse(double impulse, int contactPoint);

  /**
   * @brief Obtain the penetration of the contact point.
   *
   * @param contactPoint Which contact point to use.
   *
   * @return Penetration depth. (Negative means no penetration)
   */
  double contactPenetration(int contactPoint);

  /**
   * @brief Updates the collision after polygons involved have moved.
   *
   * @param p1T Translation of p1.
   * @param p2T Translation of p2.
   * @param p1rot Rotation of p1.
   * @param p2rot Rotation of p2.
   *
   * @return Whether the collision is still happening or not.
   */
  bool updateCollision();

  /**
   * @brief Obtains the average minimum average squared distance between the
   * closest contact points.
   *
   * @param col Collision to compare the distance to.
   *
   * @return The minimum average squared distance between the closest contacts.
   * (Returns infinity if collisions don't match)
   */
  double findDifference(Collision& col);

  /**
   * @return Friction coefficient of the collision.
   */
  double getFrictionCoeff() const;

  /**
   * @return Restitution coefficient of the collision.
   */
  double getRestitutionCoeff() const;
  /**
   * @return Vector containing the target velocities for each contact point.
   */
  const std::vector<double>& getTargetVel() const;
  /**
   * Finds the target velocities of the collision manifold based on the
   * restitution coefficient.
   */
  void findTargetVel();

private:
  std::array<int, 2>
      refEdgePI; //< Indices of sub polygon vertices of edge where collision
                 // occured on reference polygon. Actual polygon vertex index
                 // can be obtained with p.getConvexDecomP[subP1I][refEdge[0]].
                 // (Along the ccw direction.)
  env::bodies::Polygon& p1; //< First colliding polygon. (Reference)
  env::bodies::Polygon& p2; //< Second colliding polygon. (Incident)
  const int
      subP1I; //< Sub polygon on the reference plygon where collision occured.
  const int
      subP2I; //< Sub polygon on the incident plygon where collision occured.
  Eigen::Vector2d n; //< Collision normal. (Lies on p1)
  std::vector<Eigen::Vector2d>
      manifold; //< Contact manifold of the collision. (Can have at most 2
                // points)
  std::vector<double>
      accNormalImpulse; //< Total impulse applied to each point on the manifold
                        // in the normal direction.
  std::vector<double> accPseudoImpulse; //< Total pseudo impulse applied to each
                                        // point on the manifold.
  std::vector<double>
      accTangentImpulse;   //< Total impulse applied to each
                           // point on the manifold in the tangent direction.
  double frictionCoeff;    //< Friction coefficient of the collision.
  double restitutionCoeff; //< Friction coefficient of the collision.
  std::vector<double> targetVel; //< Target velocities of the contact manifold.

  /**
   * @brief Represents a collision between two polygons.
   *
   * @param p1 First colliding polygon. (Reference)
   * @param p2 Second colliding polygon. (Incident)
   * @param subP1I Sub polygon on the reference polygon where collision occured.
   * @param subP2I Sub polygon on the incident polygon where collision occured.
   * @param n Collision normal. (On reference polygon)
   * @param refEdgePi Reference edge where collision occured.
   * @param manifold Contact manifold of the collision.
   */
  Collision(env::bodies::Polygon& p1, int subP1I, env::bodies::Polygon& p2,
            int subP2I, const Eigen::Vector2d& n, std::array<int, 2> refEdgePi,
            std::vector<Eigen::Vector2d> manifold);
  /**
   * @brief Obtains the incident edge of the collision.
   *
   * @return Valid incident edge of collision if one exists. (Invalid if inside
   * edge of polygon)
   */
  static std::optional<std::vector<Eigen::Vector2d>>
  findIncEdge(const env::bodies::Polygon& p1, int subP1I,
              const env::bodies::Polygon& p2, int subP2I,
              const Eigen::Vector2d& n, std::array<int, 2> refEdgePi);
  /**
   * @brief Obtains the contact manifold of the intersecting polygons.
   *
   * @return Contact manifold of the collision.
   */
  static std::optional<std::vector<Eigen::Vector2d>>
  findManifold(env::bodies::Polygon& p1, int subP1I, env::bodies::Polygon& p2,
               int subP2I, const Eigen::Vector2d& n,
               std::array<int, 2> refEdgeTi);
};
} // namespace physics::collision
