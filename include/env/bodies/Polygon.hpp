#pragma once

#include "physics/forces/Force.hpp"
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace env::bodies {
class Polygon {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param vertices Vertices of the polygon in COUNTERCLOCKWISE order.
   */
  Polygon(const Eigen::Matrix2Xd& vertices, double rot = 0, double angV = 0,
          const Eigen::Vector2d velocity = Eigen::Vector2d{0, 0},
          double density = 1, double restitutionCoef = 0,
          double frictionCoef = 1, bool stationary = false, int id = -1, int tag = -1);

  /**
   * @brief Copy constructor.
   *
   * @param polygon Polygon to copy.
   */
  Polygon(const Polygon& polygon);

  /**
   * @brief The deconstructor.
   */
  ~Polygon();

  /**
   * @brief Translates the polygon.
   *
   * @param t Translation vector.
   */
  void translate(const Eigen::Vector2d& t);

  /**
   * @brief Rotates the polygon around its centroid (center of mass).
   *
   * @param r Rotation angle (Radian).
   */
  void rotate(double r);

  /**
   * @brief Obtains current rotation of polygon.
   *
   * @return Rotation angle (Radian).
   */
  double getRotation() const;

  /**
   * @brief Obtains reference to current rotation of polygon.
   *
   * @return Constant reference to rotation angle (Radian).
   */
  const double& getRotationR() const;

  /**
   * @brief Gets the area of the polygon.
   *
   * @return Area of the polygon.
   */
  double getArea() const;

  /**
   * @brief Gets the perimeter of the polygon.
   *
   * @return Perimeter of the polygon.
   */
  double getPerimeter() const;

  /**
   * @brief Gets the centroid of the polygon.
   *
   * @return Centroid (center of mass) location of polygon.
   */
  const Eigen::Vector2d& getCentroid() const;

  /**
   * @brief Gets the vertices of the polygon in global coordinates. (Rotates
   * then translates local vertices)
   *
   * @return Matrix of vertices of polygon in global coordinates.
   */
  const Eigen::Matrix2Xd& getGlobalVertices() const;

  /**
   * @brief Gets the separation axes of the polygon.
   *
   * @return Matrix containing separation axes.
   */
  Eigen::Matrix2Xd getSeparationAxes() const;

  /**
   * @brief Gets the vertices of the polygon in local coordinates (With respect
   * to centroid).
   *
   * @return Matrix of vertices of polygon in local coordinates.
   */
  const Eigen::Matrix2Xd& getLocalVertices() const;

  /**
   * @brief Gets the number of vertices on the polygon.
   *
   * @return Number of vertices.
   */
  int getNbVertices() const;

  /**
   * @brief Gets current centroid velocity.
   *
   * @return Velocity of the centroid. (m/s)
   */
  const Eigen::Vector2d& getVelocity() const;

  /**
   * @brief Gets current polygon angular velocity.
   *
   * @return Angular velocity of polygon.
   * (rad/s, positive for counterclockwise).
   */
  double getAngV() const;

  /**
   * @brief Adds value to current centroid velocity.
   *
   * @param v Velocity to add to current velocity. (m/s)
   */
  void addVelocity(const Eigen::Vector2d& velocity);

  /**
   * @brief Adds value to current angular velocity.
   *
   * @param angV Angular velocity to add to current angular velocity.
   * (rad/s, positive for counterclockwise).
   */
  void addAngV(double angV);

  /**
   * @brief Adds value to current position of centroid.
   *
   * @param pos Position to add to current centroid position. (in meters)
   */
  void addPos(const Eigen::Vector2d& pos);

  /**
   * @brief Gets convex state of polygon.
   *
   * @return Convex state of polygon.
   */
  bool isConvex() const;

  /**
   * @brief Gets collision state of polygon.
   *
   * @return Whether or not the polygon is in a collision.
   */
  bool isColliding() const;

  /**
   * @brief Gets collision state of polygon.
   *
   * @return Whether or not the polygon is in a collision.
   */
  void setColliding(bool colliding);

  /**
   * @brief Gets triangulation of polygon.
   *
   * @return Matrix containing the vertices of the triangles making up the
   * triangulation. Each column contains the indices of a triangle.
   */
  const Eigen::Matrix3Xi& getTriangulation() const;

  /**
   * @brief Gets convex decomposition of polygon.
   *
   * @return An array of arrays of indices containing the indices of the vertex
   * forming the convex polygons of the full polygon.
   */
  const std::vector<std::vector<int>>& getConvexDecomp() const;

  /**
   * @brief Gets ID of polygon.
   *
   * @return Polygon id.
   */
  int getId() const;

  /**
   * @brief Sets ID of polygon. (Usually set by the environment to differentiate
   * between polygons.)
   *
   * @param id Polygon id.
   */
  void setId(int id);

  /**
   * @brief Gets tag of polygon. Different from id in that it is never used by the simulation.
   *
   * @return Polygon tag.
   */
  int getTag() const;

  /**
   * @brief Sets tag of polygon.
   * @param tag Polygon tag.
   */
  void setTag(int tag);

  /**
   * @brief Adds a force influencing the polygon.
   *
   * @param source Type of force applied.
   * @param forcePos Where the force is applied relative to the centroid.
   * @param forceD Direction of the force.
   * @param amplitude Amplitude of the force.
   */
  void addForce(physics::forces::ForceSource source,
                const Eigen::Vector2d& forcePos, const Eigen::Vector2d& forceD,
                double amplitude);

  /**
   * @brief Removes a force influencing the polygon.
   *
   * @param source Type of force applied.
   */
  void removeForce(physics::forces::ForceSource source);

  /**
   * @brief Get force by source.
   *
   * @param source Type of force applied.
   *
   * @return The force matching the source.
   */
  physics::forces::Force& getForceBySource(physics::forces::ForceSource source);

  /**
   * @brief Get all sources of force applied to polygon.
   *
   * @return The force sources currently affecting the polygon.
   */
  std::unordered_map<physics::forces::ForceSource, physics::forces::Force>&
  getForceSources();

  /**
   * @brief Get moment of polygon around its centroid.
   *
   * @return The moment of the polygon.
   */
  double getMoment() const;

  /**
   * @brief Get furthest ditance from vertex to centroid.
   *
   * @return Furthest ditance from vertex to centroid.
   */
  double getFurthestVDist();

  /**
   * @brief Get density of polygon.
   *
   * @return Density of the polygon.
   */
  double getDensity() const;

  /**
   * @brief Gets mass of polygon.
   *
   * @return Mass of the polygon.
   */
  double getMass() const;

  /**
   * @brief Set density of polygon.
   *
   * @param density Density of the polygon.
   */
  void setDensity(double density);

  /**
   * @brief Sets mass of polygon.
   *
   * @param mass Mass of the polygon.
   */
  void setMass(double mass);
  /**
   * @brief Gets friction coefficient.
   *
   * @return Friction coefficient.
   */
  double getFrictionCoef() const;
  /**
   * @brief Gets restitution coefficient.
   *
   * @return Restitution coefficient.
   */
  double getResitutionCoef() const;

  /**
   * @brief Gets stationary status of polygon.
   *
   * @return Whether the polygon is unaffected by gravity.
   */
  double isStationary() const;

  /**
   * @brief Sets stationary status of polygon.
   *
   * @param stat Whether the polygon is unaffected by gravity.
   */
  void setStationary(bool stat);

  /**
   * @brief Set friction coefficient.
   *
   * @param frictionCoef Friction coefficient.
   */
  void setFrictionCoef(double frictionCoef);

  /**
   * @brief Set restitution coefficient.
   *
   * @param restitutionCoef Restitution coefficient.
   */
  void setRestitutionCoef(double frictionCoef);

private:
  int id;                   //< ID of polygon to help identify it.
  bool convex;              //< True if convex polygon, false otherwise.
  bool colliding;           //< True if the polygon is currently colliding.
  int nbVertices;           //< Number of vertices making up the polygon.
  double area;              //< Area of the polygon.
  double perimeter;         //< perimeter of the polygon.
  double rot;               //< Rotation angle.
  double angV;              //< Angular velocity.
  Eigen::Vector2d centroid; //< Centroid, i.e. center of mass of the polygon.
                            // counterclockwise order.
                            // | x1, x2, ...|
                            // | y1, y2, ...|
  Eigen::Vector2d velocity; //< Velocity of the centroid.
  const Eigen::Matrix2Xd
      localVertices; //< Array of vertices with respect to centroid.
  Eigen::Matrix3Xi
      triangulation; //< Triangulation of the polygon. Each column represents
                     // the counterclockwise vertex indices of a triangle.
  mutable Eigen::Matrix2Xd
      globalVertices; //< Array of vertices with respect to global coordinates
  mutable Eigen::Vector2d
      lastCentroid; //< Position of centroid after last call to
                    // getGlobalVertices.
  std::vector<std::vector<int>>
      convexification; //< Convex polygons making up the polygon. Stores index
                       // of vertices making it up.

  mutable double
      lastRot; //< Rotation of centroid after last call to getGlobalVertices.
  std::unordered_map<physics::forces::ForceSource, physics::forces::Force>
      forces;           // Map of forces acting on the plygon.
  double furthestVDist; //< Furthest distance between centroid and a vertex.
  double density;       //< Density of the polygon.
  double mass;          //< Mass of the polygon.
  double moment;        //< Moment of inertia of the polygon about its centroid.
  double restitutionCoef; //< Coefficient of resitution of the object.
  double frictionCoef;    //< Coefficient of friction of the object.
  bool stationary;        //< Whether the polygon is unaffected by gravity.
  int tag; //< Can get set by the user and is never used in the simulation.

  /**
   * @brief Finds the centroid of the polygon.
   *
   * @param Location of the vertices of the polygon.
   *
   * @return Location of the centroid.
   */
  Eigen::Vector2d findCentroid(const Eigen::Matrix2Xd& v);

  /**
   * @brief Finds the area of the polygon.
   *
   * @param vertices Position of global vertices.
   *
   * @return Area of the polygon.
   */
  double findArea(const Eigen::Matrix2Xd& vertices);

  /**
   * @brief Finds the perimeter of the polygon.
   *
   * @param vertices Position of global vertices.
   *
   * @return Perimeter of the polygon.
   */
  double findPerimeter(const Eigen::Matrix2Xd& vertices);

  /**
   * @brief Finds the local vertices from centroid and vertices.
   *
   * @param vertices Position of global vertices.
   *
   * @return Position of vertices realtive to centroid.
   */
  Eigen::Matrix2Xd findLocalVertices(const Eigen::Matrix2Xd& vertices);

  /**
   * @brief Finds the moment of inertia of the polygon about its centroid. Uses
   * triangulation to do so.
   *
   * @return Moment of inertia.
   */
  double findMoment();

  /**
   * @brief Finds the furthest distance between a vertex and the centroid.
   *
   * @return The furthest ditance.
   */
  double findFurthestVDist();
};
} // namespace env::bodies
