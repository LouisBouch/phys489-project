#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace env::bodies {
class Polygon {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param vertices Vertices of the polygon in COUNTERCLOCKWISE order.
   */
  Polygon(const Eigen::Matrix2Xd& vertices, double rot = 0, double angV = 0,
          const Eigen::Vector2d velocity = Eigen::Vector2d{0, 0});

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
   * @brief Gets the area of the polygon.
   *
   * @return Area of the polygon.
   */
  double getArea();

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
  double getNbVertices() const;

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
   * @brief Adds value to current rotation.
   *
   * @param rot Rotation to add to current rotation. (in rads)
   */
  void addRot(double rot);

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

private:
  bool convex;              //< True if convex polygon, false otherwise.
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
  Eigen::Matrix2Xd
      localVertices; //< Array of vertices with respect to centroid in
  mutable Eigen::Matrix2Xd
      globalVertices; //< Array of vertices with respect to global coordinates

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
   * @brief Determines whether polygon is convex or not.
   *
   * @return Convexity of polygon. (True if convex)
   */
  bool findConvexity(const Eigen::Matrix2Xd& vertices);
};
} // namespace env::bodies
