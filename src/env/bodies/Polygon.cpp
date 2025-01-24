#include "env/bodies/Polygon.hpp"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <stdexcept>
#include <string>

////////////////////////////////////////////////////////////
env::bodies::Polygon::Polygon(const Eigen::Matrix2Xd& vertices, double rot,
                              double angV, Eigen::Vector2d velocity)
    : nbVertices(vertices.cols()), area(findArea(vertices)),
      perimeter(findPerimeter(vertices)), centroid(findCentroid(vertices)),
      rot(rot), angV(angV), velocity(velocity),
      localVertices(findLocalVertices(vertices)),
      convex(findConvexity(vertices)) {
  // Requires at least 3 vertices for valid polygon.
  if (nbVertices < 3) {
    throw std::invalid_argument(
        "Polygon must have a least 3 vertices, but got" +
        std::to_string(nbVertices));
  }
  // TODO: Check for self-intersection and allow for clockwise polygons
}

////////////////////////////////////////////////////////////
env::bodies::Polygon::Polygon(const Polygon& polygon)
    : nbVertices(polygon.nbVertices), area(polygon.area),
      perimeter(polygon.perimeter), centroid(polygon.centroid),
      rot(polygon.rot), angV(polygon.angV), velocity(polygon.velocity),
      localVertices(polygon.localVertices), convex(polygon.convex) {}

////////////////////////////////////////////////////////////
env::bodies::Polygon::~Polygon() {}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::translate(const Eigen::Vector2d& t) {
  centroid.noalias() += t;
}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::rotate(double r) {
  rot = std::fmod((rot + r), (2 * M_PI));
}

////////////////////////////////////////////////////////////
double env::bodies::Polygon::getArea() { return area; }

////////////////////////////////////////////////////////////
double env::bodies::Polygon::getPerimeter() const { return perimeter; }

////////////////////////////////////////////////////////////
const Eigen::Vector2d& env::bodies::Polygon::getCentroid() const {
  return centroid;
}

////////////////////////////////////////////////////////////
const Eigen::Matrix2Xd& env::bodies::Polygon::getGlobalVertices() const {
  // Create rotation matrix
  double cosr = std::cos(rot);
  double sinr = std::sin(rot);
  Eigen::Matrix2d rM{{cosr, -sinr}, {sinr, cosr}};

  // Rotate vertices
  globalVertices.noalias() = rM * localVertices;

  // Translate vertices
  globalVertices.colwise() += centroid;

  return globalVertices;
}

////////////////////////////////////////////////////////////
const Eigen::Matrix2Xd& env::bodies::Polygon::getLocalVertices() const {
  return localVertices;
}

////////////////////////////////////////////////////////////
Eigen::Vector2d
env::bodies::Polygon::findCentroid(const Eigen::Matrix2Xd& vertices) {
  Eigen::Vector2d c;
  int nbCols = vertices.cols();
  for (int v = 0; v < nbCols; v++) {
    double subArea = vertices.col(v)[0] * vertices.col((v + 1) % nbCols)[1] -
                     vertices.col((v + 1) % nbCols)[0] * vertices.col(v)[1];
    // C_x
    c[0] += subArea * (vertices.col(v)[0] + vertices.col((v + 1) % nbCols)[0]);

    // C_y
    c[1] += subArea * (vertices.col(v)[1] + vertices.col((v + 1) % nbCols)[1]);
  }
  return c / (6.0 * area);
}

////////////////////////////////////////////////////////////
double env::bodies::Polygon::findArea(const Eigen::Matrix2Xd& vertices) {
  double a = 0;
  int nbCols = vertices.cols();
  for (int v = 0; v < nbCols; v++) {
    a += vertices.col(v)[0] * vertices.col((v + 1) % nbCols)[1] -
         vertices.col((v + 1) % nbCols)[0] * vertices.col(v)[1];
  }
  return a / 2.0;
}

////////////////////////////////////////////////////////////
double env::bodies::Polygon::findPerimeter(const Eigen::Matrix2Xd& vertices) {
  double peri = 0;
  int nbCols = vertices.cols();
  for (int v = 0; v < nbCols; v++) {
    peri += (vertices.col((v + 1) % nbCols) - vertices.col(v)).norm();
  }
  return peri;
}
////////////////////////////////////////////////////////////
Eigen::Matrix2Xd
env::bodies::Polygon::findLocalVertices(const Eigen::Matrix2Xd& vertices) {
  int nbCols = vertices.cols();
  Eigen::Matrix2Xd localV(2, nbCols);

  for (int v = 0; v < nbCols; v++) {
    localV.col(v) = vertices.col(v) - centroid;
  }
  return localV;
}
////////////////////////////////////////////////////////////
double env::bodies::Polygon::getNbVertices() const { return nbVertices; }

////////////////////////////////////////////////////////////
const Eigen::Vector2d& env::bodies::Polygon::getVelocity() const {
  return velocity;
}

////////////////////////////////////////////////////////////
double env::bodies::Polygon::getAngV() const { return angV; }

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addVelocity(const Eigen::Vector2d& velocity) {
  this->velocity[0] += velocity[0];
  this->velocity[1] += velocity[1];
}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addAngV(double angV) { this->angV += angV; }

////////////////////////////////////////////////////////////
bool env::bodies::Polygon::isConvex() const { return convex; }

////////////////////////////////////////////////////////////
bool env::bodies::Polygon::findConvexity(const Eigen::Matrix2Xd& vertices) {
  double a = 0;
  int nbCols = vertices.cols();
  for (int v = 0; v < nbCols; v++) {
    Eigen::Vector2d v1 =
        vertices.col((v + 2) % nbCols) - vertices.col((v + 1) % nbCols);
    Eigen::Vector2d v2 = vertices.col(v) - vertices.col((v + 1) % nbCols);
    // Check for angle > 180 deg.
    if (v1[0] * v2[1] - v2[0] * v1[1] < 0) {
      std::cout << "concave" << "\n";
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addRot(double rot) { this->rot += rot; }

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addPos(const Eigen::Vector2d& pos) {
  centroid.noalias() += pos;
}
