#include "env/bodies/Polygon.hpp"
#include "utils/geo/geoUtils.hpp"
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <list>
#include <numeric>
#include <stdexcept>
#include <string>

////////////////////////////////////////////////////////////
env::bodies::Polygon::Polygon(const Eigen::Matrix2Xd& vertices, double rot,
                              double angV, Eigen::Vector2d velocity)
    : nbVertices(vertices.cols()), area(findArea(vertices)), colliding(false),
      perimeter(findPerimeter(vertices)), centroid(findCentroid(vertices)),
      rot(rot), angV(angV), velocity(velocity),
      localVertices(findLocalVertices(vertices)),
      convex(findConvexity(vertices)), triangulation(triangulate(vertices)),
      globalVertices(vertices), lastRot(INFINITY), lastCentroid(centroid) {
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
    : Polygon(polygon.getLocalVertices(), polygon.rot, polygon.angV,
              polygon.velocity) {
      translate(polygon.getCentroid());
    }

////////////////////////////////////////////////////////////
env::bodies::Polygon::~Polygon() {}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::translate(const Eigen::Vector2d& t) {
  centroid += t;
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
  if (rot == lastRot && centroid == lastCentroid) {
    return globalVertices;
  }
  // Update last values
  lastRot = rot;
  lastCentroid = centroid;
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
Eigen::Matrix2Xd env::bodies::Polygon::getSeparationAxes() const {
  // Make sure globalVertices are up to date
  getGlobalVertices();
  int nbCols = globalVertices.cols();
  Eigen::Matrix2Xd separationAxes(2, nbCols);
  for (int v = 0; v < nbCols; v++) {
    Eigen::Vector2d edge =
        globalVertices.col(v) - globalVertices.col((v + 1) % nbCols);
    // Rotated 90 degree counterclockwise, so x contains -y, and y contains x.
    separationAxes.col(v)[0] = -edge[1];
    separationAxes.col(v)[1] = edge[0];
  }
  return separationAxes;
}

////////////////////////////////////////////////////////////
const Eigen::Matrix2Xd& env::bodies::Polygon::getLocalVertices() const {
  return localVertices;
}
////////////////////////////////////////////////////////////
const Eigen::Matrix3Xi& env::bodies::Polygon::getTriangulation() const {
  return triangulation;
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
int env::bodies::Polygon::getNbVertices() const { return nbVertices; }

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
    double cross = v1[0] * v2[1] - v2[0] * v1[1];
    if (cross < 0) {
      std::cout << "concave" << "\n";
      return false;
    }
    if (std::abs(cross) - 1e-6 < 0) {
      std::cout << "colinear, illegal!" << "\n";
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addRot(double rot) { this->rot += rot; }

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addPos(const Eigen::Vector2d& pos) {
  centroid += pos;
}

////////////////////////////////////////////////////////////
bool env::bodies::Polygon::isColliding() const { return colliding; }

////////////////////////////////////////////////////////////
void env::bodies::Polygon::setColliding(bool colliding) {
  this->colliding = colliding;
}

////////////////////////////////////////////////////////////
Eigen::Matrix3Xi
env::bodies::Polygon::triangulate(const Eigen::Matrix2Xd& vertices) {
  // Number of triangles found.
  int trisFound = 0;
  // List of current triangles in the triangulation.
  Eigen::Matrix3Xi tris(3, nbVertices - 2);

  // List of indices for each vertex in the polygon.
  std::list<int> indices(nbVertices);
  // Fill list with ascending numbers, starting at 0.
  std::iota(indices.begin(), indices.end(), 0);

  // Indices of the three points of a triangle.
  int a;
  int b;
  int c;

  // Triangle to be tested.
  Eigen::Matrix2Xd ttemp(2, 3);

  // Find triangles using ear clipping.
  std::list<int>::iterator it = indices.begin();
  while (trisFound != nbVertices - 2) {
    // If at first element, take the last. Otherwise, take the previous one.
    a = it == indices.begin() ? *std::prev(indices.end()) : *std::prev(it);
    ttemp.col(0) = vertices.col(a);
    b = *it;
    ttemp.col(1) = vertices.col(b);
    // If at last element, take the first. Otherwise, take the next one.
    c = it == std::prev(indices.end()) ? *indices.begin() : *std::next(it);
    ttemp.col(2) = vertices.col(c);

    // Check if vertex is a valid triangle, i.e. the vertex is convex.
    if (utils::geo::findParaArea(vertices.col(a), vertices.col(b),
                                 vertices.col(c)) < 0) {
      // The vertex is concave, thus invalid for a polygon ear.
      it++;
      continue;
    }

    // Check if another point is contained in the triangle.
    bool triValid = true;
    for (auto tit = indices.begin(); tit != indices.end(); tit++) {
      // Don't check for points that are part of the triangle
      if (*tit == a || *tit == b || *tit == c) {
        continue;
      }
      if (utils::geo::pointInPolygon(ttemp, vertices.col(*tit))) {
        // There is another point in the triangle, so the vertex is invalid.
        it++;
        triValid = false;
        break;
      }
    }
    // If the triangle is valid, add it to the list.
    if (triValid) {
      tris.col(trisFound) << a, b, c;
      trisFound++;
      // Remove vertex.
      indices.erase(it);
      // Reset iterator.
      it = indices.begin();
    }
  }
  return tris;
}
////////////////////////////////////////////////////////////
double env::bodies::Polygon::getRotation() const {
  return this->rot;
}
