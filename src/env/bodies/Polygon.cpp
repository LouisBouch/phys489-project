#include "env/bodies/Polygon.hpp"
#include "SFML/System/Vector2.hpp"
#include "physics/forces/Force.hpp"
#include "utils/geo/geoConvex.hpp"
#include "utils/geo/geoUtils.hpp"
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <stdexcept>
#include <string>

////////////////////////////////////////////////////////////
env::bodies::Polygon::Polygon(const Eigen::Matrix2Xd& vertices, double rot,
                              double angV, Eigen::Vector2d velocity, int id)
    : nbVertices(vertices.cols()), area(findArea(vertices)), colliding(false),
      perimeter(findPerimeter(vertices)), centroid(findCentroid(vertices)),
      rot(rot), angV(angV), velocity(velocity),
      localVertices(findLocalVertices(vertices)),
      convex(utils::geo::findConvexity(vertices)),
      triangulation(utils::geo::triangulate(vertices)),
      globalVertices(vertices), lastRot(INFINITY), lastCentroid(centroid),
      id(id), moment(findMoment()),
      convexification(utils::geo::convexify(vertices, triangulation)) {
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
              polygon.velocity, polygon.id) {
  translate(polygon.getCentroid());
  // Ensure forces are copied properly.
  for (auto& force : forces) {
    force.second.setRot(&rot);
  }
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
double env::bodies::Polygon::getRotation() const { return this->rot; }

////////////////////////////////////////////////////////////
const double& env::bodies::Polygon::getRotationR() const { return rot; }

////////////////////////////////////////////////////////////
int env::bodies::Polygon::getId() const { return id; }

////////////////////////////////////////////////////////////
void env::bodies::Polygon::setId(int id) { this->id = id; }

////////////////////////////////////////////////////////////
void env::bodies::Polygon::addForce(physics::forces::ForceSource source,
                                    const Eigen::Vector2d& forcePos,
                                    const Eigen::Vector2d& forceD,
                                    double amplitude) {
  forces.emplace(
      source, physics::forces::Force(forceD, amplitude, rot, forcePos, source));
}

////////////////////////////////////////////////////////////
void env::bodies::Polygon::removeForce(physics::forces::ForceSource source) {
  forces.erase(source);
}
////////////////////////////////////////////////////////////
physics::forces::Force&
env::bodies::Polygon::getForceBySource(physics::forces::ForceSource source) {
  return forces.at(source);
}

////////////////////////////////////////////////////////////
double env::bodies::Polygon::findMoment() {
  double moment = 0;
  // Add up moment of inerta of each triangle making up the polygon around the
  // centroid of the polygon. Not the triangulation triangles, but the same
  // triangles used to get the area.
  for (int curV = 0, lastV = nbVertices - 1; curV < nbVertices;
       lastV = curV++) {
    double areaTri =
        utils::geo::findParaArea(centroid, getGlobalVertices().col(lastV),
                                 getGlobalVertices().col(curV)) /
        2.0;
    // Given a vector centered at the origin, uses I=(a.a + b.b + a.b)*m/6 to
    // get moment of inertia about origin.
    Eigen::Vector2d a = getGlobalVertices().col(curV) - centroid;
    Eigen::Vector2d b = getGlobalVertices().col(lastV) - centroid;
    // Add triangle moment to total moment.
    moment += areaTri * (a.dot(a) + b.dot(b) + a.dot(b)) / 6.0;
  }
  return moment;
}

////////////////////////////////////////////////////////////
double env::bodies::Polygon::getMoment() { return moment; }

////////////////////////////////////////////////////////////
std::unordered_map<physics::forces::ForceSource, physics::forces::Force>&
env::bodies::Polygon::getForceSources() {
  return forces;
}
////////////////////////////////////////////////////////////
const std::vector<std::vector<int>>&
env::bodies::Polygon::getConvexDecomp() const {
  return convexification;
}
