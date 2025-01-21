#include "env/bodies/Polygon.hpp"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>

////////////////////////////////////////////////////////////
env::bodies::Polygon::Polygon(const Eigen::Matrix2Xd& vertices)
    : nbVertices(vertices.cols()), area(findArea(vertices)),
      perimeter(findPerimeter(vertices)), centroid(findCentroid(vertices)),
      rot(0), localVertices(findLocalVertices(vertices)) {}

////////////////////////////////////////////////////////////
env::bodies::Polygon::Polygon(const Polygon& polygon)
    : nbVertices(polygon.nbVertices), area(polygon.area),
      perimeter(polygon.perimeter), centroid(polygon.centroid),
      rot(polygon.rot), localVertices(polygon.localVertices) {}

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
