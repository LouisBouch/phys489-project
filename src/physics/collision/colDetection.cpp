#include "physics/collision/colDetection.hpp"
#include "env/bodies/Polygon.hpp"
#include "utils/geo/geoUtils.hpp"
#include <charconv>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

////////////////////////////////////////////////////////////
void physics::collision::findCollisions(env::Environment& env) {
  std::vector<env::bodies::Polygon>& polygons = env.getPolygons();
  int nbPoly = polygons.size();
  std::vector<int_fast8_t> collisions(nbPoly, 0);
  // Check for collisions between each pair of polygons.
  for (int i = 0; i < nbPoly - 1; i++) {
    for (int j = i + 1; j < nbPoly; j++) {
      // If no collision occurs, skip.
      if (!testCollision(polygons[i], polygons[j])) {
        continue;
      }
      collisions[i] = 1;
      collisions[j] = 1;
    }
  }
  // Updates collision status of polygons.
  for (int i = 0; i < nbPoly; i++) {
    if (collisions[i]) {
      polygons[i].setColliding(true);
      continue;
    }
    polygons[i].setColliding(false);
  }
}

////////////////////////////////////////////////////////////
bool physics::collision::testCollision(const env::bodies::Polygon& p1,
                                       const env::bodies::Polygon& p2) {
  const Eigen::Matrix2Xd& edges1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& edges2 = p2.getGlobalVertices();
  // Check for collisions between each pair of polygons.
  for (int a = 0, b = edges1.cols() - 1; a < edges1.cols(); b = a++) {
    for (int c = 0, d = edges2.cols() - 1; c < edges2.cols(); d = c++) {
      // std::cout << "a: " << a << ", b: " << b << ", c: " << c << ", d: " << d << "\n";
      if (utils::geo::testSegments(edges1.col(a), edges1.col(b), edges2.col(c),
                                   edges2.col(d))) {
        return true;
      }
    }
  }
  return false;
}
