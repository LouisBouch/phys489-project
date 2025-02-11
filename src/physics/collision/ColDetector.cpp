#include "physics/collision/ColDetector.hpp"
#include "env/Environment.hpp"
#include "physics/collision/Collision.hpp"
#include "utils/geo/geoUtils.hpp"
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

////////////////////////////////////////////////////////////
physics::collision::ColDetector::ColDetector(env::Environment* env)
    : env(env) {}
////////////////////////////////////////////////////////////
std::vector<physics::collision::Collision>&
physics::collision::ColDetector::findCollisions() {
  // Empty previous collisions.
  collisions.clear();
  std::vector<env::bodies::Polygon>& polygons = env->getPolygons();
  int nbPoly = polygons.size();
  std::vector<int_fast8_t> collisionStatus(nbPoly, 0);
  // Check for collisions between each pair of polygons.
  for (int i = 0; i < nbPoly - 1; i++) {
    for (int j = i + 1; j < nbPoly; j++) {
      // If no collision occurs, skip. (Either SAT collision detection or
      // edge-edge)
      if (SAT) {
        if (!testSATConcave(polygons[i], polygons[j])) {
          continue;
        }
      } else {
        if (!testEdgeCollisions(polygons[i], polygons[j])) {
          continue;
        }
      }
      collisionStatus[i] = 1;
      collisionStatus[j] = 1;
    }
  }
  // Updates collision status of polygons.
  for (int i = 0; i < nbPoly; i++) {
    if (collisionStatus[i]) {
      polygons[i].setColliding(true);
      continue;
    }
    polygons[i].setColliding(false);
  }
  env->unlockPolygons();
  return collisions;
}

////////////////////////////////////////////////////////////
bool physics::collision::ColDetector::testEdgeCollisions(
    env::bodies::Polygon& p1, env::bodies::Polygon& p2) {
  const Eigen::Matrix2Xd& edges1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& edges2 = p2.getGlobalVertices();
  // Check for collisions between each pair of polygons.
  for (int a = 0, b = edges1.cols() - 1; a < edges1.cols(); b = a++) {
    for (int c = 0, d = edges2.cols() - 1; c < edges2.cols(); d = c++) {
      if (utils::geo::testSegments(edges1.col(a), edges1.col(b), edges2.col(c),
                                   edges2.col(d))) {
        return true;
      }
    }
  }
  return false;
}
////////////////////////////////////////////////////////////
// TODO: Find fix for collisions happenning between multiple inner polygons.
bool physics::collision::ColDetector::testSATConcave(env::bodies::Polygon& p1,
                                                     env::bodies::Polygon& p2) {
  bool collided = false;
  // Let the polygon we compute the normal from be the reference polygon, and
  // let the other be called the incident polygon.
  //
  // For every triangle from the reference polygon, use only the edges that run
  // along the polygon (outside edges) as separating axes for SAT (the edes that
  // have consecutive vertices, i.e (3,4), (7,8), ...)

  // Get triangulation indices.
  const Eigen::Matrix3Xi& ts1 = p1.getTriangulation();
  const Eigen::Matrix3Xi& ts2 = p2.getTriangulation();

  // Get global vertices.
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();

  // Points of t1.
  Eigen::Matrix2Xd ps1(2, 3);
  // Points of t2.
  Eigen::Matrix2Xd ps2(2, 3);

  // For each triangle in the first polygon, check if it collides with a polygon
  // from the second polygon.
  for (int t1i = 0; t1i < ts1.cols(); t1i++) {
    const Eigen::Vector3i& t1 = ts1.col(t1i);
    ps1.col(0) = vs1.col(t1[0]);
    ps1.col(1) = vs1.col(t1[1]);
    ps1.col(2) = vs1.col(t1[2]);

    for (int t2i = 0; t2i < ts2.cols(); t2i++) {
      const Eigen::Vector3i& t2 = ts2.col(t2i);
      ps2.col(0) = vs2.col(t2[0]);
      ps2.col(1) = vs2.col(t2[1]);
      ps2.col(2) = vs2.col(t2[2]);

      // Check for collision between the triangles.
      if (subtestSATTria(p1, p2, t1, t2, ps1, ps2)) {
        collided = true;
      }
    }
  }
  return collided;
}
////////////////////////////////////////////////////////////
bool physics::collision::ColDetector::subtestSATTria(
    env::bodies::Polygon& p1, env::bodies::Polygon& p2,
    const Eigen::Vector3i& t1, const Eigen::Vector3i& t2,
    const Eigen::Matrix2Xd& ps1, const Eigen::Matrix2Xd& ps2) {

  std::array<int, 2> refEdge; // Reference edge of the collision.

  // Get vertices.
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();

  // Minimum penetration depth for the collision.
  // (Only accounts for valid edges)
  double minDepth = std::numeric_limits<double>::max();

  // Normal vector to separating axis.
  Eigen::Vector2d n;

  // Iterate over possible separating axes of first polygon.
  if (SATHelper(minDepth, ps1, ps2, n, refEdge, vs1, t1)) {
    return false;
  }
  double curMinDepth = minDepth;
  bool p1IsReference = true; // The reference polygon.
  // Iterate over possible separating axes of second polygon.
  if (SATHelper(minDepth, ps2, ps1, n, refEdge, vs2, t2)) {
    return false;
  }
  // If new minimum depth was found, the reference polygon is now p2.
  if (minDepth < curMinDepth) {
    p1IsReference = false;
  }

  // No separating axis found, add the collision.
  if (p1IsReference) {
    std::optional<Collision> col = physics::collision::Collision::create(
        p1, t1, p2, t2, n, minDepth, refEdge);
    if (col.has_value()) {
      collisions.push_back(col.value());
    }
  } else {
    std::optional<Collision> col = physics::collision::Collision::create(
        p2, t2, p1, t1, n, minDepth, refEdge);
    if (col.has_value()) {
      collisions.push_back(col.value());
    }
  }
  return true;
}

////////////////////////////////////////////////////////////
bool physics::collision::ColDetector::SATHelper(
    double& minDepthT, const Eigen::Matrix2Xd& ps1, const Eigen::Matrix2Xd& ps2,
    Eigen::Vector2d& n, std::array<int, 2>& refEdge, const Eigen::Matrix2Xd& vs,
    const Eigen::Vector3i& t) {
  // Iterate over first triangle edges ccw.
  for (int curV = 0, lastV = 2; curV < 3; lastV = curV++) {
    Eigen::Vector2d tempN =
        utils::geo::rotatePoints(ps1.col(curV) - ps1.col(lastV), -M_PI / 2.0);
    // Projected points of t1 onto the SA.
    Eigen::VectorXd pps1 = utils::geo::projectPointsMagnitude(ps1, tempN);

    // Projected points of t2 onto the SA.
    Eigen::VectorXd pps2 = utils::geo::projectPointsMagnitude(ps2, tempN);

    // Check for overlap between pps1 and pps2.
    double overlap = utils::geo::findOverlap(pps1, pps2);
    // Get maximum penetration depth along normal.
    double penDepth = -(
        utils::geo::projectPointsMagnitude(ps2.colwise() - ps1.col(curV), tempN)
            .minCoeff());
    if (overlap == 0) {
      // There is no overlap, thus we found a separating axis.
      return true;
    } else if (penDepth < minDepthT) {
      // Check if edge is valid, i.e. the edge is an outside edge.
      if (((t[lastV]) + 1) % vs.cols() == t[curV]) {
        minDepthT = penDepth;
        refEdge[0] = lastV;
        refEdge[1] = curV;
        n = tempN;
      }
    }
  }
  return false;
}

////////////////////////////////////////////////////////////
const std::vector<physics::collision::Collision>&
physics::collision::ColDetector::getCollisions() const {
  return collisions;
}

////////////////////////////////////////////////////////////
void physics::collision::ColDetector::setEnvironment(env::Environment* env) {
  this->env = env;
}
