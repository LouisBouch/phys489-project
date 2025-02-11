#include "physics/collision/ColDetector.hpp"
#include "env/Environment.hpp"
#include "physics/collision/Collision.hpp"
#include "utils/geo/geoUtils.hpp"
#include <array>
#include <cstdint>
#include <functional>
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

  // Get polygon decomposition indices.
  const std::vector<std::vector<int>>& polDecompI1 = p1.getConvexDecomp();
  const std::vector<std::vector<int>>& polDecompI2 = p2.getConvexDecomp();

  // Get global vertices.
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();

  // For each polygon in p1, check if it collides with a polygon from p2.
  for (int subP1I = 0; subP1I < polDecompI1.size(); subP1I++) {
    // Points of a polygon from the decomposition of p1.
    Eigen::Matrix2Xd ps1(2, polDecompI1[subP1I].size());

    const std::vector<int>& subP1 = polDecompI1[subP1I];
    for (int i = 0; i < subP1.size(); i++) {
      ps1.col(i) = vs1.col(subP1[i]);
    }

    for (int subP2I = 0; subP2I < polDecompI2.size(); subP2I++) {
      // Points of a polygon from the decomposition of p2.
      Eigen::Matrix2Xd ps2(2, polDecompI2[subP2I].size());

      const std::vector<int>& subP2 = polDecompI2[subP2I];
      for (int i = 0; i < subP2.size(); i++) {
        ps2.col(i) = vs2.col(subP2[i]);
      }

      // Check for collision between the sub polyguns.
      if (subtestSATTria(p1, p2, subP1, subP2, ps1, ps2)) {
        collided = true;
      }
    }
  }
  return collided;
}
////////////////////////////////////////////////////////////
bool physics::collision::ColDetector::subtestSATTria(
    env::bodies::Polygon& p1, env::bodies::Polygon& p2,
    const std::vector<int>& subP1, const std::vector<int>& subP2,
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
  if (SATHelper(minDepth, ps1, ps2, n, refEdge, vs1, subP1)) {
    return false;
  }
  double curMinDepth = minDepth;
  bool p1IsReference = true; // The reference polygon.
  // Iterate over possible separating axes of second polygon.
  if (SATHelper(minDepth, ps2, ps1, n, refEdge, vs2, subP2)) {
    return false;
  }
  // If new minimum depth was found, the reference polygon is now p2.
  if (minDepth < curMinDepth) {
    p1IsReference = false;
  }
  // No separating axis found, add the collision.
  if (p1IsReference) {
    std::optional<Collision> col = physics::collision::Collision::create(
        p1, subP1, p2, subP2, n, minDepth, refEdge);
    if (col.has_value()) {
      collisions.push_back(col.value());
    }
  } else {
    std::optional<Collision> col = physics::collision::Collision::create(
        p2, subP2, p1, subP1, n, minDepth, refEdge);
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
    const std::vector<int>& subP) {
  // Iterate over first sub polygon edges ccw.
  for (int curV = 0, lastV = subP.size() - 1; curV < subP.size();
       lastV = curV++) {
    Eigen::Vector2d tempN =
        utils::geo::rotatePoints(ps1.col(curV) - ps1.col(lastV), -M_PI / 2.0);
    // Projected points of first sub polygon onto the SA's normal.
    Eigen::VectorXd pps1 = utils::geo::projectPointsMagnitude(ps1, tempN);

    // Projected points of second sub polygon onto the SA's normal.
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
      if (((subP[lastV]) + 1) % vs.cols() == subP[curV]) {
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
