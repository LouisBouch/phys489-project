#include "physics/collision/Collision.hpp"
#include "env/bodies/Polygon.hpp"
#include "utils/geo/geoUtils.hpp"
#include <array>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <optional>
#include <vector>

//////////////////////////////FACTORY//////////////////////////////
std::optional<physics::collision::Collision>
physics::collision::Collision::create(const env::bodies::Polygon& p1,
                                      const Eigen::Vector3i& t1,
                                      const env::bodies::Polygon& p2,
                                      const Eigen::Vector3i& t2,
                                      const Eigen::Vector2d& n, double depth,
                                      std::array<int, 2> refEdgeTi) {
  std::optional<std::vector<Eigen::Vector2d>> mani =
      findManifold(p1, t1, p2, t2, n, depth, refEdgeTi);
  // Check if valid manifold exists.
  if (!mani.has_value()) {
    return std::nullopt;
  }

  return Collision(p1, t1, p2, t2, n, depth, refEdgeTi, mani.value());
}
//////////////////////////////CONSTRUCTORS//////////////////////////////
physics::collision::Collision::Collision(const Collision& col)
    : Collision(col.p1, col.t1, col.p2, col.t2, col.n, col.depth, col.refEdgeTi,
                col.manifold) {}

////////////////////////////////////////////////////////////
physics::collision::Collision::Collision(const env::bodies::Polygon& p1,
                                         const Eigen::Vector3i& t1,
                                         const env::bodies::Polygon& p2,
                                         const Eigen::Vector3i& t2,
                                         const Eigen::Vector2d& n, double depth,
                                         std::array<int, 2> refEdgeTi,
                                         std::vector<Eigen::Vector2d> manifold)
    : p1(p1), p2(p2), n(n.normalized()), depth(depth), manifold(manifold),
      refEdgeTi(refEdgeTi), t1(t1), t2(t2) {}
//////////////////////////////GETTERS//////////////////////////////
const env::bodies::Polygon&
physics::collision::Collision::getFirstPolygon() const {
  return p1;
}

////////////////////////////////////////////////////////////
const env::bodies::Polygon&
physics::collision::Collision::getSecondPolygon() const {
  return p2;
}

////////////////////////////////////////////////////////////
const Eigen::Vector2d& physics::collision::Collision::getNormal() const {
  return n;
}

////////////////////////////////////////////////////////////
double physics::collision::Collision::getDepth() const { return depth; }

////////////////////////////////////////////////////////////
const std::vector<Eigen::Vector2d>
physics::collision::Collision::getManifold() const {
  return manifold;
}

//////////////////////////////PRIVATE METHODS//////////////////////////////
std::optional<std::vector<Eigen::Vector2d>>
physics::collision::Collision::findManifold(
    const env::bodies::Polygon& p1, const Eigen::Vector3i& t1,
    const env::bodies::Polygon& p2, const Eigen::Vector3i& t2,
    const Eigen::Vector2d& n, double depth, std::array<int, 2> refEdgeTi) {
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();

  std::optional<std::vector<Eigen::Vector2d>> maniOpt =
      findIncEdge(p1, t1, p2, t2, n, depth, refEdgeTi);
  // Check if the incident edge is invalid.
  if (!maniOpt.has_value()) {
    return std::nullopt;
  }
  std::vector<Eigen::Vector2d> mani = maniOpt.value();
  std::cout << "inc edge: \n";
  std::cout << mani[0] << "\n";
  std::cout << mani[1] << "\n\n";

  std::cout << "ref edge: \n";
  std::cout << vs1.col(t1[refEdgeTi[0]]) << "\n";
  std::cout << vs1.col(t1[refEdgeTi[1]]) << "\n\n";

  // Now clip the manifold.
  for (int i = 0; i < 2; i++) {
    // Point on the half plane clipping line.
    Eigen::Vector2d halfEdgeP = vs1.col(t1[refEdgeTi[i]]);
    // Half plane normal.
    double sign = i == 0 ? -1 : 1;
    Eigen::Vector2d nHS =
        sign * (vs1.col(t1[refEdgeTi[1]]) - vs1.col(t1[refEdgeTi[0]]));
    // Clip each point in the manifold.
    for (int mi = 0; mi < 2; mi++) {
      if (!utils::geo::isAboveLine(mani[mi], nHS, halfEdgeP)) {
        // Don't clip if point is not outside the clipping line.
        continue;
      }
      // Clip with half plane.
      std::optional<Eigen::Vector2d> intersection =
          utils::geo::interSegLine(mani[0], mani[1], nHS, halfEdgeP);
      if (intersection.has_value()) {
        mani[mi] = intersection.value();
      }
    }
  }
  // For the final clip, remove points above the reference edge.
  Eigen::Vector2d nf = utils::geo::rotatePoints(
      vs1.col(t1[refEdgeTi[1]]) - vs1.col(t1[refEdgeTi[0]]), -M_PI / 2.0);
  // Remove points in the manifold above this line.
  for (int mi = 0; mi < 2; mi++) {
    if (utils::geo::isAboveLine(mani[mi], nf, vs1.col(t1[refEdgeTi[0]]))) {
      mani.erase(mani.begin() + mi);
    }
  }
  return mani;
}
////////////////////////////////////////////////////////////
std::optional<std::vector<Eigen::Vector2d>>
physics::collision::Collision::findIncEdge(
    const env::bodies::Polygon& p1, const Eigen::Vector3i& t1,
    const env::bodies::Polygon& p2, const Eigen::Vector3i& t2,
    const Eigen::Vector2d& n, double depth, std::array<int, 2> refEdgeTi) {
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  std::vector<Eigen::Vector2d> incEdge;
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();
  int furthestVTi = 0; // Which edge on the triangle is the furthest edge along
                       // the normal. (between 0 and 2)
  int maxDepth = 0;
  // Find furthest vertices along the collision normal of the incident polygon's
  // triangle.
  for (int i = 0; i < 3; i++) {
    // To ensure we get the distance from the edge to the vertex, we must
    // subtract from the vertex the values of a point on the edge.
    const Eigen::Vector2d& v = vs2.col(t2[i]) - vs1.col(t1[refEdgeTi[0]]);
    double vDepth = utils::geo::projectPointsMagnitude(v, n)[0];
    if (vDepth < maxDepth) {
      maxDepth = vDepth;
      furthestVTi = i;
    }
  }
  // From the furthest point, find the incident edge. (Edge attached to furthest
  // vertex on incident triangle that is most aligned with reference edge.)
  Eigen::Vector2d refEdge =
      vs1.col(t1[refEdgeTi[1]]) - vs1.col(t1[refEdgeTi[0]]); // Reference edge.
  int furthestVi = t2[furthestVTi];
  // First possible incident edge.
  int furthestNextVi = t2[(furthestVTi + 1) % 3];
  Eigen::Vector2d e1 = vs2.col(furthestNextVi) - vs2.col(furthestVi);
  // Second possible incident edge.
  int furthestPrevVi = t2[(furthestVTi + 2) % 3];
  Eigen::Vector2d e2 = vs2.col(furthestVi) - vs2.col(furthestPrevVi);

  // Find most aligned edge.
  int a = utils::geo::mostAligned(e1, e2, refEdge);
  // Initial values of contact manifold before clipping.
  incEdge.push_back(vs2.col(t2[furthestVTi]));
  if (a >= 0) {
    // Ensure valid edge
    if ((furthestVi + 1) % p2.getNbVertices() != furthestNextVi) {
      return std::nullopt;
    }
    incEdge.push_back(vs2.col(furthestNextVi));
  } else {
    // Ensure valid edge
    if ((furthestPrevVi + 1) % p2.getNbVertices() != furthestVi) {
      return std::nullopt;
    }
    incEdge.push_back(vs2.col(furthestPrevVi));
  }
  return incEdge;
}
