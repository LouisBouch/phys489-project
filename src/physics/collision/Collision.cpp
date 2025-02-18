#include "physics/collision/Collision.hpp"
#include "env/bodies/Polygon.hpp"
#include "utils/geo/geoUtils.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

//////////////////////////////FACTORY//////////////////////////////
std::optional<physics::collision::Collision>
physics::collision::Collision::create(env::bodies::Polygon& p1, int subP1I,
                                      env::bodies::Polygon& p2, int subP2I,
                                      const Eigen::Vector2d& n,
                                      std::array<int, 2> refEdgePI) {
  std::optional<std::vector<Eigen::Vector2d>> mani =
      findManifold(p1, subP1I, p2, subP2I, n, refEdgePI);
  // Check if valid manifold exists.
  if (!mani.has_value()) {
    return std::nullopt;
  }

  return Collision(p1, subP1I, p2, subP2I, n, refEdgePI, mani.value());
}
//////////////////////////////CONSTRUCTORS//////////////////////////////
physics::collision::Collision::Collision(const Collision& col)
    : Collision(col.p1, col.subP1I, col.p2, col.subP2I, col.n, col.refEdgePI,
                col.manifold) {
  accNormalImpulse = col.accNormalImpulse;
  accPseudoImpulse = col.accPseudoImpulse;
  accTangentImpulse = col.accTangentImpulse;
}

////////////////////////////////////////////////////////////
physics::collision::Collision::Collision(env::bodies::Polygon& p1, int subP1I,
                                         env::bodies::Polygon& p2, int subP2I,
                                         const Eigen::Vector2d& n,
                                         std::array<int, 2> refEdgePI,
                                         std::vector<Eigen::Vector2d> manifold)
    : p1(p1), p2(p2), n(n.normalized()), manifold(manifold),
      refEdgePI(refEdgePI), subP1I(subP1I), subP2I(subP2I),
      accNormalImpulse(manifold.size()), accPseudoImpulse(manifold.size()),
      accTangentImpulse(manifold.size()) {}
//////////////////////////////GETTERS//////////////////////////////
env::bodies::Polygon& physics::collision::Collision::getFirstPolygon() {
  return p1;
}

////////////////////////////////////////////////////////////
env::bodies::Polygon& physics::collision::Collision::getSecondPolygon() {
  return p2;
}

////////////////////////////////////////////////////////////
const Eigen::Vector2d& physics::collision::Collision::getNormal() const {
  return n;
}

////////////////////////////////////////////////////////////
const std::vector<Eigen::Vector2d>
physics::collision::Collision::getManifold() const {
  return manifold;
}

//////////////////////////////PRIVATE METHODS//////////////////////////////
std::optional<std::vector<Eigen::Vector2d>>
physics::collision::Collision::findManifold(
    env::bodies::Polygon& p1, int subP1I, env::bodies::Polygon& p2, int subP2I,
    const Eigen::Vector2d& n, std::array<int, 2> refEdgePI) {
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();

  const std::vector<int>& subP1 = p1.getConvexDecomp()[subP1I];
  const std::vector<int>& subP2 = p2.getConvexDecomp()[subP2I];

  std::optional<std::vector<Eigen::Vector2d>> maniOpt =
      findIncEdge(p1, subP1I, p2, subP2I, n, refEdgePI);
  // Check if the incident edge is invalid.
  if (!maniOpt.has_value()) {
    return std::nullopt;
  }
  std::vector<Eigen::Vector2d> mani = maniOpt.value();
  // std::cout << "inc edge: \n";
  // std::cout << mani[0] << "\n";
  // std::cout << mani[1] << "\n\n";
  //
  // std::cout << "ref edge: \n";
  // std::cout << vs1.col(subP1[refEdgePI[0]]) << "\n";
  // std::cout << vs1.col(subP1[refEdgePI[1]]) << "\n\n";

  // Now clip the manifold.
  for (int i = 0; i < 2; i++) {
    // Point on the half plane clipping line.
    Eigen::Vector2d halfEdgeP = vs1.col(subP1[refEdgePI[i]]);
    // Half plane normal.
    double sign = i == 0 ? -1 : 1;
    Eigen::Vector2d nHS =
        sign * (vs1.col(subP1[refEdgePI[1]]) - vs1.col(subP1[refEdgePI[0]]));
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
      vs1.col(subP1[refEdgePI[1]]) - vs1.col(subP1[refEdgePI[0]]), -M_PI / 2.0);
  // Remove points in the manifold above this line.
  for (int mi = 0; mi < 2; mi++) {
    if (utils::geo::isAboveLine(mani[mi], nf, vs1.col(subP1[refEdgePI[0]]))) {
      mani.erase(mani.begin() + mi);
    }
  }
  return mani;
}
////////////////////////////////////////////////////////////
std::optional<std::vector<Eigen::Vector2d>>
physics::collision::Collision::findIncEdge(const env::bodies::Polygon& p1,
                                           int subP1I,
                                           const env::bodies::Polygon& p2,
                                           int subP2I, const Eigen::Vector2d& n,

                                           std::array<int, 2> refEdgePI) {
  const Eigen::Matrix2Xd& vs1 = p1.getGlobalVertices();
  std::vector<Eigen::Vector2d> incEdge;
  const Eigen::Matrix2Xd& vs2 = p2.getGlobalVertices();

  const std::vector<int>& subP1 = p1.getConvexDecomp()[subP1I];
  const std::vector<int>& subP2 = p2.getConvexDecomp()[subP2I];

  int furthestVPi = 0; // Which edge on the sub polygon is the furthest edge
                       // along the normal.
  int maxDepth = 0;
  // Find furthest vertices along the collision normal of the incident polygon's
  // sub polygon.
  for (int i = 0; i < subP2.size(); i++) {
    // To ensure we get the distance from the edge to the vertex, we must
    // subtract from the vertex the values of a point on the edge.
    const Eigen::Vector2d& v = vs2.col(subP2[i]) - vs1.col(subP1[refEdgePI[0]]);
    double vDepth = utils::geo::projectPointsMagnitude(v, n)[0];
    if (vDepth < maxDepth) {
      maxDepth = vDepth;
      furthestVPi = i;
    }
  }
  // From the furthest point, find the incident edge. (Edge attached to furthest
  // vertex on incident sub polygon that is most aligned with reference edge.)
  Eigen::Vector2d refEdge = vs1.col(subP1[refEdgePI[1]]) -
                            vs1.col(subP1[refEdgePI[0]]); // Reference edge.
  int furthestVi = subP2[furthestVPi];
  // First possible incident edge.
  int furthestNextVi = subP2[(furthestVPi + 1) % subP2.size()];
  Eigen::Vector2d e1 = vs2.col(furthestNextVi) - vs2.col(furthestVi);
  // Second possible incident edge.
  int furthestPrevVi = subP2[(furthestVPi + subP2.size() - 1) % subP2.size()];
  Eigen::Vector2d e2 = vs2.col(furthestVi) - vs2.col(furthestPrevVi);

  // Find most aligned edge.
  int a = utils::geo::mostAligned(e1, e2, refEdge);
  // Initial values of contact manifold before clipping.
  incEdge.push_back(vs2.col(subP2[furthestVPi]));
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
////////////////////////////////////////////////////////////
const int physics::collision::Collision::getSubP1I() const { return subP1I; }

////////////////////////////////////////////////////////////
const std::array<int, 2>& physics::collision::Collision::getRefEdgePI() const {
  return refEdgePI;
}
////////////////////////////////////////////////////////////
const std::vector<double>&
physics::collision::Collision::getAccNormalImpulse() const {
  return accNormalImpulse;
}
////////////////////////////////////////////////////////////
const std::vector<double>&
physics::collision::Collision::getAccTangentImpulse() const {
  return accTangentImpulse;
}
////////////////////////////////////////////////////////////
const std::vector<double>&
physics::collision::Collision::getAccPseudoImpulse() const {
  return accPseudoImpulse;
}

////////////////////////////////////////////////////////////
void physics::collision::Collision::addNormalImpulse(double impulse,
                                                     int contactPoint) {
  if (contactPoint < this->accNormalImpulse.size()) {
    this->accNormalImpulse[contactPoint] =
        std::max<double>(0, this->accNormalImpulse[contactPoint] + impulse);
    // this->accNormalImpulse[contactPoint] =
    //     this->accNormalImpulse[contactPoint] + impulse;
  }
}

////////////////////////////////////////////////////////////
void physics::collision::Collision::setNormalImpulse(
    const std::vector<double>& impulse) {
  if (impulse.size() == this->accNormalImpulse.size()) {
    this->accNormalImpulse = impulse;
  }
}
////////////////////////////////////////////////////////////
void physics::collision::Collision::addTangentImpulse(double impulse,
                                                      int contactPoint) {
  double fricCoeff = std::sqrt(p1.getFrictionCoef() * p2.getFrictionCoef());
  double normalImpulse = std::abs(getAccNormalImpulse()[contactPoint]);
  if (contactPoint < this->accTangentImpulse.size()) {
    this->accTangentImpulse[contactPoint] =
        std::clamp(this->accTangentImpulse[contactPoint] + impulse,
                   -fricCoeff * normalImpulse, fricCoeff * normalImpulse);
  }
}
////////////////////////////////////////////////////////////
void physics::collision::Collision::setTangentImpulse(
    const std::vector<double>& impulse) {
  if (impulse.size() == this->accTangentImpulse.size()) {
    this->accTangentImpulse = impulse;
  }
}
////////////////////////////////////////////////////////////
void physics::collision::Collision::addPseudoImpulse(double impulse,
                                                     int contactPoint) {
  if (contactPoint < this->accPseudoImpulse.size()) {
    this->accPseudoImpulse[contactPoint] =
        std::max<double>(0, this->accPseudoImpulse[contactPoint] + impulse);
  }
}
////////////////////////////////////////////////////////////
double physics::collision::Collision::contactPenetration(int contactPoint) {
  if (contactPoint < this->accNormalImpulse.size()) {
    const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();
    const std::vector<int>& subP = p1.getConvexDecomp()[subP1I];
    return -utils::geo::projectPointsMagnitude(
        manifold[contactPoint] - vs.col(subP[refEdgePI[0]]), n)[0];
  }
  return 0;
}
////////////////////////////////////////////////////////////
bool physics::collision::Collision::updateCollision() {
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();
  const std::vector<int>& sp = p1.getConvexDecomp()[subP1I];
  Eigen::Vector2d refEdgeV =
      vs.col(sp[refEdgePI[1]]) - vs.col(sp[refEdgePI[0]]);
  n = Eigen::Vector2d{refEdgeV.y(), -refEdgeV.x()}.normalized();
  auto opt = findManifold(p1, subP1I, p2, subP2I, n, refEdgePI);
  if (opt.has_value() && opt.value().size() > 0) {
    // Update impulse vectors if necessary.
    if (manifold.size() != opt.value().size()) {
      accNormalImpulse = std::vector<double>(opt.value().size());
      accPseudoImpulse = std::vector<double>(accNormalImpulse.size());
      accTangentImpulse = std::vector<double>(accNormalImpulse.size());
    }
    manifold = opt.value();
    return true;
  }
  return false;
}
////////////////////////////////////////////////////////////
double physics::collision::Collision::findDifference(
    physics::collision::Collision& col) {
  // Ensure collisions match.
  if ((col.getFirstPolygon().getId() != p1.getId() &&
       col.getFirstPolygon().getId() != p2.getId()) ||
      (col.getSecondPolygon().getId() != p1.getId() &&
       col.getSecondPolygon().getId() != p2.getId()) ||
      col.getManifold().size() != manifold.size()) {
    return std::numeric_limits<double>::max();
  }
  // If they do, return the minimum average squared distance between the
  // collision points.
  const std::vector<Eigen::Vector2d>& maniC = col.getManifold();
  if (maniC.size() == 1) {
    return (maniC[0] - manifold[0]).squaredNorm();
  }
  return std::min<double>((maniC[0] - manifold[0]).squaredNorm() +
                              (maniC[1] - manifold[1]).squaredNorm(),
                          (maniC[0] - manifold[1]).squaredNorm() +
                              (maniC[1] - manifold[0]).squaredNorm()) /
         2.0;
}
