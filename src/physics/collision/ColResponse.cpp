#include "physics/collision/ColResponse.hpp"
#include "env/bodies/Polygon.hpp"
#include "utils/geo/geoUtils.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>

//////////////////////////////CONSTRUCTOR//////////////////////////////
physics::collision::ColResponse::ColResponse(int nbIterations, bool coupled)
    : nbIterations(nbIterations), coupled(coupled) {}

////////////////////////////////////////////////////////////
std::vector<double> physics::collision::ColResponse::findImpulseMagnitude(
    const Collision& collision) {
  // The magnitudes.
  std::vector<double> mag(collision.getManifold().size());
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();
  const Eigen::Vector2d& n = collision.getNormal();

  // Compute magnitude for each contact point.
  for (int c = 0; c < mag.size(); c++) {
    const env::bodies::Polygon& p1 = collision.getFirstPolygon();
    const env::bodies::Polygon& p2 = collision.getSecondPolygon();

    const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();
    // Indices of reference sub polygon.
    const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];
    Eigen::Vector2d maniPoint =
        collision.getManifold()[c]; // Contact point position.

    Eigen::Vector2d r2 =
        maniPoint - p2.getCentroid(); // Contact point on second polygon
                                      // relative to centroid.
    Eigen::Vector2d r1 =
        utils::geo::projectPoints(maniPoint - vs.col(subP[refEdge[0]]),
                                  vs.col(subP[refEdge[1]]) -
                                      vs.col(subP[refEdge[0]]))
            .col(0) +
        vs.col(subP[refEdge[0]]) -
        p1.getCentroid(); // Contact point on first polygon relative
                          // to centroid.
    // Relative velocity between contact points.
    Eigen::Vector2d vRel =
        (p2.getVelocity() - Eigen::Vector2d{-r2.y(), r2.x()} * p2.getAngV()) -
        (p1.getVelocity() - Eigen::Vector2d{-r1.y(), r1.x()} * p1.getAngV());

    double nominator = -vRel.dot(n); // Nominator of the magnitude equation.
    double r1CrossN = utils::geo::cross2D(r1, n);
    double r2CrossN = utils::geo::cross2D(r2, n);
    double denominator = 1 / p1.getArea() + 1 / p2.getArea() +
                         (r1CrossN*r2CrossN) / p1.getMoment() +
                         (r2CrossN*r2CrossN) / p2.getMoment();

    mag[c] = nominator / denominator;
  }
  std::cout << mag[0] << "\n"; 
  return mag;
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::resolveCollisions(
    const std::vector<Collision>& collisions) {
  std::vector<double> mag = findImpulseMagnitude(collisions[0]);
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::resolveCollision(
    const Collision& collision) {}
