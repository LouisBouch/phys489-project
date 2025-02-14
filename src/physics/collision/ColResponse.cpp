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
std::vector<double>
physics::collision::ColResponse::findImpulseMagnitude(Collision& collision) {
  // The impulse magnitudes.
  std::vector<double> impulses(collision.getManifold().size());
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();
  const Eigen::Vector2d& n = collision.getNormal();

  const env::bodies::Polygon& p1 = collision.getFirstPolygon();
  const env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Compute magnitude for each contact point.
  for (int c = 0; c < impulses.size(); c++) {

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
        (p2.getVelocity() + Eigen::Vector2d{-r2.y(), r2.x()} * p2.getAngV()) -
        (p1.getVelocity() + Eigen::Vector2d{-r1.y(), r1.x()} * p1.getAngV());

    double nominator = -vRel.dot(n); // Nominator of the magnitude equation.
    double r1CrossN = utils::geo::cross2D(r1, n);
    double r2CrossN = utils::geo::cross2D(r2, n);
    double denominator =
        1 / p1.getMass() + 1 / p2.getMass() +
        (r1CrossN * r1CrossN) / p1.getMoment() +
        (r2CrossN * r2CrossN) /
            p2.getMoment(); // Also known as the effective mass.

    impulses[c] = nominator / denominator;
  }
  return impulses;
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::resolveCollisions(
    std::vector<Collision>& collisions) {
  for (auto& col : collisions) {
    resolveCollision(col);
  }
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::resolveCollision(Collision& collision) {
  std::vector<double> deltaImps = findImpulseMagnitude(collision);

  const std::array<int, 2>& refEdge = collision.getRefEdgePI();
  const Eigen::Vector2d& n = collision.getNormal();

  env::bodies::Polygon& p1 = collision.getFirstPolygon();
  env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Apply and update impulse for each contact point.
  for (int c = 0; c < deltaImps.size(); c++) {
    double deltaImp = deltaImps[c];
    double oldImp = collision.getImpulse()[c];
    collision.addImpulse(deltaImp, c);
    // Get actual impulse change after clamping.
    deltaImp = collision.getImpulse()[c] - oldImp;

    // Apply the impulse.
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
    applyImpulse(deltaImp, n, p1, p2, r1, r2);
  }
}
////////////////////////////////////////////////////////////
void physics::collision::ColResponse::applyImpulse(double impulse,
                                                   const Eigen::Vector2d& n,
                                                   env::bodies::Polygon& p1,
                                                   env::bodies::Polygon& p2,
                                                   const Eigen::Vector2d& r1,
                                                   const Eigen::Vector2d& r2) {
  // Velocity
  p1.addVelocity(-impulse * n / p1.getMass());
  p2.addVelocity(impulse * n / p2.getMass());

  // Angualr velocity
  p1.addAngV(-impulse * utils::geo::cross2D(r1, n) / p1.getMoment());
  p2.addAngV(impulse * utils::geo::cross2D(r2, n) / p2.getMoment());
}
