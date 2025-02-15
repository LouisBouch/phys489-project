#include "physics/collision/ColResponse.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/collision/Collision.hpp"
#include "utils/geo/geoUtils.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>

//////////////////////////////CONSTRUCTOR//////////////////////////////
physics::collision::ColResponse::ColResponse(int nbIterations, int nbPosIt, bool coupled)
    : nbVelIt(nbIterations), nbPosIt(nbPosIt), coupled(coupled) {}

////////////////////////////////////////////////////////////
double
physics::collision::ColResponse::findImpulseMagnitude(Collision& collision,
                                                      int c, double targetVel) {
  // The impulse magnitudes.
  double impulse = 0;
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();
  const Eigen::Vector2d& n = collision.getNormal();

  const env::bodies::Polygon& p1 = collision.getFirstPolygon();
  const env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Compute magnitude for contact point.
  Eigen::Vector2d maniPoint =
      collision.getManifold()[c]; // Contact point position.

  Eigen::Vector2d r2 =
      maniPoint - p2.getCentroid(); // Contact point on second polygon
                                    // relative to centroid.
  Eigen::Vector2d r1 = utils::geo::projectPoints(
                           maniPoint - vs.col(subP[refEdge[0]]),
                           vs.col(subP[refEdge[1]]) - vs.col(subP[refEdge[0]]))
                           .col(0) +
                       vs.col(subP[refEdge[0]]) -
                       p1.getCentroid(); // Contact point on first polygon
                                         // relative to centroid.
  // Relative velocity between contact points.
  Eigen::Vector2d vRel =
      (p2.getVelocity() + Eigen::Vector2d{-r2.y(), r2.x()} * p2.getAngV()) -
      (p1.getVelocity() + Eigen::Vector2d{-r1.y(), r1.x()} * p1.getAngV());

  double nominator =
      targetVel - vRel.dot(n); // Nominator of the magnitude equation.
  double r1CrossN = utils::geo::cross2D(r1, n);
  double r2CrossN = utils::geo::cross2D(r2, n);
  double denominator = 1 / p1.getMass() + 1 / p2.getMass() +
                       (r1CrossN * r1CrossN) / p1.getMoment() +
                       (r2CrossN * r2CrossN) /
                           p2.getMoment(); // Also known as the effective mass.

  impulse = nominator / denominator;
  return impulse;
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::resolveCollisions(
    std::vector<Collision>& collisions, double dt) {
  for (int i = 0; i < nbVelIt; i++) {
    for (auto& col : collisions) {
      enforceVelConstraint(col);
    }
  }
  for (int i = 0; i < nbPosIt; i++) {
    for (auto& col : collisions) {
      // Update collision info in case previous collision resolution moved
      // polygons around. (Only if collision still exists)
      if (!col.updateCollision()) {
        continue;
      }

      enforcePosConstraint(col, dt, 0.5);
    }
  }
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::enforceVelConstraint(
    Collision& collision) {
  std::vector<double> deltaImps(collision.getManifold().size());
  for (int c = 0; c < deltaImps.size(); c++) {
    deltaImps[c] = findImpulseMagnitude(collision, c);
  }

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
    double oldImp = collision.getAccImpulse()[c];
    collision.addImpulse(deltaImp, c);
    // Get actual impulse change after clamping.
    deltaImp = collision.getAccImpulse()[c] - oldImp;

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
////////////////////////////////////////////////////////////
void physics::collision::ColResponse::enforcePosConstraint(Collision& collision,
                                                           double dt,
                                                           double steerConst) {
  std::vector<double> deltaImps(collision.getManifold().size());
  for (int c = 0; c < deltaImps.size(); c++) {
    deltaImps[c] = findImpulseMagnitude(
        collision, c, steerConst * collision.contactPenetration(c) / dt);
  }
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
    double oldImp = collision.getAccPseudoImpulse()[c];
    collision.addPseudoImpulse(deltaImp, c);
    // Get actual impulse change after clamping.
    deltaImp = collision.getAccPseudoImpulse()[c] - oldImp;

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
    applyImpulseDirectly(deltaImp, n, p1, p2, r1, r2, dt, collision);
  }
}
////////////////////////////////////////////////////////////
void physics::collision::ColResponse::applyImpulseDirectly(
    double impulse, const Eigen::Vector2d& n, env::bodies::Polygon& p1,
    env::bodies::Polygon& p2, const Eigen::Vector2d& r1,
    const Eigen::Vector2d& r2, double dt, Collision& collision) {
  // Velocity
  Eigen::Vector2d p1T = -impulse * n / p1.getMass() * dt;
  Eigen::Vector2d p2T = impulse * n / p2.getMass() * dt;
  p1.addPos(p1T);
  p2.addPos(p2T);

  // Angualr velocity
  double p1R = -impulse * utils::geo::cross2D(r1, n) / p1.getMoment() * dt;
  double p2R = impulse * utils::geo::cross2D(r2, n) / p2.getMoment() * dt;
  p1.rotate(p1R);
  p2.rotate(p2R);
}
