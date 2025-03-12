#include "physics/collision/ColResponse.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/collision/ColDetector.hpp"
#include "physics/collision/Collision.hpp"
#include "utils/geo/geoUtils.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>

//////////////////////////////CONSTRUCTOR//////////////////////////////
physics::collision::ColResponse::ColResponse(int nbIterations, int nbPosIt,
                                             double minRelVel,
                                             double minRelFric)
    : nbVelIt(nbIterations), nbPosIt(nbPosIt), minRelVel(minRelVel),
      minRelFric(minRelFric),
      relVelR(std::exp(std::log(minRelVel) / (nbIterations - 1))),
      relFricR(std::exp(std::log(minRelFric) / (nbIterations - 1))) {}

////////////////////////////////////////////////////////////
double physics::collision::ColResponse::findImpulseMagnitude(
    Collision& collision, const Eigen::Vector2d& n, int c, double targetVel,
    double restitution) {
  // The impulse magnitudes.
  double impulse = 0;
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

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
  double numerator =
      targetVel - vRel.dot(n); // Nominator of the magnitude equation.
  double r1CrossN = utils::geo::cross2D(r1, n);
  double r2CrossN = utils::geo::cross2D(r2, n);
  double denominator = 1 / p1.getMass() + 1 / p2.getMass() +
                       (r1CrossN * r1CrossN) / p1.getMoment() +
                       (r2CrossN * r2CrossN) /
                           p2.getMoment(); // Also known as the effective mass.

  impulse = numerator / denominator;
  return impulse;
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::resolveCollisions(
    std::unordered_map<std::pair<int, int>, std::unique_ptr<Collision>,
                       PairHash>& collisions,
    double dt) {
  // Randomize order of collisions.
  // std::vector<Collision*> colsRandom;
  // colsRandom.reserve(collisions.size());
  // for (auto& colPair : collisions) {
  //   colsRandom.push_back(colPair.second.get());
  // }
  // std::mt19937 r(seed++);
  // std::shuffle(colsRandom.begin(), colsRandom.end(), r);
  // Start by obtaining target velocities based on the restitution coefficients.
  for (auto& colPair : collisions) {
    colPair.second.get()->findTargetVel();
  }
  // Apply warm started impulses.
  for (auto& colPair : collisions) {
    warmStart(*colPair.second.get());
  }
  // Apply velocity constraints and friction constraints.
  relaxFric = relaxVel = 1;
  for (int i = 0; i < nbVelIt; i++) {
    for (auto& colPair : collisions) {
      enforceVelConstraint(*colPair.second.get());
    }
    for (auto& colPair : collisions) {
      enforceFrictionConstraint(*colPair.second.get());
    }
    relaxVel *= relVelR;
    relaxFric *= relFricR;
  }
  // Apply position constraints.
  for (int i = 0; i < nbPosIt; i++) {
    for (auto& colPair : collisions) {
      auto& col = *colPair.second.get();
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
  const Eigen::Vector2d& n = collision.getNormal();

  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  env::bodies::Polygon& p1 = collision.getFirstPolygon();
  env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Apply and update impulse for each contact point.
  // std::vector<double> impulses = findImpulseMagnitudeCoupled(collision, n);
  // std::cout << "i1: " << impulses[0] << ", i2: " << impulses[1] << "\n";
  for (int c = 0; c < collision.getManifold().size(); c++) {
    double deltaImp =
        findImpulseMagnitude(collision, n, c, collision.getTargetVel()[c]);
    double oldImp = collision.getAccNormalImpulse()[c];
    collision.addNormalImpulse(deltaImp, c);
    // Get actual impulse change after clamping.
    deltaImp = collision.getAccNormalImpulse()[c] - oldImp;

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
    applyImpulse(deltaImp * relaxVel, n, p1, p2, r1, r2);
    // std::cout << "vel: " << deltaImp << "\n";
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
void physics::collision::ColResponse::enforceFrictionConstraint(
    Collision& collision) {

  const Eigen::Vector2d& n = collision.getNormal();
  const Eigen::Vector2d t =
      utils::geo::rotatePoints(n, M_PI / 2.0); // Tangent to normal.
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  env::bodies::Polygon& p1 = collision.getFirstPolygon();
  env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Apply and update impulse for each contact point.
  // std::vector<double> impulses = findImpulseMagnitudeCoupled(collision, t);
  for (int c = 0; c < collision.getManifold().size(); c++) {
    double deltaImp = findImpulseMagnitude(collision, t, c);
    double oldImp = collision.getAccTangentImpulse()[c];
    collision.addTangentImpulse(deltaImp, c);
    // Get actual impulse change after clamping.
    deltaImp = collision.getAccTangentImpulse()[c] - oldImp;

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
    double w = p2.getAngV();
    // std::cout << "id:\n " << p2.getId() << "\n";
    // std::cout
    // << "vel before: "
    // << (p2.getVelocity() + Eigen::Vector2d{-w * r2.y(), w * r2.x()}).dot(t)
    // << "\n";
    applyImpulse(deltaImp * relaxFric, t, p1, p2, r1, r2);
    // w = p2.getAngV();
    // std::cout
    // << "vel after: "
    // << (p2.getVelocity() + Eigen::Vector2d{-w * r2.y(), w * r2.x()}).dot(t)
    // << "\n";
  }
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::enforcePosConstraint(Collision& collision,
                                                           double dt,
                                                           double steerConst) {
  const Eigen::Vector2d& n = collision.getNormal();
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  env::bodies::Polygon& p1 = collision.getFirstPolygon();
  env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Apply and update impulse for each contact point.
  double allowedPixErr =
      0.5; // Depth in pixel that the polygons can penetrate the ground.
  for (int c = 0; c < collision.getManifold().size(); c++) {
    double deltaImp = findImpulseMagnitude(
        collision, n, c,
        steerConst * (collision.contactPenetration(c) - allowedPixErr) / dt);
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
    applyImpulseDirectly(deltaImp, n, p1, p2, r1, r2, dt);
  }
}
////////////////////////////////////////////////////////////
void physics::collision::ColResponse::applyImpulseDirectly(
    double impulse, const Eigen::Vector2d& n, env::bodies::Polygon& p1,
    env::bodies::Polygon& p2, const Eigen::Vector2d& r1,
    const Eigen::Vector2d& r2, double dt) {
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
////////////////////////////////////////////////////////////
std::vector<double>
physics::collision::ColResponse::findImpulseMagnitudeCoupled(
    Collision& collision, const Eigen::Vector2d& n,
    const Eigen::Vector2d& targetVel) {
  // Check if we only have 1 contact point.
  if (collision.getManifold().size() == 1) {
    return {findImpulseMagnitude(collision, n, 0)};
  }
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  const env::bodies::Polygon& p1 = collision.getFirstPolygon();
  const env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Velocity matrix
  Eigen::VectorXd V(12);
  V << p1.getVelocity(), 0, 0, 0, p1.getAngV(), p2.getVelocity(), 0, 0, 0,
      p2.getAngV();

  // Inverse masses matrix
  double m1 = 1 / p1.getMass();
  double m2 = 1 / p2.getMass();
  double I1 = 1 / p1.getMoment();
  double I2 = 1 / p2.getMoment();
  Eigen::DiagonalMatrix<double, 12> M;
  M.diagonal() << m1, m1, m1, I1, I1, I1, m2, m2, m2, I2, I2, I2;
  // Jacobians
  Eigen::VectorXd J1(12);
  Eigen::VectorXd J2(12);
  std::array<Eigen::VectorXd*, 2> Js = {&J1, &J2};

  // Populate jacobians
  for (int c = 0; c < collision.getManifold().size(); c++) {

    // Compute magnitude for contact point.
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
        p1.getCentroid(); // Contact point on first polygon
                          // relative to centroid.
    double wa = utils::geo::cross2D(r1, n);
    double wb = utils::geo::cross2D(r2, n);
    *Js[c] << -n.x(), -n.y(), 0, 0, 0, -wa, n.x(), n.y(), 0, 0, 0, wb;
  }
  // Solve system using Ax=b
  Eigen::Vector2d b =
      targetVel - Eigen::Vector2d{J1.transpose() * V, J2.transpose() * V};

  Eigen::Matrix2d A;
  A << J1.transpose() * M * J1, J1.transpose() * M * J2,
      J2.transpose() * M * J1, J2.transpose() * M * J2;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(A);
  std::cout << "condittion: " << A.norm() * A.inverse().norm() << "\n";
  std::cout << "norm: " << A.norm() << "\n";
  std::cout << "max eigen: " << eigensolver.eigenvalues().maxCoeff() << "\n";
  A.diagonal().array() += 1e-3;

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
  return {x.x(), x.y()};
}
////////////////////////////////////////////////////////////
void physics::collision::ColResponse::warmStart(Collision& collision) {
  const Eigen::Vector2d& n = collision.getNormal();
  const Eigen::Vector2d t =
      utils::geo::rotatePoints(n, M_PI / 2.0); // Tangent to normal.

  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  env::bodies::Polygon& p1 = collision.getFirstPolygon();
  env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Apply current impulse of collision.
  for (int c = 0; c < collision.getManifold().size(); c++) {
    double normalImpulse = collision.getAccNormalImpulse()[c];
    double tangentImpulse = collision.getAccTangentImpulse()[c];
    if (normalImpulse == 0) {
      continue;
    }

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
    // Apply impulse in normal direction.
    applyImpulse(normalImpulse, n, p1, p2, r1, r2);
    applyImpulse(tangentImpulse, t, p1, p2, r1, r2);
  }
}

////////////////////////////////////////////////////////////
void physics::collision::ColResponse::enforceVelFricConstraint(
    Collision& collision) {
  const Eigen::Vector2d& n = collision.getNormal();
  const Eigen::Vector2d t =
      utils::geo::rotatePoints(n, M_PI / 2.0); // Tangent to normal.

  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  env::bodies::Polygon& p1 = collision.getFirstPolygon();
  env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Apply and update impulse for each contact point.
  std::array<const Eigen::Vector2d*, 2> dirs = {&n, &t};
  for (int c = 0; c < collision.getManifold().size(); c++) {

    // Find relative distances.
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

    // Friction and velocity impulse at the contact point.
    std::vector<double> impulses =
        findImpulseMagnitudeCoupledSingle(collision, n, t, c);
    // Apply the impulse.
    for (int f = 0; f < 2; f++) {
      double deltaImp = impulses[f];
      // Get actual impulse change after clamping.
      if (f == 0) {
        double oldImp = collision.getAccNormalImpulse()[c];
        collision.addNormalImpulse(deltaImp, c);
        deltaImp = collision.getAccNormalImpulse()[c] - oldImp;
      } else {
        double oldImp = collision.getAccTangentImpulse()[c];
        collision.addTangentImpulse(deltaImp, c);
        deltaImp = collision.getAccTangentImpulse()[c] - oldImp;
      }
      applyImpulse(deltaImp, *dirs[f], p1, p2, r1, r2);
    }
  }
}
////////////////////////////////////////////////////////////
std::vector<double>
physics::collision::ColResponse::findImpulseMagnitudeCoupledSingle(
    Collision& collision, const Eigen::Vector2d& n1, const Eigen::Vector2d& n2,
    int c, const Eigen::Vector2d& targetVel) {
  const std::array<int, 2>& refEdge = collision.getRefEdgePI();

  const env::bodies::Polygon& p1 = collision.getFirstPolygon();
  const env::bodies::Polygon& p2 = collision.getSecondPolygon();
  const Eigen::Matrix2Xd& vs = p1.getGlobalVertices();

  // Indices of vertices of reference sub polygon.
  const std::vector<int>& subP = p1.getConvexDecomp()[collision.getSubP1I()];

  // Velocity matrix
  Eigen::VectorXd V(12);
  V << p1.getVelocity(), 0, 0, 0, p1.getAngV(), p2.getVelocity(), 0, 0, 0,
      p2.getAngV();

  // Inverse masses matrix
  double m1 = 1 / p1.getMass();
  double m2 = 1 / p2.getMass();
  double I1 = 1 / p1.getMoment();
  double I2 = 1 / p2.getMoment();
  Eigen::DiagonalMatrix<double, 12> M;
  M.diagonal() << m1, m1, m1, I1, I1, I1, m2, m2, m2, I2, I2, I2;
  // Jacobians
  Eigen::VectorXd J1(12);
  Eigen::VectorXd J2(12);

  std::array<Eigen::VectorXd*, 2> Js = {&J1, &J2};
  std::array<const Eigen::Vector2d*, 2> Ns = {&n1, &n2};

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

  // Populate jacobians
  for (int f = 0; f < 2; f++) {
    const Eigen::Vector2d& n = *Ns[f];
    double wa = utils::geo::cross2D(r1, n);
    double wb = utils::geo::cross2D(r2, n);
    *Js[f] << -n.x(), -n.y(), 0, 0, 0, -wa, n.x(), n.y(), 0, 0, 0, wb;
  }
  // Solve system using Ax=b
  Eigen::Vector2d b =
      targetVel - Eigen::Vector2d{J1.transpose() * V, J2.transpose() * V};

  Eigen::Matrix2d A;
  A << J1.transpose() * M * J1, J1.transpose() * M * J2,
      J2.transpose() * M * J1, J2.transpose() * M * J2;
  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(A);
  // std::cout << "condittion: " << A.norm() * A.inverse().norm() << "\n";
  // std::cout << "norm: " << A.norm() << "\n";
  // std::cout << "max eigen: " << eigensolver.eigenvalues().maxCoeff() << "\n";
  // A.diagonal().array() += 1e-3;

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
  std::cout << "A:\n" << A << "\n";
  std::cout << "b:\n" << b << "\n";
  std::cout << "x:\n" << x << "\n";
  return {x.x(), x.y()};
}
