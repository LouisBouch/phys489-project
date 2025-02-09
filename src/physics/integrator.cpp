#include "physics/integrator.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/forces/Force.hpp"
#include "utils/geo/geoUtils.hpp"
#include <cmath>
#include <complex>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
////////////////////////////////////////////////////////////
void physics::stepEnvironment(env::Environment& env, double dt) {
  for (env::bodies::Polygon& p : env.getPolygons()) {
    stepPolygon(p, dt);
  }
  env.unlockPolygons();
}

////////////////////////////////////////////////////////////
void physics::stepPolygon(env::bodies::Polygon& polygon, double dt) {
  polygon.addPos(polygon.getVelocity() * dt);
  polygon.rotate(polygon.getAngV() * dt);
}

////////////////////////////////////////////////////////////
void physics::applyForces(env::Environment& env, double dt) {
  for (env::bodies::Polygon& p : env.getPolygons()) {
    std::unordered_map<physics::forces::ForceSource, physics::forces::Force>&
        forcesMap = p.getForceSources();
    for (auto& force : forcesMap) {
      applyForceToPoly(p, force.second, dt);
    }
  }
  env.unlockPolygons();
}

////////////////////////////////////////////////////////////
void physics::applyForceToPoly(env::bodies::Polygon& polygon, forces::Force& f,
                               double dt) {
  // Apply translational force.
  Eigen::Vector2d acc = (f.getAmplitude() / polygon.getArea()) * f.getForceD();
  polygon.addVelocity(acc * dt);

  // Apply rotation force.
  Eigen::Vector2d r = f.getForcePos();
  double torque =  utils::geo::cross2D(r, f.getForceD()) * f.getAmplitude();
  torque/=5;
  polygon.addAngV(torque / polygon.getMoment()*dt);
}
