#include "physics/integrator.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/forces/Force.hpp"
#include "utils/geo/geoUtils.hpp"
#include <cmath>
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
    bool dragged =
        false; // Whether the shape is being dragged or not by the user.
    std::unordered_map<physics::forces::ForceSource, physics::forces::Force>&
        forcesMap = p.getForceSources();
    double r = 0.999; // Ratio of rotation lost every second.
    for (auto& force : forcesMap) {
      // Don't apply gravity to statonary polygon.
      if ((force.first == forces::ForceSource::Gravity) && p.isStationary()) {
        continue;
      }
      if (force.first == forces::ForceSource::NoRot) {
        r = 1; // Removes all spin.
        continue;
      }
      if (force.first == forces::ForceSource::UserInducedDrag) {
        dragged = true;
      }
      applyForceToPoly(p, force.second, dt);
    }
    // Dampen rotation by some percentage per second if user is dragging the
    // polygon was applied.
    if (forcesMap.size() > 0 && dragged) {
      p.addAngV(p.getAngV() * (std::pow(1 - r, dt) - 1));
    }
  }
  env.unlockPolygons();
}

////////////////////////////////////////////////////////////
void physics::applyForceToPoly(env::bodies::Polygon& polygon, forces::Force& f,
                               double dt) {
  // Apply translational force.
  Eigen::Vector2d acc = (f.getAmplitude() / polygon.getMass()) * f.getForceD();
  polygon.addVelocity(acc * dt);

  // Apply rotation force.
  Eigen::Vector2d r = f.getForcePos();
  double torque = utils::geo::cross2D(r, f.getForceD()) * f.getAmplitude();
  polygon.addAngV(torque / polygon.getMoment() * dt);
}
