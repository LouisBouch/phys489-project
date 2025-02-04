#include "physics/integrator.hpp"
#include "env/bodies/Polygon.hpp"
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
  polygon.addRot(polygon.getAngV() * dt);
}
