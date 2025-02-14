#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/forces/Force.hpp"
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>

////////////////////////////////////////////////////////////
env::Environment::Environment() : Environment(std::vector<bodies::Polygon>()) {}

////////////////////////////////////////////////////////////
env::Environment::Environment(std::vector<bodies::Polygon> polygons)
    : polygons(polygons), timeBuffer(0), totalTime(0), curId(0) {}

////////////////////////////////////////////////////////////
env::Environment::Environment(const Environment& env)
    : polygons(env.polygons), timeBuffer(env.timeBuffer.load()) {}

////////////////////////////////////////////////////////////
env::Environment::~Environment() {}

////////////////////////////////////////////////////////////
void env::Environment::addPolygon(bodies::Polygon& polygon, bool grav) {
  polygons.push_back(polygon);
  polygons.back().setId(curId);
  // Remake the hashmap to prevent dangling reference after vector copy.
  for (int i = 0; i < polygons.size() - 1; i++) {
    polyById[polygons[i].getId()] = &polygons[i];
  }
  polyById[curId] = &polygons.back();
  if (grav) {
    polygons.back().addForce(physics::forces::ForceSource::Gravity, {0, 0},
                             {0, -1}, GRAV * polygon.getMass());
  }
  curId++;
}

////////////////////////////////////////////////////////////
std::vector<env::bodies::Polygon>& env::Environment::getPolygons() {
  polygons_m.lock();
  return polygons;
}

////////////////////////////////////////////////////////////
void env::Environment::unlockPolygons() { polygons_m.unlock(); }

////////////////////////////////////////////////////////////
bool env::Environment::addToTimeBuffer(int time) {
  if (time + timeBuffer.load() < 0) {
    return false;
  }
  timeBuffer.fetch_add(time);
  return true;
}

////////////////////////////////////////////////////////////
int_least64_t env::Environment::getTimeBuffer() { return timeBuffer.load(); }

////////////////////////////////////////////////////////////
int_least64_t env::Environment::getTotalTime() { return totalTime; }

////////////////////////////////////////////////////////////
void env::Environment::addToTotalTime(int_least64_t time) { totalTime += time; }

////////////////////////////////////////////////////////////
std::optional<env::bodies::Polygon*> env::Environment::getPolyById(int id) {
  try {
    env::bodies::Polygon* pp = polyById.at(id);
    return pp;
  } catch (std::out_of_range e) {
    return std::nullopt;
  }
}
