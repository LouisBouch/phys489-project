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
void env::Environment::addPolygon(bodies::Polygon& polygon) {
  polygons.push_back(polygon);
  bodies::Polygon& curPol = polygons.back();
  curPol.setId(curId);
  // Remake the hashmap to prevent dangling reference after vector copy.
  for (int i = 0; i < polygons.size() - 1; i++) {
    polyById[polygons[i].getId()] = &polygons[i];
  }
  polyById[curId] = &curPol;
  curPol.addForce(physics::forces::ForceSource::Gravity, {0, 0},
                           {0, -1}, GRAV * polygon.getMass());
  curId++;
}

////////////////////////////////////////////////////////////
std::vector<env::bodies::Polygon>& env::Environment::getPolygons() {
  polygons_mW.lock();
  polygons_m.lock();
  polygons_mW.unlock();
  return polygons;
}
////////////////////////////////////////////////////////////
void env::Environment::lockPolygons() {
  polygons_mW.lock();
  polygons_m.lock();
  polygons_mW.unlock();
}

////////////////////////////////////////////////////////////
void env::Environment::unlockPolygons() { polygons_m.unlock(); }

////////////////////////////////////////////////////////////
void env::Environment::lockEnv() {
  env_mW.lock();
  env_m.lock();
  env_mW.unlock();
}

////////////////////////////////////////////////////////////
void env::Environment::unlockEnv() { env_m.unlock(); }

////////////////////////////////////////////////////////////
bool env::Environment::addToTimeBuffer(int time) {
  int_least64_t current = timeBuffer.load();
  while (true) {
    if (time + current < 0) {
      return false;
    }
    if (timeBuffer.compare_exchange_weak(current, current + time)) {
      return true;
    }
  }
}

////////////////////////////////////////////////////////////
int_least64_t env::Environment::getTimeBuffer() { return timeBuffer.load(); }

////////////////////////////////////////////////////////////
int_least64_t env::Environment::getTotalTime() { return totalTime; }

////////////////////////////////////////////////////////////
void env::Environment::addToTotalTime(int_least64_t time) {
  int_least64_t current = totalTime.load();
  while (true) {
    if (totalTime.compare_exchange_weak(current, current + time)) {
      return;
    }
  }
}

////////////////////////////////////////////////////////////
std::optional<env::bodies::Polygon*> env::Environment::getPolyById(int id) {
  try {
    env::bodies::Polygon* pp = polyById.at(id);
    return pp;
  } catch (std::out_of_range e) {
    return std::nullopt;
  }
}
