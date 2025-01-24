#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include <cstdint>

////////////////////////////////////////////////////////////
env::Environment::Environment()
    : polygons(std::vector<bodies::Polygon>()), timeBuffer(0) {}

////////////////////////////////////////////////////////////
env::Environment::Environment(std::vector<bodies::Polygon> polygons)
    : polygons(polygons) {}

////////////////////////////////////////////////////////////
env::Environment::Environment(const Environment& env)
    : polygons(env.polygons), timeBuffer(env.timeBuffer.load()) {}

////////////////////////////////////////////////////////////
env::Environment::~Environment() {}

////////////////////////////////////////////////////////////
void env::Environment::addPolygon(bodies::Polygon& polygon) {
  polygons.push_back(polygon);
}

////////////////////////////////////////////////////////////
std::vector<env::bodies::Polygon>& env::Environment::getPolygons() {
  return polygons;
}

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
void env::Environment::addToTotalTime(int_least64_t time) {
  totalTime += time;
}
