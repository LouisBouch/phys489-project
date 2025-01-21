#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"

////////////////////////////////////////////////////////////
env::Environment::Environment() : polygons(std::vector<bodies::Polygon>()) {}

////////////////////////////////////////////////////////////
env::Environment::Environment(std::vector<bodies::Polygon> polygons)
    : polygons(polygons) {}

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
