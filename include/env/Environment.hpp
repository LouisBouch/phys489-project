#pragma once
#include "bodies/Polygon.hpp"
#include "boundaries/Wall.hpp"
#include <vector>

namespace env {
class Environment {
public:
  /**
   * @brief Default constructor.
   */
  Environment();

  /**
   * @brief Parameterized constructor.
   *
   * @polygons List of polygons to add in the environment.
   */
  Environment(std::vector<bodies::Polygon> polygons);

  /**
   * @brief Deconstructor.
   */
  ~Environment();

  /**
   * @brief Adds a polygon to the environment.
   *
   * @param polygon Polygon to introduce in simulation.
   */
  void addPolygon(bodies::Polygon& polygon);

  /**
   * @brief Gets the polygon array.
   *
   * @return Polygons the array of polygons.
   */
  std::vector<bodies::Polygon>& getPolygons();

private:
  std::vector<bodies::Polygon> polygons; //< Array of polygons.
};
} // namespace env
