#pragma once
#include "bodies/Polygon.hpp"
#include "boundaries/Wall.hpp"
#include <cstdint>
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
   * @param polygons List of polygons to add in the environment.
   */
  Environment(std::vector<bodies::Polygon> polygons);

  /**
   * @brief Copy constructor.
   *
   * @param env Environment to copy
   */
  Environment(const Environment& env);

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

  /**
   * @brief Add time to the buffer. (Remove with negative time value)
   *
   * @param time Amount of time to be added/removed. (In microseconds)
   *
   * @return Whether or not the time was added. (Fails if timBuffer + time < 0)
   */
  bool addToTimeBuffer(int time);

  /**
   * @brief Gets remaining time buffer.
   *
   * @return Amount of time left in buffer. (In microseconds)
   */
  int_least64_t getTimeBuffer();
  /**
   * @brief Gets total time since start of sim.
   *
   * @return Amount of time since start of sim. (In microseconds)
   */
  int_least64_t getTotalTime();
  /**
   * @brief Adds to total time of sim.
   *
   * @param time Amount of time to add to the sim. (In microseconds)
   */
  void addToTotalTime(int_least64_t time);

private:
  std::vector<bodies::Polygon> polygons; //< Array of polygons.
  std::atomic<int_least64_t> timeBuffer; //< Time to be consumed by physics engine given by the window. Made atomic to ensure thread safety. (In microseconds)
  std::atomic<int_least64_t>
      totalTime; //< Time since start of simulation. (In microseconds)
};
} // namespace env
