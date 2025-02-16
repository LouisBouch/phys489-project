#pragma once
#include "bodies/Polygon.hpp"
#include "boundaries/Wall.hpp"
#include "physics/collision/Collision.hpp"
#include <cstdint>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#define GRAV 500

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
   * @param grav Whether the shape is affected by gravity or not.
   */
  void addPolygon(bodies::Polygon& polygon, bool grav = true);

  /**
   * @brief Gets the polygon array. REQUIRES MANUAL UNLOCKING AFTER THE USER IS
   * DONE WITH THE POLYGONS.
   *
   * @return Polygons the array of polygons.
   */
  std::vector<bodies::Polygon>& getPolygons();

  /**
   * @brief Locks the polygons list.
   *
   */
  void lockPolygons();
  /**
   * @brief Unlocks the polygons list.
   *
   */
  void unlockPolygons();

  /**
   * @brief Locks the environment.
   *
   */
  void lockEnv();
  /**
   * @brief Unlocks the environment.
   *
   */
  void unlockEnv();

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

  /**
   * @brief Gets the polygons represented by its id. (Locks polygons, must be
   * manually unlocked aftewards.)
   *
   * @param id Id of the polygon to fetch.
   *
   * @return Polygon pointer corresonding to id.
   */
  std::optional<env::bodies::Polygon*> getPolyById(int id);

private:
  std::vector<bodies::Polygon> polygons; //< Array of polygons.
  int curId; //< ID for the next polygon to be introduced in environment.
  std::unordered_map<int, bodies::Polygon*>
      polyById; //< List of IDs and their affiliated polygon pointers.
  std::atomic<int_least64_t>
      timeBuffer; //< Time to be consumed by physics engine given by the window.
                  // Made atomic to ensure thread safety. (In microseconds)
  std::atomic<int_least64_t>
      totalTime; //< Time since start of simulation. (In microseconds)
                 // stores. (Only used for debug info for the renderer)
  mutable std::mutex polygons_m; //< Mutex for polygons list.
  mutable std::mutex env_m; //< Mutex for environment.
  mutable std::mutex polygons_mW; //< Waitlist mutex for the polygons mutex.
  mutable std::mutex env_mW; //< Waitlist mutex for the env mutex.
};
} // namespace env
