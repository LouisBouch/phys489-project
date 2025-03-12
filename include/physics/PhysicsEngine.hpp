#pragma once
#include "env/Environment.hpp"
#include "nlohmann/json.hpp"
#include "physics/collision/ColDetector.hpp"
#include "physics/collision/ColResponse.hpp"
#include <limits>
#include <thread>
namespace physics {
class PhysicsEngine {
public:
  /**
   * @brief Default constructor.
   */
  PhysicsEngine();

  /**
   * @brief Parameterized constructor.
   *
   * @param dt Delta time with which the engine updates the environment. (In
   * seconds)
   */
  PhysicsEngine(double dt);

  /**
   * @brief The deconstructor.
   */
  ~PhysicsEngine();

  /**
   * @brief Starts the loop that executes physics computation.
   *
   * @maxTime Maximum time the simulation will run before terminating.
   * @filename Where to save the json.
   */
  void startSimLoop(double maxTime = std::numeric_limits<double>::infinity(), std::string filename = "");

  /**
   * @brief Sets the pause value of the sim loop.
   */
  void setPaused(bool paused);

  /**
   * @brief Stops the loop that runs the sim.
   */
  void stopSimLoop();

  /**
   * @brief Contains the loop propulsing the simulation forward.
   *
   * @maxTime Maximum time the simulation will run before terminating.
   * @filename Where to save the json.
   */
  void simLoop(double maxTime = std::numeric_limits<double>::infinity(), std::string filename = "");

  /**
   * @brief Getter for environment.
   *
   * @return The environment of the simulation.
   */
  env::Environment* getEnv();

  /**
   * @brief Setter for environment.
   *
   * @param env The environment of the simulation.
   */
  void setEnv(env::Environment* env);

  /**
   * @brief Gets the collision detector.
   *
   * @return The collision detector.
   */
  collision::ColDetector& getColDetector();

  /**
   * @brief Converts the current environment into a json.
   *
   * @return The json.
   */
  nlohmann::json envToJson();
  /**
   * @brief Getter for delta time.
   *
   * @return The delta time of the simulation. (In seconds)
   */
  double getDt() const;

  /**
   * @brief Setter for delta time.
   *
   * @param env The new delta time of the simulation.
   */
  void setDt(double dt);

  /**
   * @brief Setter for the save attribute.
   *
   * @param saveSteps Whether to save the time steps to a json or not.
   */
  void setSaveSteps(bool saveSteps);

  /**
   * @brief Joins the calling thread to the thread that runs the sim loop.
   */
  void joinThread();

private:
  env::Environment* env; //< Simulation content.
  double slowdown;       // Slows down time by the factor.
  bool running;          //< Whether the simulation is running or not.
  bool paused;           //< Whether the simulation is paused or not.
  std::thread simThread; //< Thread rendering the screen.
  double dt; //< Delta time with which the engine updates the environment. (In
             // seconds)
  physics::collision::ColDetector
      colDet; //< Used to detect collisions in the environment.
  physics::collision::ColResponse
      colRes;     //< Used to resolve collisions detected in the environment.
  bool saveSteps; //< Whether or not to save the iterations in a json.
};
} // namespace physics
