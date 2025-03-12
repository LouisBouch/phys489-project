#include "physics/PhysicsEngine.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/collision/ColDetector.hpp"
#include "physics/collision/ColResponse.hpp"
#include "physics/integrator.hpp"
#include <chrono>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <ostream>
#include <fstream>
#include <thread>

////////////////////////////////////////////////////////////
physics::PhysicsEngine::PhysicsEngine() : physics::PhysicsEngine(0) {}

////////////////////////////////////////////////////////////
physics::PhysicsEngine::PhysicsEngine(double dt)
    : env(nullptr), running(false), dt(dt),
      colDet(physics::collision::ColDetector()), slowdown(1),
      colRes(collision::ColResponse()), saveSteps(false) {}

////////////////////////////////////////////////////////////
physics::PhysicsEngine::~PhysicsEngine() { joinThread(); }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::startSimLoop(double maxTime, std::string filename) {
  if (running) {
    return;
  }
  running = true;
  simThread = std::thread(&physics::PhysicsEngine::simLoop, this, maxTime, filename);
}

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::setPaused(bool paused) { this->paused = paused; }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::setSaveSteps(bool saveSteps) {
  this->saveSteps = saveSteps;
}

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::stopSimLoop() { running = false; }

////////////////////////////////////////////////////////////
nlohmann::json physics::PhysicsEngine::envToJson() {
  nlohmann::json step;
  step["time"].push_back(env->getTotalTime() / 1e6);

  // Push each polygon in the json.
  std::vector<env::bodies::Polygon> pols = env->getPolygons();
  for (env::bodies::Polygon& pol : pols) {
    nlohmann::json polygonJson;

    // Store centroid.
    const Eigen::Vector2d& cent = pol.getCentroid();
    polygonJson["centroid"] = {cent.x(), cent.y()};

    // Store global vertices.
    nlohmann::json globalVJ = nlohmann::json::array();
    const Eigen::Matrix2Xd& gs = pol.getGlobalVertices();
    for (int v = 0; v < gs.cols(); v++) {
      Eigen::Vector2d ve = gs.col(v);
      globalVJ.push_back({ve.x(), ve.y()});
    }
    polygonJson["global_vertices"] = globalVJ;

    // Store rotation.
    polygonJson["rotation"] = pol.getRotation();

    // Store angular velocity.
    polygonJson["angular_vel"] = pol.getAngV();

    // Store velocity.
    const Eigen::Vector2d& vel = pol.getVelocity();
    polygonJson["velocity"] = {vel.x(), vel.y()};

    // Store polygon id.
    polygonJson["id"] = pol.getTag();

    // Store in time step.
    step["polygons"].push_back(polygonJson);
  }
  env->unlockPolygons();
  return step;
}
////////////////////////////////////////////////////////////
void physics::PhysicsEngine::simLoop(double maxTime, std::string filename) {
  int microDt = dt * 1e6;
  nlohmann::json run;
  while (running) {
    // If not enough time in buffer, skip this iteration and wait.
    if (!env->addToTimeBuffer(-microDt)) {
      int sleep = (int)microDt - env->getTimeBuffer();
      std::this_thread::sleep_for(std::chrono::microseconds(sleep));
      continue;
    }
    // Save environent to a json.
    if (saveSteps) {
      run["time_steps"].push_back(envToJson());
    }
    // Exit after specified amount of time.
    if (env->getTotalTime() / 1e6 >= maxTime) {
      running = false;
      break;
    }
    // Step Environment.
    env->lockEnv(); // Ensure the environment is locked from the stepping all
                    // the way to penetration correction. Otherwise, the
                    // renderer might draw the polygon after the environment has
                    // been stepped but before the position was corrected.
    // Apply forces on polygons.
    applyForces(*env, dt * slowdown);

    // Detect collisions.
    colDet.findCollisions();

    // Resolve collisions.
    if (colDet.getCollisions().size() != 0) {
      colRes.resolveCollisions(colDet.getCollisions(), dt);
    }
    // Step the environment forward in time.
    stepEnvironment(*env, dt * slowdown);
    env->addToTotalTime(microDt * slowdown);

    env->unlockEnv();
  }
  // Save run
  if (filename == "") {
    return;
  }
  std::ofstream file(filename);
  if (!file) {
    std::cout << "Error opening file for writing." << std::endl;
  }

  file << run.dump(2);
  file.close();
}

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::joinThread() {
  if (simThread.joinable()) {
    simThread.join();
  }
}

////////////////////////////////////////////////////////////
env::Environment* physics::PhysicsEngine::getEnv() { return env; }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::setEnv(env::Environment* env) {
  colDet.setEnvironment(env);
  this->env = env;
}
////////////////////////////////////////////////////////////
physics::collision::ColDetector& physics::PhysicsEngine::getColDetector() {
  return colDet;
}

////////////////////////////////////////////////////////////
double physics::PhysicsEngine::getDt() const { return dt; }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::setDt(double dt) { this->dt = dt; }
