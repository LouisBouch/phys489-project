#include "physics/PhysicsEngine.hpp"
#include "physics/collision/ColDetector.hpp"
#include "physics/collision/ColResponse.hpp"
#include "physics/integrator.hpp"
#include <chrono>
#include <iostream>
#include <thread>

////////////////////////////////////////////////////////////
physics::PhysicsEngine::PhysicsEngine() : physics::PhysicsEngine(0) {}

////////////////////////////////////////////////////////////
physics::PhysicsEngine::PhysicsEngine(double dt)
    : env(nullptr), running(false), dt(dt),
      colDet(physics::collision::ColDetector()), slowdown(1),
      colRes(collision::ColResponse()) {}

////////////////////////////////////////////////////////////
physics::PhysicsEngine::~PhysicsEngine() { joinThread(); }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::startSimLoop() {
  if (running) {
    return;
  }
  running = true;
  simThread = std::thread(&physics::PhysicsEngine::simLoop, this);
}

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::setPaused(bool paused) { this->paused = paused; }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::stopSimLoop() { running = false; }

////////////////////////////////////////////////////////////
void physics::PhysicsEngine::simLoop() {
  int microDt = dt * 1e6;
  while (running) {
    // If not enough time in buffer, skip this iteration and wait.
    if (!env->addToTimeBuffer(-microDt)) {
      int sleep = (int)microDt - env->getTimeBuffer();
      std::this_thread::sleep_for(std::chrono::microseconds(sleep));
      continue;
    }
    // Step Environment.
    env->lockEnv(); // Ensure the environment is locked from the stepping all
                    // the way to penetration correction. Otherwise, the
                    // renderer might draw the polygon after the environment has
                    // been stepped but before the position was corrected.
    // Apply forces on polygons.
    applyForces(*env, dt * slowdown);

    stepEnvironment(*env, dt * slowdown);
    env->addToTotalTime(microDt * slowdown);

    // Detect collisions.
    colDet.findCollisions();

    // Resolve collisions.
    if (colDet.getCollisions().size() != 0) {
      colRes.resolveCollisions(colDet.getCollisions(), dt);
    }
    env->unlockEnv();
  }
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
