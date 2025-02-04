#include "physics/PhysicsEngine.hpp"
#include "physics/collision/ColDetector.hpp"
#include "physics/integrator.hpp"
#include <chrono>
#include <iostream>
#include <thread>

////////////////////////////////////////////////////////////
physics::PhysicsEngine::PhysicsEngine() : physics::PhysicsEngine(0) {}

////////////////////////////////////////////////////////////
physics::PhysicsEngine::PhysicsEngine(double dt)
    : env(nullptr), running(false), dt(dt),
      colDet(physics::collision::ColDetector()), slowdown(1) {}

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
    // If not enough time in buffer
    if (!env->addToTimeBuffer(-microDt)) {
      std::this_thread::sleep_for(
          std::chrono::microseconds((int)microDt - env->getTimeBuffer()));
      continue;
    }
    // Step Environment.
    stepEnvironment(*env, dt*slowdown);

    // Detect collisions.
    colDet.findCollisions();

    // Resolve collisions.
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
  env->setCollisionsP(&colDet.getCollisions());
  this->env = env;
}
