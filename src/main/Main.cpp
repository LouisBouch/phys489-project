#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "env/bodies/Square.hpp"
#include "physics/PhysicsEngine.hpp"
#include "ui/frame/SFMLWindow.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <ostream>

int main() {
  env::bodies::Polygon err(
      Eigen::MatrixXd{{0, 0}, {100, 0}, {0, 100}, {10, 50}}.transpose());
  env::bodies::Polygon err2(
      Eigen::MatrixXd{{0, 0}, {100, 0}, {120, 50}, {60, 30}, {10, 100}}
          .transpose());
  env::bodies::Square s(Eigen::Vector2d{100, 200}, 100,
                        15.0 / 360 * (2 * M_PI));
  env::bodies::Square s1(Eigen::Vector2d{100, 200}, 100, 0, 1, {10, 10});
  // Create environment
  env::Environment simEnv;
  // simEnv.addPolygon(s);
  simEnv.addPolygon(s1);
  // simEnv.addPolygon(err);

  // Create window and configure renderer with environment
  ui::frame::SFMLWindow window;
  window.create("test");
  window.getRenderer().setEnv(&simEnv);
  window.startWindowLoop();

  // Create physics engine
  physics::PhysicsEngine engine(0.01);
  engine.setEnv(&simEnv);
  engine.startSimLoop();

  // Listen for window termination and then terminate physics engine
  window.joinThread();
  engine.stopSimLoop();
  engine.joinThread();
}
