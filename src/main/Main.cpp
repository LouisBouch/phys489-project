#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "env/bodies/Square.hpp"
#include "physics/PhysicsEngine.hpp"
#include "physics/collision/colDetection.hpp"
#include "ui/frame/SFMLWindow.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <ostream>

int main() {
  env::bodies::Polygon c1(
      Eigen::MatrixXd{
          {0, 0}, {200, 0}, {200, 200}, {0, 200}, {150, 150}, {150, 50}}
          .transpose(),
      0, 0, {0, 0});

  env::bodies::Polygon p1(
      Eigen::MatrixXd{{0, 0}, {100, 0}, {0, 100}, {-100, 50}}.transpose(), 0, 6,
      {12, 12});

  env::bodies::Polygon t1(
      Eigen::MatrixXd{{130, 50}, {130, 150}, {50, 100}}.transpose(), 0, 1,
      {-2, 5});

  env::bodies::Square s1(Eigen::Vector2d{150, 200}, 100, 0, 6, {10, 10});
  env::bodies::Square s2(Eigen::Vector2d{100, 200}, 100,
                         15.0 / 360 * (2 * M_PI));

  // Create environment
  env::Environment simEnv;
  simEnv.addPolygon(c1);
  simEnv.addPolygon(t1);

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
