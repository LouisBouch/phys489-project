#include "env/Environment.hpp"
#include "env/bodies/Square.hpp"
#include "ui/frame/SFMLWindow.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ostream>

int main() {
  env::bodies::Square s(Eigen::Vector2d{100, 200}, 100, 45.0/360*(2*M_PI));
  env::bodies::Square s1(Eigen::Vector2d{100, 200}, 100);
  // Create environment
  env::Environment simEnv;
  simEnv.addPolygon(s);
  simEnv.addPolygon(s1);

  // Create window and configure renderer with environment
  ui::frame::SFMLWindow window;
  window.create("test");
  window.getRenderer().setEnv(simEnv);
  window.joinThread();

  // Create physics engine
}
