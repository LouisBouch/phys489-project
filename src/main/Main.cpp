#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "env/bodies/Square.hpp"
#include "physics/PhysicsEngine.hpp"
#include "ui/frame/SFMLWindow.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>

int main() {
  // env::bodies::Polygon p1(
  //     Eigen::MatrixXd{{0, 0}, {100, 0}, {0, 100}, {-100, 50}}.transpose(), 0,
  //     6, {12, 12});

  env::bodies::Polygon t1(
      Eigen::MatrixXd{{130, 50}, {130, 150}, {50, 100}}.transpose(), 0, 1,
      {-2, 5});

  env::bodies::Polygon c1(
      Eigen::MatrixXd{
          {0, 0}, {200, 0}, {200, 200}, {0, 200}, {150, 150}, {150, 50}}
          .transpose(),
      0, 0, {0, 0});
  env::bodies::Polygon c2(Eigen::MatrixXd{{300, 300},
                                          {400, 200},
                                          {300, 100},
                                          {300, 200},
                                          {200, 100},
                                          {300, 0},
                                          {500, 200}}
                              .transpose());
  env::bodies::Polygon c3(Eigen::MatrixXd{
      {710, 0},
      {860, 50},
      {810, 100},
      {910, 150},
      {860, 250},
      {590, 80},
      {710, 300},
      {660, 250},
      {510, 200},
      {610, 150},
      {560, 50},
  }
                              .transpose());
  env::bodies::Polygon c4(Eigen::MatrixXd{
      {710, 300},
      {860, 350},
      {810, 400},
      {910, 450},
      {860, 550},
      {800, 550},
      {710, 600},
      {660, 550},
      {510, 500},
      {610, 450},
      {560, 350},
  }
                              .transpose());

  // env::bodies::Polygon t3(
  //     Eigen::MatrixXd{{0, 250}, {150, 180}, {200, 250}}.transpose(), 0, 0,
  //     {0, 0});
  //
  // env::bodies::Polygon t4(
  //     Eigen::MatrixXd{{0, 190}, {150, 120}, {200, 190}}.transpose(), 0, 0,
  //     {0, 0});
  //
  // env::bodies::Polygon t5(
  //     Eigen::MatrixXd{{92, 131}, {101, 222}, {16, 190}}.transpose(), 0, 0,
  //     {0, 0});
  // env::bodies::Polygon t6(
  //     Eigen::MatrixXd{{3, 197}, {149, 149}, {197, 195}}.transpose(), 0, 0,
  //     {0, 0});
  //
  // env::bodies::Polygon t2(
  //     Eigen::MatrixXd{{150, 50}, {200, 50}, {150, 150}}.transpose());
  // env::bodies::Polygon t7(
  //     Eigen::MatrixXd{{200, 150}, {200, 250}, {100, 250}}.transpose(),
  //     35 / 360.0 * (2 * M_PI), 0);
  //
  env::bodies::Square s1(Eigen::Vector2d{150, 200}, 100, 0, 1, {10, 10});
  // env::bodies::Square s2(Eigen::Vector2d{100, 200}, 100,
  //                        45.0 / 360 * (2 * M_PI));

  // Create environment
  env::Environment simEnv;
  // simEnv.addPolygon(t7);
  // simEnv.addPolygon(s2);
  simEnv.addPolygon(t1);
  simEnv.addPolygon(c1);
  simEnv.addPolygon(c2);
  simEnv.addPolygon(c3);
  simEnv.addPolygon(c4);

  // Create physics engine
  physics::PhysicsEngine engine(0.01);
  engine.setEnv(&simEnv);
  engine.startSimLoop();

  // Create window and configure renderer with environment
  ui::frame::SFMLWindow window(engine);
  window.create("test");
  window.startWindowLoop();

  // Listen for window termination and then terminate physics engine
  window.joinThread();
  engine.stopSimLoop();
  engine.joinThread();
}
