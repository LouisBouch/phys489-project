#pragma once

#include "env/bodies/Polygon.hpp"
#include <eigen3/Eigen/Dense>

namespace env::bodies {
class Square : public Polygon {
public:
  /**
   * @brief Default constructor.
   *
   * 1x1 square centered at the origin (0, 0)
   */
  Square();

  /**
   * @brief Default constructor.
   *
   * 1x1 square centered at the origin (0, 0)
   *
   * @param centroid Center of the square.
   * @param w Width of the edges.
   * @param r Roration of the square (Radian).
   */
  Square(Eigen::Vector2d centroid, double w = 1, double rot = 0,
         double angV = 0,
         const Eigen::Vector2d velocity = Eigen::Vector2d{0, 0});
};
} // namespace env::bodies
