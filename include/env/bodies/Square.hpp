#pragma once

#include "Polygon.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>

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
  Square(Eigen::Vector2d centroid, double w = 1, double r = 0);
};
} // namespace env::bodies
