#include "env/bodies/Square.hpp"
#include "env/bodies/Polygon.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

env::bodies::Square::Square()
    : Polygon(
          Eigen::MatrixXd{{0.5, 0.5}, {-0.5, 0.5}, {-0.5, -0.5}, {0.5, -0.5}}
              .transpose()) {}
env::bodies::Square::Square(Eigen::Vector2d centroid, double w, double r)
    : Polygon(Eigen::MatrixXd{{w / 2.0, w / 2.0},
                              {-w / 2.0, w / 2.0},
                              {-w / 2.0, -w / 2.0},
                              {w / 2.0, -w / 2.0}}
                  .transpose()) {
  rotate(r);
  translate(centroid);
}
