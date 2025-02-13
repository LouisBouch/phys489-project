#include "utils/geo/geoUtils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <optional>

#define EPSILON 1e-6

////////////////////////////////////////////////////////////
double utils::geo::findParaArea(const Eigen::Vector2d& a,
                                const Eigen::Vector2d& b,
                                const Eigen::Vector2d& c) {
  return (c[0] - b[0]) * (a[1] - b[1]) - (c[1] - b[1]) * (a[0] - b[0]);
}

////////////////////////////////////////////////////////////
bool utils::geo::testSegments(const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b,
                              const Eigen::Vector2d& c,
                              const Eigen::Vector2d& d) {
  // Sign of areas correspond to which side of ab the points c and d appear.
  double a1 = utils::geo::findParaArea(a, b, d);
  double a2 = utils::geo::findParaArea(a, b, c);
  // For intersection to occur, a1 and a2 must have opposite signs.
  if (a1 * a2 < 0.0) {
    // Sign of areas correspond to which side of cd the points a and b appear.
    double a3 = utils::geo::findParaArea(c, d, a);
    // Given that a1 - a2 = a3 - a4 (Areas are the same)
    double a4 = a3 + a2 - a1;
    // For intersection to occur, a3 and a4 must also have opposite signs.
    if (a3 * a4 < 0.0) {
      // Can also find intersection point.
      // Given:
      // h1:    Height from segment cd to point a.
      // h2:    Height from segment cd to point b. (negative value)
      // t:     Parameterization value allowing to go from a to b.
      // theta: Angle between normal vector from cd and ab.
      // L(t) = a + t*(b-a): Point along segment ab.
      // L(t0): Intersection point between ab an cd.
      // h(L(t)) = h1 - t*ab*cos(theta): Height from cd to point L(t).
      // h(L(t)) = h1 - t*(h1-h2)
      // h(L(t0)) = 0 = h1 - t0*(h1-h2)
      // t0 = h1 / (h1 - h2) = (cd/2*h1) / (cd/2*h1 - cd/2*h2) = a3 / (a3 - a4)
      //
      // double t0 = a3 / (a3 - a4);
      // Eigen::Vector2d p = a + t*(b - a);
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////
bool utils::geo::pointInPolygon(const Eigen::Matrix2Xd& vs,
                                const Eigen::Vector2d& point) {
  bool inside = false;
  int nbV = vs.cols(); // Number of vertices in polygon.
  int curV = 0;        // Current vertex index.
  int lastV = 0;       // Last vertex index.
  for (curV = 0, lastV = nbV - 1; curV < nbV; lastV = curV++) {
    // First (before &&), check if point is between the vertices in the vertical
    // direction. After (After &&), check if the point is on the left of the
    // edge given by curV and lastV.
    if (((vs.col(curV)[1] > point[1]) != (vs.col(lastV)[1] > point[1])) &&
        (point[0] < (vs.col(lastV)[0] - vs.col(curV)[0]) /
                            (vs.col(lastV)[1] - vs.col(curV)[1]) *
                            (point[1] - vs.col(curV)[1]) +
                        vs.col(curV)[0])) {
      // Given that the point is between the two vertices (vertically) and on
      // the left, the ray will cross this edge as it goes right.
      inside = !inside;
    }
  }
  // If the ray crossed an odd number of edges, it must be inside the polygon.
  return inside;
}
////////////////////////////////////////////////////////////
Eigen::Matrix2Xd utils::geo::rotatePoints(const Eigen::Matrix2Xd& ps,
                                          double r) {
  // Create rotation matrix
  double cosr = std::cos(r);
  double sinr = std::sin(r);
  Eigen::Matrix2d rM{{cosr, -sinr}, {sinr, cosr}};
  // Rotate point
  return rM * ps;
}
////////////////////////////////////////////////////////////
Eigen::Matrix2Xd utils::geo::projectPoints(const Eigen::Matrix2Xd& ps,
                                           const Eigen::Vector2d& a) {
  return a * (a.transpose() * ps / a.dot(a));
}
////////////////////////////////////////////////////////////
Eigen::VectorXd utils::geo::projectPointsMagnitude(const Eigen::Matrix2Xd& ps,
                                                   const Eigen::Vector2d& a) {
  return a.transpose() * ps / a.norm();
}
////////////////////////////////////////////////////////////
double utils::geo::findOverlap(const Eigen::VectorXd& ps1,
                               const Eigen::VectorXd& ps2) {
  double min1 = ps1.minCoeff();
  double max1 = ps1.maxCoeff();

  double min2 = ps2.minCoeff();
  double max2 = ps2.maxCoeff();

  double minMax = max1 < max2 ? max1 : max2;
  double maxMin = min1 > min2 ? min1 : min2;
  double overlap = minMax - maxMin;
  // Equivalent to:
  // std::max(0,std::min(max1, max2) - std::max(min1, min2));
  return 0 > overlap ? 0 : overlap;
}
////////////////////////////////////////////////////////////
int utils::geo::mostAligned(const Eigen::Vector2d& v1,
                            const Eigen::Vector2d& v2,
                            const Eigen::Vector2d& r) {
  if (std::abs((v1.dot(v2) - v1.norm() * v2.norm())) - EPSILON < 0) {
    // v1 and v2 are paralell
    return 0;
  }
  double cosinet1 = std::abs(v1.dot(r) / (v1.norm() * r.norm()));
  double cosinet2 = std::abs(v2.dot(r) / (v2.norm() * r.norm()));
  // Arccos of these values would yield the serparation between the vectors.
  // Given that arccos is monotonously decreasing, the greater cosinet will
  // correspond to the most aligned vector.
  return cosinet1 > cosinet2 ? 1 : -1;
}
////////////////////////////////////////////////////////////
bool utils::geo::isAboveLine(const Eigen::Vector2d& p, const Eigen::Vector2d& n,
                             const Eigen::Vector2d& pl) {
  double projP = projectPointsMagnitude(p - pl, n)[0];
  return projP >= 0;
}

////////////////////////////////////////////////////////////
bool utils::geo::segCrossesLine(const Eigen::Vector2d& p1,
                                const Eigen::Vector2d& p2,
                                const Eigen::Vector2d& n,
                                const Eigen::Vector2d& pl) {
  double d1 = projectPointsMagnitude(p1 - pl, n)[0];
  double d2 = projectPointsMagnitude(p2 - pl, n)[0];
  return d1 * d2 < 0;
}

////////////////////////////////////////////////////////////
std::optional<Eigen::Vector2d>
utils::geo::interSegLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                         const Eigen::Vector2d& n, const Eigen::Vector2d& pl) {
  // Check if there is an intersection
  if (!segCrossesLine(p1, p2, n, pl)) {
    return std::nullopt;
  }
  // Point can be defined as P = p1 + t * (p2 - p1)
  // Where t can be found to be:
  double t = n.dot(pl - p1) / n.dot(p2 - p1);
  return p1 + t * (p2 - p1);
}
////////////////////////////////////////////////////////////
double utils::geo::cross2D(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x()*b.y() - a.y()*b.x();
}
////////////////////////////////////////////////////////////
int utils::geo::sign(double v) {
  return (v > 0) - (v < 0);
}
////////////////////////////////////////////////////////////
double utils::geo::signedAngle(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return std::atan2(cross2D(a, b), a.dot(b));
}
