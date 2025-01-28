#include "utils/geo/geoUtils.hpp"

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
