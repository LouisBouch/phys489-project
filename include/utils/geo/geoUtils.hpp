#pragma once

#include <eigen3/Eigen/Dense>
namespace utils::geo {
/**
 * @brief Test whether the segments ab and cd are intersecting.
 *
 * @param a First point of first segment.
 * @param b Second point of first segment.
 * @param c First point of second segment.
 * @param d Second point of second segment.
 *
 * @return Whether the segments intersect.
 */
bool testSegments(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                  const Eigen::Vector2d& c, const Eigen::Vector2d& d);

/**
 * @brief Determines if a point is inside a polygon or not.
 * Uses the Jordn curve theorem
 *
 * @param vs Vertices defining the polygon.
 * @param point Point to verify against.
 *
 * @return True if the point is inside the polygon.
 */
bool pointInPolygon(const Eigen::Matrix2Xd& vs, const Eigen::Vector2d& point);

/**
 * @brief Obtains twice the signed area of triangle abc. (Parallelogram area)
 *
 * @param a First point.
 * @param b Middle point.
 * @param c Last pont.
 *
 * @return Twice the signed area of given triangle. (Positive for
 * counterclockwise abc)
 */
double findParaArea(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                    const Eigen::Vector2d& c);
} // namespace utils
