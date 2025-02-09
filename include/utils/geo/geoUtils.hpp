#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <optional>
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
/**
 * @brief Rotates points around origin.
 *
 * @param p Points to rotate.
 * @param r Rotation value. (In radian)
 *
 * @return Rotated point.
 */
Eigen::Matrix2Xd rotatePoints(const Eigen::Matrix2Xd& p, double r);

/**
 * @brief Projects a list of points onto a vector.
 *
 * @param p Points to project.
 * @param a Vector to project upon.
 *
 * @return List of projected points.
 */
Eigen::Matrix2Xd projectPoints(const Eigen::Matrix2Xd& p,
                               const Eigen::Vector2d& a);

/**
 * @brief Projects a list of poins onto a vector, but only check magnitude.
 *
 * @param p Points to project.
 * @param a Vector to project upon.
 *
 * @return List of projected point magnitudes
 */
Eigen::VectorXd projectPointsMagnitude(const Eigen::Matrix2Xd& p,
                                       const Eigen::Vector2d& a);

/**
 * @brief Check overlap between two sets of points on a line.
 *
 * @param p1 First set of points.
 * @param p2 Second set of points.
 *
 * @return Overlap between two sets of points. (0 if they don't overlap,
 * positive otherwise)
 */
double findOverlap(const Eigen::VectorXd& p1, const Eigen::VectorXd& p2);

/**
 * @brief Finds which vector is most aligned with a reference vector.
 *
 * @param v1 First vector.
 * @param v2 Second vector.
 * @param r Reference bector.
 *
 * @return 1 if v1 is most aligned, -1 if v2 is most aligned, and 0 if v1 and v2
 * are parallel.
 */
int mostAligned(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2,
                const Eigen::Vector2d& r);

/**
 * @brief Determines if a point is above or on a line.
 *
 * @param p The point to check.
 * @param n Normal to the line.
 * @param pl Point on the line.
 *
 * @return Whether the point is above or on the line (Above line in direction of
 * normal).
 */
bool isAboveLine(const Eigen::Vector2d& p, const Eigen::Vector2d& n,
                 const Eigen::Vector2d& pl);

/**
 * @brief Determiens if a segment crosses a line.
 *
 * @param p1 First point on segment.
 * @param p2 Second point on segment.
 * @param n Normal to the line.
 * @param pl Point on the line.
 *
 * @return Whether the segment fully crosses the line.
 */
bool segCrossesLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                    const Eigen::Vector2d& n, const Eigen::Vector2d& pl);

/**
 * @brief Determines where a segment intersects a line.
 *
 * @param p1 First point on segment.
 * @param p2 Second point on segment.
 * @param n Normal to the line.
 * @param pl Point on the line.
 *
 * @return Whether the segment fully crosses the line, and if so, where.
 */
std::optional<Eigen::Vector2d> interSegLine(const Eigen::Vector2d& p1,
                                            const Eigen::Vector2d& p2,
                                            const Eigen::Vector2d& n,
                                            const Eigen::Vector2d& pl);

/**
 * @brief Performs 2D cross product (Assume 0 z value for both vectors).
 *
 * @param a First vector.
 * @param b Second vector.
 *
 * @return The z value of the cross product axb.
 */
double cross2D(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
/**
 * @brief Finds the angle between the two vectors..
 *
 * @param a First vector.
 * @param b Second vector.
 *
 * @return The angle between a and b. (In radians, and positive if b is left of a)
 */
double signedAngle(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
/**
 * @brief Finds the sign of a double.
 *
 * @param v Value to obtain the sign from.
 *
 * @return Sign of value. (returns 0 if value is 0)
 */
int sign(double v);
} // namespace utils::geo
