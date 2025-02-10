#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
namespace utils::geo {

/**
 * @brief Triangulates a polygon.
 *
 * @param vertices Vertices to triangulate.
 *
 * @return List of indices representing the vertices of each triangle.
 */
Eigen::Matrix3Xi triangulate(const Eigen::Matrix2Xd& vertices);
/**
 * @brief Convexifies the polygon.
 *
 * @param vertices Vertices to convexify.
 * @param triangulation Triangulation of the vertices.
 *
 * @return List of indices representing the vertices of each convex polygon.
 */
std::vector<std::vector<int>> convexify(const Eigen::Matrix2Xd& vertices,
                                        const Eigen::Matrix3Xi& triangulation);
/**
 * @brief Determines whether polygon is convex or not.
 *
 * @return Convexity of polygon. (True if convex)
 */
bool findConvexity(const Eigen::Matrix2Xd& vertices);
} // namespace utils::geo

// 1. Create a DCEL from the triangualtion of the polygon.
//
// 2. Go through each half-edge in the DCEL, and check if the incident of both
// twin half-edges can be merged into a convex polygon. If so, go to step 3,
// else to step 4.
//
// 3. You must delete the half-edges between the polygons and merge the faces.
// Which can be done with the following instructions:
// 3.1 Update the incidentEdge pointer of the half-edges origin vertices as:
// e->origin.incidentEdge = e->twin.next.
// 3.2 Update the left face's outerEdge as such: eLeft->face.outerEdge = eLeft.next.
// 3.3 For every edge surrounding
// the right face, change their face pointer as such: e.face = eLeft.face.
// 3.3 Delete the right face.
// 3.4 Update next and previous edges from soon to be
// deleted as such: e->prev.next = e->twin.next, e->next.prev = e->twin.prev.
// 3.5 Delete both twin edges from the DCEL.
// 3.6 Go back to step 2.
//
// 4. If all half-edges have been checked, iterate over the faces of the DCEL
// and use these as the convex polygon decomposition. Else, go back to 2.
