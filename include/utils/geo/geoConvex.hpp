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

// 1. Create hashmap containing inner edges as key and objects as value. These
// objects will contain 4 vertices describing the 2 inner edge vertices and the
// vertices before and after the edge. Given edge (b,c), let the 4 vertices be
// as follows: (a,b,c,d). The first polygon will be left of the edge, and the
// second polygon will be the one on the right. Sort the edge vertices by index
// number.
// 1.1 Create another hashmap. Where each key is a polygon (list of
// vertices), and the value is a list of adjacent edges.
//
// 2. For each inner edge, check if adjacent convex polygons make a convex
// polygon. If true, go to step 3, else go to 4.
//
// 3. Merge both convex polygon into a bigger polygon.
// 3.1 Remove both polygons
// from the second hashmap and create another entry with the list of adjacent
// edges being the union of each list of adjacent edges from the recently
// deleted polygons. 3.2 For each adjacent edges of deleted polygon, update the
// values they contain to reflect the new merged polygon.
//
// 4. If all edges have been checked and none can be removed, go to 5. Else go
// back to 2.
//
// 5. Go through the second hashmap and form the list of convex polygon indices.
