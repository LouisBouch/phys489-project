#include "utils/geo/geoConvex.hpp"
#include "utils/geo/geoUtils.hpp"
#include <iostream>
#include <list>
#include <numeric>

////////////////////////////////////////////////////////////
std::vector<std::vector<int>>
utils::geo::convexify(const Eigen::Matrix2Xd& vertices,
                      const Eigen::Matrix3Xi& triangulation) {
  std::vector<std::vector<int>> convexification;
  // Start with the triangulation.
  for (int tri = 0; tri < triangulation.cols(); tri++) {
    convexification.push_back(std::vector<int>());
    for (int triV = 0; triV < 3; triV++) {
      convexification[tri].push_back(triangulation.col(tri)[triV]);
    }
  }
  // Goes through every triangle and see if it can remove an edge separating
  // it from another triangle.
  return convexification;
}

////////////////////////////////////////////////////////////
Eigen::Matrix3Xi utils::geo::triangulate(const Eigen::Matrix2Xd& vertices) {
  int nbVertices = vertices.cols();
  // Number of triangles found.
  int trisFound = 0;
  // List of current triangles in the triangulation.
  Eigen::Matrix3Xi tris(3, nbVertices - 2);

  // List of indices for each vertex in the polygon.
  std::list<int> indices(nbVertices);
  // Fill list with ascending numbers, starting at 0.
  std::iota(indices.begin(), indices.end(), 0);

  // Indices of the three points of a triangle.
  int a;
  int b;
  int c;

  // Triangle to be tested.
  Eigen::Matrix2Xd ttemp(2, 3);

  // Find triangles using ear clipping.
  std::list<int>::iterator it = indices.begin();
  while (trisFound != nbVertices - 2) {
    // If at first element, take the last. Otherwise, take the previous one.
    a = it == indices.begin() ? *std::prev(indices.end()) : *std::prev(it);
    ttemp.col(0) = vertices.col(a);
    b = *it;
    ttemp.col(1) = vertices.col(b);
    // If at last element, take the first. Otherwise, take the next one.
    c = it == std::prev(indices.end()) ? *indices.begin() : *std::next(it);
    ttemp.col(2) = vertices.col(c);

    // Check if vertex is a valid triangle, i.e. the vertex is convex.
    if (utils::geo::findParaArea(vertices.col(a), vertices.col(b),
                                 vertices.col(c)) < 0) {
      // The vertex is concave, thus invalid for a polygon ear.
      it++;
      continue;
    }

    // Check if another point is contained in the triangle.
    bool triValid = true;
    for (auto tit = indices.begin(); tit != indices.end(); tit++) {
      // Don't check for points that are part of the triangle
      if (*tit == a || *tit == b || *tit == c) {
        continue;
      }
      if (utils::geo::pointInPolygon(ttemp, vertices.col(*tit))) {
        // There is another point in the triangle, so the vertex is invalid.
        it++;
        triValid = false;
        break;
      }
    }
    // If the triangle is valid, add it to the list.
    if (triValid) {
      tris.col(trisFound) << a, b, c;
      trisFound++;
      // Remove vertex.
      indices.erase(it);
      // Reset iterator.
      it = indices.begin();
    }
  }
  return tris;
}
////////////////////////////////////////////////////////////
bool utils::geo::findConvexity(const Eigen::Matrix2Xd& vertices) {
  double a = 0;
  int nbCols = vertices.cols();
  for (int v = 0; v < nbCols; v++) {
    Eigen::Vector2d v1 =
        vertices.col((v + 2) % nbCols) - vertices.col((v + 1) % nbCols);
    Eigen::Vector2d v2 = vertices.col(v) - vertices.col((v + 1) % nbCols);
    // Check for angle > 180 deg.
    double cross = v1[0] * v2[1] - v2[0] * v1[1];
    if (cross < 0) {
      std::cout << "concave" << "\n";
      return false;
    }
    if (std::abs(cross) - 1e-6 < 0) {
      std::cout << "colinear, illegal!" << "\n";
      return false;
    }
  }
  return true;
}
