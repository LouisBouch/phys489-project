#include "utils/geo/geoConvex.hpp"
#include "utils/DCEL/DCEL.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/geo/geoUtils.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
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
  // Creates a DCEL from the triangulation.
  utils::DCEL::DCEL d =
      utils::DCEL::DCEL::makeDCEL(convexification, vertices.cols());

  // Convexifies the polygon by going through its DCEL edges and removing the
  // ones that can be removed.
  auto& halfEdgesMap = d.getHalfEdges();
  for (auto it = halfEdgesMap.begin(); it != halfEdgesMap.end();) {
    const auto& [key, edge] = *it;
    // If twin edge has been checked and determined to not be a candidate, then
    // don't check this one.
    if (!edge->isCandidate() || !edge->getTwin()->isCandidate()) {
      it++;
      continue;
    }

    // Check origin angle.
    Eigen::Vector2d a =
        vertices.col(edge->getPrev()->getOrigin()->getVertexI());
    Eigen::Vector2d b = vertices.col(edge->getOrigin()->getVertexI());
    Eigen::Vector2d c = vertices.col(
        edge->getTwin()->getNext()->getDestination()->getVertexI());
    double signedArea = utils::geo::findParaArea(a, b, c);
    // If area is negative, we have an angle > 180, which means we cannot remove
    // the edge without indtroducing concavity.
    if (signedArea < 0) {
      edge->setCandidate(false);
      it++;
      continue;
    }

    // Check destination angle.
    a = vertices.col(edge->getTwin()->getPrev()->getOrigin()->getVertexI());
    b = vertices.col(edge->getDestination()->getVertexI());
    c = vertices.col(edge->getNext()->getDestination()->getVertexI());
    signedArea = utils::geo::findParaArea(a, b, c);
    // If area is negative, we have an angle > 180, which means we cannot remove
    // the edge without indtroducing concavity.
    if (signedArea < 0) {
      edge->setCandidate(false);
      it++;
      continue;
    }
    // If both angles were less than 180 degrees, remove the half-edge and its
    // twin, but update the iterator first to ensure it doesn't get deleted with
    // the edge.
    it++;
    if (it != halfEdgesMap.end()) {
      // If not the last element, make sure the next element is not the twin.
      const auto& [keyN, _] = *it;
      if (keyN.second == key.first && keyN.first == key.second) {
        it++;
      }
    }
    d.removeEdge(key);
  }

  // From the list of faces, create the final convexification.
  convexification.clear();
  for (const auto& [key, face] : d.getFaces()) {
    // Add face to convexification.
    convexification.push_back(std::vector<int>());
    utils::DCEL::HalfEdge* startHE = face->getOuterEdge();
    utils::DCEL::HalfEdge* curHE = startHE;
    do {
      convexification.back().push_back(curHE->getOrigin()->getVertexI());
      // Get next edge of the face.
      curHE = curHE->getNext();
    } while (curHE != startHE);
  }

  // Debugging
  // for (auto& face : convexification) {
  //   for (int vertexI : face) {
  //     std::cout << vertexI << " ";
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << "\n";

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
      return false;
    }
    if (std::abs(cross) - 1e-6 < 0) {
      std::cout << "colinear, illegal!" << "\n";
      return false;
    }
  }
  return true;
}
