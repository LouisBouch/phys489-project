#pragma once

#include "utils/DCEL/HalfEdge.hpp"

namespace utils::DCEL {
// Forward declaration
class HalfEdge;

class Face {
public:
  /**
   * @brief Paramaterized constructor
   *
   * @param incidentEdge Half-edge that bounds the face.
   */
  Face(HalfEdge* outerEdge = nullptr);

  /**
   * @brief Gets the outer edge.
   *
   * @return Outer half-edge.
   */
  HalfEdge* getOuterEdge();

  /**
   * @brief Gets the outer edge.
   *
   * @return Outer half-edge.
   */
  void setOuterEdge(HalfEdge* outerEdge);

private:
  HalfEdge* outerEdge; //< Half-edge that bounds the face.
};
} // namespace utils::DCEL
