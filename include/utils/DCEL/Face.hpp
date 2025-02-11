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
   * @brief Gets the face id.
   *
   * @return Face id.
   */
  int getId();

  /**
   * @brief Gets the outer edge.
   *
   * @return Outer half-edge.
   */
  void setOuterEdge(HalfEdge* outerEdge);
  /**
   * @brief Sets the face id.
   *
   * @param id Face id.
   */
  void setId(int id);

private:
  int id;              //< ID of the face.
  HalfEdge* outerEdge; //< Half-edge that bounds the face.
};
} // namespace utils::DCEL
