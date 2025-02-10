#pragma once

#include "utils/DCEL/HalfEdge.hpp"

namespace utils::DCEL {
// Forward declaration
class HalfEdge;

class Vertex {
public:
  /**
   * @brief Paramaterized constructor
   *
   * @param vertexI Index on the polygon the vertx points to.
   * @param incidentEdge Incident half-edge.
   */
  Vertex(int vertexI, HalfEdge* incidentEdge = nullptr);

  /**
   * @brief Getter for the vertex index.
   *
   * @return Vertex index.
   */
  int getVertexI();
  /**
   * @brief Getter for the incident edge of the vertex.
   *
   * @return Incident edge.
   */
  HalfEdge* getIncidentEdge();
  /**
   * @brief Setter for the incident edge of the vertex.
   *
   * @param incidentEdge Incident edge.
   */
  void setIncidentEdge(HalfEdge* incidentEdge);

private:
  int vertexI;            //< The index on the polygon the vertx points to.
  HalfEdge* incidentEdge; //< Incident half-edge.
};
} // namespace utils::DCEL
