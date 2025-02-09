#pragma once

#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Vertex.hpp"
#include <memory>
#include <vector>

namespace utils::DCEL {
class DCEL {
public:
  /**
   * @brief Adds vertex to DCEL.
   *
   * @param vertexI The index on the polygon the vertx points to.
   *
   * @return Pointer to newly created vertex. (Required to set the (1)incident
   * edge)
   */
  Vertex* addVertex(int vertexI);
  /**
   * @brief Adds edge to DCEL.
   *
   * @return Pointer to newly created edge. (Required to set the (1)origin
   * vertex, (2)twin half-edge, (3)incident face, (4)next half-edge, and (5)prev
   * half-edge)
   */
  HalfEdge* addHalfEdge();
  /**
   * @brief Adds face to DCEL.
   *
   * @return Pointer to newly created face. (Required to set the (1)outer
   * half-edge)
   */
  Face* addFace();

  /**
   * @brief Gets vertices of the DCEL.
   *
   * @return Vertices of the DCEL.
   */
  const auto& getVertices() const;
  /**
   * @brief Gets half-edges of the DCEL.
   *
   * @return Half-edges of the DCEL.
   */
  const auto& getHalfEdges() const;
  /**
   * @brief Gets faces of the DCEL.
   *
   * @return Faces of the DCEL.
   */
  const auto& getFaces() const;

private:
  std::vector<std::unique_ptr<Vertex>>
      vertices; //< List of vertices in the DCEL.
  std::vector<std::unique_ptr<HalfEdge>>
      halfEdges;                            //< List of halfedges in the DCEL.
  std::vector<std::unique_ptr<Face>> faces; //< List of faces in the DCEL.
};
} // namespace utils::DCEL
