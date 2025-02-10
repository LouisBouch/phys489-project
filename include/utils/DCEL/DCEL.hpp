#pragma once

#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Vertex.hpp"
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace utils::DCEL {
// Hash for a pair of ints.
struct PairHash {
  std::size_t operator()(const std::pair<int, int>& p) const {
    std::size_t h1 = std::hash<int>{}(p.first);
    std::size_t h2 = std::hash<int>{}(p.second);
    return h1 ^ (31 * h2);
  }
};
// Hash and equality for a unique ptr.
struct uPtrHash {
  template <typename T>
  std::size_t operator()(const std::unique_ptr<T>& ptr) const {
    return std::hash<T*>{}(ptr.get());
  }
};
struct uPtrEq {
  template <typename T>
  bool operator()(const std::unique_ptr<T>& l,
                         const std::unique_ptr<T>& r) const {
    return l.get() == r.get();
  }
};
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
   * @param vs Pair of connected vertices. (vs.first is origin)
   *
   * @return Pointer to newly created edge. (Required to set the (1)origin
   * vertex, (2)twin half-edge, (3)incident face, (4)next half-edge, and (5)prev
   * half-edge)
   */
  HalfEdge* addHalfEdge(std::pair<int, int> vs);
  /**
   * @brief Removes an half-edge and its twin.
   *
   * @param vs Key of any half-edge of the full edge.
   */
  void removeEdge(std::pair<int, int> vs);
  /**
   * @brief Adds face to DCEL.
   *
   * @return Pointer to newly created face. (Required to set the (1)outer
   * half-edge)
   */
  Face* addFace();

  /**
   * @brief Creates a DCEL from a list of faces defind in CCW order.
   *
   * @param faces List of list of vertex indices. Each list defines a face in CCW
   * order.
   * @param nbVertices Total number of vertices in the original polygon.
   *
   * @return DCEL formed from the list of faces.
   */
  static DCEL makeDCEL(const std::vector<std::vector<int>>& faces, int nbVertices);

  /**
   * @brief Gets vertices of the DCEL.
   *
   * @return Vertices of the DCEL.
   */
  const std::unordered_map<int, std::unique_ptr<Vertex>>& getVertices() const;
  /**
   * @brief Gets half-edges of the DCEL.
   *
   * @return Half-edges of the DCEL.
   */
  const std::unordered_map<std::pair<int, int>, std::unique_ptr<HalfEdge>,
                           PairHash>&
  getHalfEdges() const;
  /**
   * @brief Gets faces of the DCEL.
   *
   * @return Faces of the DCEL.
   */
  const std::unordered_set<std::unique_ptr<Face>, uPtrHash, uPtrEq>& getFaces() const;

private:
  std::unordered_map<int, std::unique_ptr<Vertex>>
      vertices; //< List of vertices in the DCEL.
  std::unordered_map<std::pair<int, int>, std::unique_ptr<HalfEdge>, PairHash>
      halfEdges; //< List of halfedges in the DCEL.
  std::unordered_set<std::unique_ptr<Face>, uPtrHash, uPtrEq>
      faces; //< List of faces in the DCEL.
};
} // namespace utils::DCEL
