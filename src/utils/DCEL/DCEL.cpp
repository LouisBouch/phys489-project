#include "utils/DCEL/DCEL.hpp"
#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Vertex.hpp"
#include <memory>

////////////////////////////////////////////////////////////
utils::DCEL::Vertex* utils::DCEL::DCEL::addVertex(int vertexI) {
  vertices.emplace_back(std::make_unique<Vertex>(vertexI));
  return vertices.back().get();
}

////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::DCEL::addHalfEdge() {
  vertices.emplace_back(std::make_unique<HalfEdge>());
  return halfEdges.back().get();
}

////////////////////////////////////////////////////////////
utils::DCEL::Face* utils::DCEL::DCEL::addFace() {
  vertices.emplace_back(std::make_unique<Face>());
  return faces.back().get();
}

////////////////////////////////////////////////////////////
const auto& utils::DCEL::DCEL::getVertices() const { return vertices; }

////////////////////////////////////////////////////////////
const auto& utils::DCEL::DCEL::getHalfEdges() const { return halfEdges; }

////////////////////////////////////////////////////////////
const auto& utils::DCEL::DCEL::getFaces() const { return faces; }
