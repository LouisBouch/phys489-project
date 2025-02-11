#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Vertex.hpp"

////////////////////////////////////////////////////////////
utils::DCEL::Face::Face(HalfEdge* outerEdge) : outerEdge(outerEdge) {}

////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::Face::getOuterEdge() {
  return outerEdge;
}
////////////////////////////////////////////////////////////
int utils::DCEL::Face::getId() {
  return id;
}

////////////////////////////////////////////////////////////
void utils::DCEL::Face::setOuterEdge(HalfEdge* outerEdge) {
  this->outerEdge = outerEdge;
}
////////////////////////////////////////////////////////////
void utils::DCEL::Face::setId(int id) {
  this->id = id;
}
