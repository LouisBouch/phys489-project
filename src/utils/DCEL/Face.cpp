#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Vertex.hpp"

////////////////////////////////////////////////////////////
utils::DCEL::Face::Face(HalfEdge* outerEdge) : outerEdge(outerEdge) {}

////////////////////////////////////////////////////////////
const utils::DCEL::HalfEdge* utils::DCEL::Face::getOuterEdge() const {
  return outerEdge;
}

////////////////////////////////////////////////////////////
void utils::DCEL::Face::setOuterEdge(HalfEdge* outerEdge) {
  this->outerEdge = outerEdge;
}
