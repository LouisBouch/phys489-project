#include "utils/DCEL/Vertex.hpp"
#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"

////////////////////////////////////////////////////////////
utils::DCEL::Vertex::Vertex(int vertexI, HalfEdge* incidentEdge)
    : vertexI(vertexI), incidentEdge(incidentEdge) {}

////////////////////////////////////////////////////////////
int utils::DCEL::Vertex::getVertexI() { return vertexI; }

////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::Vertex::getIncidentEdge() {
  return incidentEdge;
}
////////////////////////////////////////////////////////////
void utils::DCEL::Vertex::setIncidentEdge(utils::DCEL::HalfEdge* incidentEdge) {
  this->incidentEdge = incidentEdge;
}
