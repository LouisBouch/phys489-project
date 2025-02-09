#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/Vertex.hpp"

////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge::HalfEdge(Vertex* origin, HalfEdge* twin, HalfEdge* next,
                                HalfEdge* prev, Face* incidentFace)
    : origin(origin), twin(twin), next(next), prev(prev),
      incidentFace(incidentFace) {}
////////////////////////////////////////////////////////////
const utils::DCEL::Vertex* utils::DCEL::HalfEdge::getOrigin() const {
  return origin;
}
////////////////////////////////////////////////////////////
const utils::DCEL::HalfEdge* utils::DCEL::HalfEdge::getTwin() const {
  return twin;
}
////////////////////////////////////////////////////////////
const utils::DCEL::HalfEdge* utils::DCEL::HalfEdge::getNext() const {
  return next;
}
////////////////////////////////////////////////////////////
const utils::DCEL::HalfEdge* utils::DCEL::HalfEdge::getPrev() const {
  return prev;
}
////////////////////////////////////////////////////////////
const utils::DCEL::Face* utils::DCEL::HalfEdge::getIncidentFace() const {
  return incidentFace;
}
////////////////////////////////////////////////////////////
const utils::DCEL::Vertex* utils::DCEL::HalfEdge::getDestination() const {
  return twin ? twin->origin : nullptr;
}
////////////////////////////////////////////////////////////
void utils::DCEL::HalfEdge::setOrigin(Vertex* origin) { this->origin = origin; }
////////////////////////////////////////////////////////////
void utils::DCEL::HalfEdge::setTwin(HalfEdge* twin) { this->twin = twin; }
////////////////////////////////////////////////////////////
void utils::DCEL::HalfEdge::setNext(HalfEdge* next) { this->next = next; }
////////////////////////////////////////////////////////////
void utils::DCEL::HalfEdge::setPrev(HalfEdge* prev) { this->prev = prev; }
////////////////////////////////////////////////////////////
void utils::DCEL::HalfEdge::setIncidentFace(Face* face) {
  this->incidentFace = face;
}
