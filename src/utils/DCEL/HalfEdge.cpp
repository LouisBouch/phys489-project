#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/Vertex.hpp"

////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge::HalfEdge(Vertex* origin, HalfEdge* twin, HalfEdge* next,
                                HalfEdge* prev, Face* incidentFace)
    : origin(origin), twin(twin), next(next), prev(prev),
      incidentFace(incidentFace), candidate(true) {}
////////////////////////////////////////////////////////////
utils::DCEL::Vertex* utils::DCEL::HalfEdge::getOrigin() {
  return origin;
}
////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::HalfEdge::getTwin() {
  return twin;
}
////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::HalfEdge::getNext() {
  return next;
}
////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::HalfEdge::getPrev() {
  return prev;
}
////////////////////////////////////////////////////////////
utils::DCEL::Face* utils::DCEL::HalfEdge::getIncidentFace() {
  return incidentFace;
}
////////////////////////////////////////////////////////////
utils::DCEL::Vertex* utils::DCEL::HalfEdge::getDestination() {
  return twin ? twin->origin : nullptr;
}
////////////////////////////////////////////////////////////
bool utils::DCEL::HalfEdge::isCandidate() const { return candidate; }

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
////////////////////////////////////////////////////////////
void utils::DCEL::HalfEdge::setCandidate(bool candidate) {
  this->candidate = candidate;
}
