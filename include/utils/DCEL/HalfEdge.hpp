#pragma once

#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/Vertex.hpp"

namespace utils::DCEL {
// Forward declaration
class Face;
class Vertex;

class HalfEdge {
public:
  /**
   * @brief Paramaterized constructor
   *
   * @param origin Vertex at the origin.
   * @param twin Twin half-edge.
   * @param next Next half-edge.
   * @param prev Previous half-edge.
   * @param incidentFace Face incident to the half-edge.
   */
  HalfEdge(Vertex* origin = nullptr, HalfEdge* twin = nullptr,
           HalfEdge* next = nullptr, HalfEdge* prev = nullptr,
           Face* incidentFace = nullptr);

  /**
   * @brief Gets origin vertex.
   *
   * @return Origin vertex.
   */
  Vertex* getOrigin();
  /**
   * @brief Gets twin half-edge.
   *
   * @return Twin half-edge.
   */
  HalfEdge* getTwin();
  /**
   * @brief Gets next half-edge.
   *
   * @return Next half-edge.
   */
  HalfEdge* getNext();
  /**
   * @brief Gets previous half-edge.
   *
   * @return Previous half-edge.
   */
  HalfEdge* getPrev();
  /**
   * @brief Gets incident face.
   *
   * @return Incident face.
   */
  Face* getIncidentFace();
  /**
   * @brief Gets destination vertex.
   *
   * @return Destination vertex.
   */
  Vertex* getDestination();

  /**
   * @brief Gets canditate status of edge.
   *
   * @return Whether the edge could be removed without introducing concaveness.
   */
  bool isCandidate() const;

  /**
   * @brief Sets origin vertex.
   *
   * @param origin Origin vertex.
   */
  void setOrigin(Vertex* origin);
  /**
   * @brief Sets twin half-edge.
   *
   * @param twin Twin half-edge.
   */
  void setTwin(HalfEdge* twin);
  /**
   * @brief Sets next half-edge.
   *
   * @param next Next half-edge.
   */
  void setNext(HalfEdge* next);
  /**
   * @brief Sets previous half-edge.
   *
   * @param next Previous half-edge.
   */
  void setPrev(HalfEdge* prev);
  /**
   * @brief Sets incident face.
   *
   * @param face Incident face.
   */
  void setIncidentFace(Face* face);

  /**
   * @brief Sets canditate status of edge.
   *
   * @param candidate Whether the edge could be removed without introducing
   * concaveness.
   */
  void setCandidate(bool candidate);

private:
  bool candidate; //< If true, the edge will be considered when going over inner
                  // edges to convexify the polygon. If false, it implies the
                  // edge and its twin cannot be removed without introducing
                  // concaveness.
  Vertex* origin; //< Vertex at the origin.
  HalfEdge* twin; //< Twin half-edge.
  HalfEdge* next; //< Next half-edge.
  HalfEdge* prev; //< Previous half-edge.
  Face* incidentFace; //< Face incident to the half-edge.
};
} // namespace utils::DCEL
