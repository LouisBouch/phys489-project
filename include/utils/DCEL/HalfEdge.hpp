#pragma once

#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/Vertex.hpp"

// Forward declaration
class Face;
class Vertex;

namespace utils::DCEL {
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
  const Vertex* getOrigin() const;
  /**
   * @brief Gets twin half-edge.
   *
   * @return Twin half-edge.
   */
  const HalfEdge* getTwin() const;
  /**
   * @brief Gets next half-edge.
   *
   * @return Next half-edge.
   */
  const HalfEdge* getNext() const;
  /**
   * @brief Gets previous half-edge.
   *
   * @return Previous half-edge.
   */
  const HalfEdge* getPrev() const;
  /**
   * @brief Gets incident face.
   *
   * @return Incident face.
   */
  const Face* getIncidentFace() const;
  /**
   * @brief Gets destination vertex.
   *
   * @return Destination vertex.
   */
  const Vertex* getDestination() const;

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

private:
  Vertex* origin;     //< Vertex at the origin.
  HalfEdge* twin;     //< Twin half-edge.
  HalfEdge* next;     //< Next half-edge.
  HalfEdge* prev;     //< Previous half-edge.
  Face* incidentFace; //< Face incident to the half-edge.
};
} // namespace utils::DCEL
