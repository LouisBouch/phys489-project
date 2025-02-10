#include "utils/DCEL/DCEL.hpp"
#include "utils/DCEL/Face.hpp"
#include "utils/DCEL/HalfEdge.hpp"
#include "utils/DCEL/Vertex.hpp"
#include <iostream>
#include <memory>
#include <utility>

////////////////////////////////////////////////////////////
utils::DCEL::Vertex* utils::DCEL::DCEL::addVertex(int vertexI) {
  auto v = vertices.find(vertexI);
  // If vertex already exists, return it.
  if (v != vertices.end()) {
    return v->second.get();
  }

  // Otherwise, create the vertex and add it to the map.
  auto newV = std::make_unique<Vertex>(vertexI);
  Vertex* newVP = newV.get();
  vertices[vertexI] = std::move(newV);

  return newVP;
}

////////////////////////////////////////////////////////////
utils::DCEL::HalfEdge* utils::DCEL::DCEL::addHalfEdge(std::pair<int, int> vs) {
  auto he = halfEdges.find(vs);
  // If half-edge already exists, return it.
  if (he != halfEdges.end()) {
    return he->second.get();
  }

  // Otherwise, create the edge and add it to the map.
  auto newHE = std::make_unique<HalfEdge>();
  HalfEdge* newHEP = newHE.get();
  halfEdges[vs] = std::move(newHE);

  // Check if twin exists, and if so, bind them together.
  auto twinHe = halfEdges.find({vs.second, vs.first});
  if (twinHe != halfEdges.end()) {
    HalfEdge* twinHEP = twinHe->second.get();
    twinHEP->setTwin(newHEP);
    newHEP->setTwin(twinHEP);
  }

  // If source vertex exists, make it the origin vertex. Also, if this origin
  // does not have an incident edge, make this new edge the incident edge.
  auto v = vertices.find(vs.first);
  if (v != vertices.end()) {
    newHEP->setOrigin(v->second.get());

    if (v->second.get()->getIncidentEdge() == nullptr) {
      v->second.get()->setIncidentEdge(newHEP);
    }
  }
  return newHEP;
}
////////////////////////////////////////////////////////////
void utils::DCEL::DCEL::removeEdge(std::pair<int, int> vs) {
  HalfEdge* he = halfEdges.find(vs)->second.get();
  HalfEdge* heTwin = he->getTwin();
  // Update incident half-edge of origin and destination vertices.
  he->getOrigin()->setIncidentEdge(heTwin->getNext());
  heTwin->getOrigin()->setIncidentEdge(he->getNext());

  // Update the left face's outer edge.
  Face* leftFace = he->getIncidentFace();
  leftFace->setOuterEdge(he->getNext());

  // Update each edge surrounding the right face and make them point to the
  // left face.
  Face* rightFace = heTwin->getIncidentFace();
  utils::DCEL::HalfEdge* startHE = rightFace->getOuterEdge();
  utils::DCEL::HalfEdge* curHE = startHE;
  do {
    curHE->setIncidentFace(leftFace);
    // Get next edge of the face.
    curHE = curHE->getNext();
  } while (curHE != startHE);

  // Delete the right face.
    std::cout << (faces.find(std::make_unique<Face>(*rightFace))==faces.end()) << "\n";
  faces.erase(faces.find(std::make_unique<Face>(*rightFace)));

  // Update edges that are pointing to the soon to be deleted edges.
  he->getPrev()->setNext(heTwin->getNext());
  he->getNext()->setPrev(heTwin->getPrev());

  heTwin->getPrev()->setNext(he->getNext());
  heTwin->getNext()->setPrev(he->getPrev());

  // Delete both half edges from the DCEL.
  halfEdges.erase(vs);
  halfEdges.erase({vs.second, vs.first});
}

////////////////////////////////////////////////////////////
utils::DCEL::Face* utils::DCEL::DCEL::addFace() {
  // Create the face and add it to the map.
  auto newF = std::make_unique<Face>();
  Face* newFP = newF.get();
  faces.insert(std::move(newF));

  return newFP;
}

////////////////////////////////////////////////////////////
const std::unordered_map<int, std::unique_ptr<utils::DCEL::Vertex>>&
utils::DCEL::DCEL::getVertices() const {
  return vertices;
}

////////////////////////////////////////////////////////////
const std::unordered_map<std::pair<int, int>,
                         std::unique_ptr<utils::DCEL::HalfEdge>,
                         utils::DCEL::PairHash>&
utils::DCEL::DCEL::getHalfEdges() const {
  return halfEdges;
}

////////////////////////////////////////////////////////////
const std::unordered_set<std::unique_ptr<utils::DCEL::Face>,
                         utils::DCEL::uPtrHash, utils::DCEL::uPtrEq>&
utils::DCEL::DCEL::getFaces() const {
  return faces;
}

////////////////////////////////////////////////////////////
utils::DCEL::DCEL
utils::DCEL::DCEL::makeDCEL(const std::vector<std::vector<int>>& faces,
                            int nbVertices) {
  DCEL d;
  for (auto& face : faces) {
    Face* f = d.addFace();
    HalfEdge* prevHE = nullptr;
    for (int vCur = 0, vPrev = face.size() - 1; vCur < face.size();
         vPrev = vCur++) {
      d.addVertex(face[vPrev]);
      HalfEdge* he = d.addHalfEdge({face[vPrev], face[vCur]});
      he->setIncidentFace(f);
      f->setOuterEdge(he);
      // Link prev edge to current one.
      if (prevHE != nullptr) {
        prevHE->setNext(he);
        he->setPrev(prevHE);
      }
      prevHE = he;
    }
    // Link last helf-edge to first half-edge.
    HalfEdge* firstHE =
        d.getHalfEdges().find({face[face.size() - 1], face[0]})->second.get();
    firstHE->setPrev(prevHE);
    prevHE->setNext(firstHE);
  }
  // Create outer half-edges.
  HalfEdge* prevHE = nullptr;
  for (int vCur = 0, vPrev = nbVertices - 1; vCur < nbVertices;
       vPrev = vCur++) {
    HalfEdge* he = d.addHalfEdge({vCur, vPrev});

    // Cannot be removed if it is an outer edge.
    he->setCandidate(false);

    // Link prev edge to current one. CCW order on the outside.
    if (prevHE != nullptr) {
      prevHE->setPrev(he);
      he->setNext(prevHE);
    }
    prevHE = he;
  }
  // Link last helf-edge to first half-edge.
  HalfEdge* firstHE = d.getHalfEdges().find({0, nbVertices - 1})->second.get();
  firstHE->setNext(prevHE);
  prevHE->setPrev(firstHE);

  return d;
}
