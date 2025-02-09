#include "physics/forces/Force.hpp"
#include <iostream>
#include <limits>

////////////////////////////////////////////////////////////
physics::forces::Force::Force(Eigen::Vector2d forceD, double amplitude,
                              double& rot, Eigen::Vector2d forcePos,
                              ForceSource source)
    : forceD(forceD.normalized()), amplitude(amplitude), rot(&rot),
      lastRot(std::numeric_limits<double>::max()), forcePos(forcePos),
      forcePosLocal(findForcePosLocal()), source(source) {
}

////////////////////////////////////////////////////////////
const Eigen::Vector2d& physics::forces::Force::getForceD() const {
  return forceD;
}

////////////////////////////////////////////////////////////
double physics::forces::Force::getAmplitude() const { return amplitude; }

////////////////////////////////////////////////////////////
const Eigen::Vector2d& physics::forces::Force::getForcePos() const {
  double r = *this->rot;
  if (lastRot == r) {
    return forcePos;
  }
  // Update last value of rotation.
  lastRot = r;
  // Create rotation matrix
  double cosr = std::cos(r);
  double sinr = std::sin(r);
  Eigen::Matrix2d rM{{cosr, -sinr}, {sinr, cosr}};

  // Find current position given rotation.
  forcePos.noalias() = rM * forcePosLocal;

  return forcePos;
}

////////////////////////////////////////////////////////////
void physics::forces::Force::setForceD(const Eigen::Vector2d& forceD) {
  this->forceD = forceD.normalized();
}

////////////////////////////////////////////////////////////
void physics::forces::Force::setAmplitude(double amplitude) {
  this->amplitude = amplitude;
}

////////////////////////////////////////////////////////////
void physics::forces::Force::setForcePos(const Eigen::Vector2d& forcePos) {
  this->forcePos = forcePos;
  forcePosLocal = findForcePosLocal();
}

////////////////////////////////////////////////////////////
physics::forces::ForceSource physics::forces::Force::getForceSource() const {
  return source;
}

//////////////////////////////PRIVATE METHODS//////////////////////////////
Eigen::Vector2d physics::forces::Force::findForcePosLocal() {
  double r = *this->rot;
  // Create rotation matrix that rotates back to initial position.
  double cosr = std::cos(-r);
  double sinr = std::sin(-r);
  Eigen::Matrix2d rM{{cosr, -sinr}, {sinr, cosr}};

  // Return unrotated posittion of the force.
  return rM * forcePos;
}

////////////////////////////////////////////////////////////
void physics::forces::Force::setRot(double* rot) { this->rot = rot; }
