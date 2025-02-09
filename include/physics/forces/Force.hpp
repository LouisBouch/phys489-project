#pragma once

#include "eigen3/Eigen/Dense"

namespace physics::forces {
enum class ForceSource {
  UserPull,        //< Created by the user dragging the polygon.
  UserInducedDrag, //< Force used to slow down the polygon.
};
class Force {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param forceD Direction of the force.
   * @param amplitude Amplitude of the force.
   * @param rot Reference to the rotation of the polygon which feels the force.
   * @param forcePos Position of the force relative to the centroid of the
   * polygon.
   * @param source Source of the force.
   */
  Force(Eigen::Vector2d forceD, double amplitude, double& rot,
        Eigen::Vector2d forcePos, ForceSource source);

  /**
   * @brief Gets the normalized force direction.
   *
   * @return Force direction.
   */
  const Eigen::Vector2d& getForceD() const;
  /**
   * @brief Gets the force amplitude.
   *
   * @return Force amplitude.
   */
  double getAmplitude() const;
  /**
   * @brief Gets the force position.
   *
   * @return Force position.
   */
  const Eigen::Vector2d& getForcePos() const;

  /**
   * @brief Sets the force direction.
   *
   * @param forceD Force direction.
   */
  void setForceD(const Eigen::Vector2d& forceD);
  /**
   * @brief Sets the force amplitude.
   *
   * @param amplitude Force amplitude.
   */
  void setAmplitude(double amplitude);
  /**
   * @brief Sets the force position.
   *
   * @param forcePos Force position.
   */
  void setForcePos(const Eigen::Vector2d& forcePos);
  /**
   * @brief Gets the force source.
   *
   * @return Force source.
   */
  ForceSource getForceSource() const;

  /**
   * @brief Sets the rotation pointer. (Used by copy constructor of polygon)
   *
   * @param rot New rotation pointer.
   */
  void setRot(double* rot);

private:
  ForceSource source;     //< Source of the force.
  Eigen::Vector2d forceD; //< Normalized vector of the force.
  double amplitude;       //< Amplitude of the force.
  double* rot; //< Rotation angle of the polygon the force is attached to.
  mutable double lastRot; //< Rotation when force position was last obtained.
  mutable Eigen::Vector2d
      forcePos; //< Where the force is being applied relative to the
                // centroid after rotation of the polygon is applied.
  Eigen::Vector2d
      forcePosLocal; //< Where the force is being applied relative to the
                     // centroid before rotation of the polygon is applied. Thus
                     // to find where the force is being applid on the current
                     // polygon, you must rotate it with the global vertices.
  /**
   * @brief Given force position and polygon rotation, find the local posittion
   * of the position of the force.
   *
   * @return Local position of the force.
   */
  Eigen::Vector2d findForcePosLocal();
};
} // namespace physics::forces
