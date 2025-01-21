#pragma once
#include <vector>

namespace utils::math {
class Vector2 {
public:
  /**
   * @brief Default constructor.
   * Initializes vector to (0, 0).
   */
  Vector2();
  /**
   * @brief Parameterized constructor.
   *
   * @param x The x-coordinate of the vector.
   * @param y The y-coordinate of the vector.
   */
  Vector2(double x, double y);
  /**
   * @brief Components constructor.
   *
   * @param c The components used to construct the vector.
   */
  Vector2(std::vector<double>& c);
  /**
   * @brief Copy constructor.
   *
   * @param v The vector to copy.
   */
  Vector2(const Vector2& v);
  /**
   * @brief The deconstructor.
   */
  ~Vector2();

  /**
   * @brief Getter for x-coordinate.
   *
   * @return x-coordinate of the vector.
   */
  double getX() const;
  /**
   * @brief Getter for y-coordinate.
   *
   * @return y-coordinate of the vector.
   */
  double getY() const;
  /**
   * @brief Getter for vector components.
   *
   * @return Vector components as std::vector.
   */
  const std::vector<double>& getComponents() const;

  /**
   * @brief Setter for x-coordinate.
   *
   * @param New x-coordinate of the vector.
   */
  void setX(double x);
  /**
   * @brief Setter for y-coordinate.
   *
   * @param New y-coordinate of the vector.
   */
  void setY(double y);
  /**
   * @brief Setter for vector components.
   *
   * @param New vector components as std::vector.
   */
  void setComponents(std::vector<double>& c);

private:
  std::vector<double> components; //< Vector components.
};
} // namespace utils::math
