#pragma once
#include "SFML/Graphics/Shape.hpp"
#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"

namespace ui::frame {
class SFMLRenderer {
public:
  /**
   * @brief Default constructor.
   */
  SFMLRenderer();
  /**
   * @brief The deconstructor.
   */
  ~SFMLRenderer();

  /**
   * @brief Getter for environment.
   *
   * @return The environment of the simulation.
   */
  env::Environment& getEnv();

  /**
   * @brief Setter for environment.
   *
   * @param env The environment of the simulation.
   */
  void setEnv(env::Environment& env);

  /**
   * @brief Draw the environment.
   */
  void drawEnv();


  /**
   * @brief Sets the graphical context.
   *
   * @param
   */
  void setG2d(std::function<void(const sf::Shape&)> g2d);
private:
  env::Environment env; //< Simulation content.
  std::function<void(const sf::Shape&)>
      g2d; //< Lambda function to draw on screen. Called on each shape in the environment.

  /**
   * @brief Draw shape.
   */
  void drawShape(const env::bodies::Polygon& p);
};
} // namespace ui::frame
