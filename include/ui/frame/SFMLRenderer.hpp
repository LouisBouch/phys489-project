#pragma once
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/Drawable.hpp"
#include "SFML/Graphics/Font.hpp"
#include "SFML/System/Vector2.hpp"
#include "physics/PhysicsEngine.hpp"

namespace ui::frame {
class SFMLRenderer {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param engine Engine that will simulate the environment.
   */
  SFMLRenderer(physics::PhysicsEngine& engine);
  /**
   * @brief The deconstructor.
   */
  ~SFMLRenderer();

  /**
   * @brief Draw the environment.
   */
  void drawEnv();

  /**
   * @brief Sets the graphical context.
   *
   * @param The Graphical context.
   */
  void setG2d(std::function<void(const sf::Drawable&)> g2d);

  /**
   * @brief Sets the height of the window.
   *
   * @param The current height of the window. (In pixels)
   */
  void setWindowHeight(int windowHeight);

  /**
   * @brief Shows time since begginnig of sim (Environment time in seconds).
   */
  void showTime();

  /**
   * @brief Draws a line between two points.
   *
   * @param a First point.
   * @param b Second point.
   * @param thickness Thickness of the line.
   *
   */
  void drawLine(sf::Vector2f a, sf::Vector2f b, int thickness);

private:
  int windowHeight;      //< Height of window.
  sf::Font font;      //< Font to use to draw stuff.
  physics::PhysicsEngine& engine; //< Physics engine that simulates the environment.
  std::function<void(const sf::Drawable&)>
      g2d; //< Lambda function to draw on screen. Called on each shape in the
           // environment.

  /**
   * @brief Draws shape on screen with filled interior. Uses triangulation to ensure proper filling of concave polygons.
   */
  void fillShape(const Eigen::Matrix2Xd& vertices, sf::Color& color);

  /**
   * @brief Loads a font.
   */
  sf::Font loadFont();
};
} // namespace ui::frame
