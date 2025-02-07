#include "ui/frame/SFMLRenderer.hpp"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/ConvexShape.hpp"
#include "SFML/Graphics/Drawable.hpp"
#include "SFML/Graphics/Font.hpp"
#include "SFML/Graphics/RectangleShape.hpp"
#include "SFML/Graphics/Text.hpp"
#include "SFML/System/Angle.hpp"
#include "SFML/System/Vector2.hpp"
#include "env/Environment.hpp"
#include "physics/collision/Collision.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::SFMLRenderer(physics::PhysicsEngine& engine)
    : g2d([](const sf::Drawable& s) {}), engine(engine) {}

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::~SFMLRenderer() {}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setG2d(
    std::function<void(const sf::Drawable&)> g2d) {
  this->g2d = g2d;
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::drawEnv() {
  env::Environment& env = *engine.getEnv();
  std::vector<env::bodies::Polygon>& polygons = env.getPolygons();
  // Iterate over polygons triangulation and draw them
  Eigen::Matrix2Xd triVertices(2, 3);
  for (env::bodies::Polygon& p : polygons) {
    // Set color depending on collision status.
    sf::Color col =
        p.isColliding() ? sf::Color(255, 0, 0, 150) : sf::Color(0, 255, 0, 150);
    const Eigen::Matrix3Xi& tris = p.getTriangulation();
    const Eigen::Matrix2Xd& gv = p.getGlobalVertices();
    for (int tri = 0; tri < p.getNbVertices() - 2; tri++) {
      // Indices of each vertex of the triangle.
      int a = tris.col(tri)[0];
      int b = tris.col(tri)[1];
      int c = tris.col(tri)[2];
      // Get real world vertices
      triVertices.col(0) = gv.col(a);
      triVertices.col(1) = gv.col(b);
      triVertices.col(2) = gv.col(c);
      // Draw triangle
      fillShape(triVertices, col);
    }
  }
  env.unlockPolygons();
  // Draw contact points of collision manifold.
  const std::vector<physics::collision::Collision>& cols =
      engine.getColDetector().getCollisions();
  sf::CircleShape point(2.f);
  // Point representing contact manifold.
  point.setFillColor(sf::Color::Blue);
  point.setOrigin({2.f, 2.f});
  for (auto col : cols) {
    for (Eigen::Vector2d c : col.getManifold()) {
      point.setPosition({(float)c[0], (float)-c[1] + windowHeight});
      g2d(point);
    }
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::fillShape(const Eigen::Matrix2Xd& vertices,
                                        sf::Color& color) {
  sf::ConvexShape s;

  // Place points on polygon
  int nbVertices = vertices.cols();
  s.setPointCount(nbVertices);
  for (int v = 0; v < nbVertices; v++) {
    s.setPoint(v, {(float)vertices.col(v)[0],
                   (float)-vertices.col(v)[1] + windowHeight});
  }
  s.setFillColor(color);
  s.setOutlineColor(sf::Color(255, 255, 255));
  s.setOutlineThickness(-1);

  // Draw shape
  g2d(s);
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setWindowHeight(int windowHeight) {
  this->windowHeight = windowHeight;
}
////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::showTime() {
  sf::Font font;
  if (!font.openFromFile("data/TTF/InconsolataGoNerdFont-Bold.ttf")) {
    std::cout << "Couldn't load font\n";
    return;
  }
  double timeSec = engine.getEnv()->getTotalTime() / 1e6;
  sf::Text timeText(font);
  timeText.setString(std::to_string(timeSec));
  timeText.setFillColor(sf::Color::Green);
  timeText.setStyle(sf::Text::Bold);
  timeText.setCharacterSize(24);
  g2d(timeText);
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::drawLine(sf::Vector2f a, sf::Vector2f b,
                                       int thickness) {
  // Direction vector from a to b.
  sf::Vector2f dir = b - a;
  // Angle offset when on the left of screen.
  double phi = dir.x < 0 ? M_PI : 0;
  double angle = std::atan(dir.y / dir.x) + phi;
  sf::RectangleShape line;
  line.setFillColor(sf::Color::White);
  line.setSize({dir.length(), (float)thickness});
  line.setOrigin({0, (float)thickness / 2.0f});
  // Requires some change of coordinate to set origin at bottom left of screen.
  line.setPosition({a.x, (float)windowHeight -
                             a.y}); // Flip axis and translate down a screen.
  line.setRotation(
      sf::radians(-angle)); // Negative rotation to simulate a vertical flip.
  g2d(line);
}
