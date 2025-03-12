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
#include <sstream>
#include <string>
#include <vector>

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::SFMLRenderer(physics::PhysicsEngine& engine)
    : g2d([](const sf::Drawable& s) {}), engine(engine), font(loadFont()) {}

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
  const std::vector<env::bodies::Polygon>& polygons = env.getPolygons();
  // Iterate over polygons convex decomposition and draw each sub polygon.
  for (const env::bodies::Polygon& p : polygons) {
    // Set color depending on collision status.
    sf::Color col =
        p.isColliding() ? sf::Color(255, 0, 0, 150) : sf::Color(0, 255, 0, 150);
    const std::vector<std::vector<int>>& conPolys = p.getConvexDecomp();
    const Eigen::Matrix2Xd& gv = p.getGlobalVertices();
    for (int convP = 0; convP < conPolys.size(); convP++) {
      Eigen::Matrix2Xd convPVertices(2, conPolys[convP].size());
      for (int i = 0; i < conPolys[convP].size(); i++) {
        // Index of the vertex.
        int vIndex = conPolys[convP][i];
        // Position in space of the vertex.
        convPVertices.col(i) = gv.col(vIndex);
      }
      // Draw convex polygon.
      fillShape(convPVertices, col);
    }
    int id = p.getId();
    sf::Text idText(font);
    idText.setString(std::to_string(id));
    idText.setFillColor(sf::Color::White);
    idText.setStyle(sf::Text::Bold);
    idText.setCharacterSize(20);
    idText.setOrigin(
        {idText.getLocalBounds().size.x / 2, idText.getLocalBounds().size.y});
    idText.setPosition({(float)p.getCentroid().x(),
                        windowHeight - (float)p.getCentroid().y()});
    g2d(idText);
  }
  // Draw contact points of collision manifold.
  const std::unordered_map<std::pair<int, int>,
                           std::unique_ptr<physics::collision::Collision>,
                           physics::collision::PairHash>& cols =
      engine.getColDetector().getCollisions();
  float rad = 7.5;
  sf::CircleShape circle(rad);
  circle.setOrigin({rad, rad});
  // Point representing contact manifold.
  circle.setOutlineColor(sf::Color::Blue);
  circle.setFillColor(sf::Color::Transparent);
  circle.setOutlineThickness(2);
  // Text for force
  sf::Text normalForceText(font);
  normalForceText.setFillColor(sf::Color::Green);
  normalForceText.setStyle(sf::Text::Bold);
  int fontSize = 16;
  normalForceText.setCharacterSize(fontSize);
  for (auto& colPair : cols) {
    auto& col = *colPair.second.get();
    // for (Eigen::Vector2d c : col.getManifold()) {
    for (int ci = 0; ci < col.getManifold().size(); ci++) {
      Eigen::Vector2d c = col.getManifold()[ci];
      // Draw circle around it.
      circle.setPosition({(float)c[0], (float)-c[1] + windowHeight});
      g2d(circle);
      // Draw normal of collision
      int length = 10;
      Eigen::Vector2d e1 = c + col.getNormal() * length / 2;
      Eigen::Vector2d e2 = c - col.getNormal() * length / 2;
      drawLine({(float)e1[0], (float)e1[1]}, {(float)e2[0], (float)e2[1]}, 2);
      // Draw normal force.
      normalForceText.setPosition(
          {(float)c.x() + fontSize / 2.f, windowHeight - (float)c.y()});
      std::ostringstream oss;
      oss << std::setprecision(2) << std::scientific
          << std::abs(col.getAccNormalImpulse()[ci]);
      normalForceText.setString(oss.str());
      g2d(normalForceText);
      // Draw friction force.
      normalForceText.setPosition({(float)c.x() + fontSize / 2.f,
                                   windowHeight - (float)c.y() - fontSize});
      oss.str("");
      oss.clear();
      oss << std::setprecision(2) << std::scientific
          << std::abs(col.getAccTangentImpulse()[ci]);
      normalForceText.setString(oss.str());
      g2d(normalForceText);
    }
  }
  env.unlockPolygons();
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
  double timeSec = engine.getEnv()->getTotalTime() / 1e6;
  sf::Text timeText(font);
  timeText.setString(std::to_string(timeSec));
  timeText.setFillColor(sf::Color::Green);
  timeText.setStyle(sf::Text::Bold);
  timeText.setCharacterSize(24);
  g2d(timeText);
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::drawLine(const sf::Vector2f& a,
                                       const sf::Vector2f& b, int thickness) {
  // Direction vector from a to b.
  sf::Vector2f dir = b - a;
  // Angle offset when on the left of screen.
  double phi = dir.x < 0 ? M_PI : 0;
  double angle = std::atan(dir.y / dir.x) + phi;
  sf::RectangleShape line;
  line.setFillColor(sf::Color::Yellow);
  line.setSize({dir.length(), (float)thickness});
  line.setOrigin({0, (float)thickness / 2.0f});
  // Requires some change of coordinate to set origin at bottom left of screen.
  line.setPosition({a.x, (float)windowHeight -
                             a.y}); // Flip axis and translate down a screen.
  line.setRotation(
      sf::radians(-angle)); // Negative rotation to simulate a vertical flip.
  g2d(line);
}
////////////////////////////////////////////////////////////
sf::Font ui::frame::SFMLRenderer::loadFont() {
  sf::Font font;
  if (!font.openFromFile("data/TTF/InconsolataGoNerdFont-Bold.ttf")) {
    std::cout << "Couldn't load font\n";
  }
  return font;
}
