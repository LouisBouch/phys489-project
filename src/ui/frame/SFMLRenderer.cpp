#include "ui/frame/SFMLRenderer.hpp"
#include "SFML/Graphics/ConvexShape.hpp"
#include "env/Environment.hpp"

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::SFMLRenderer()
    : g2d([](const sf::Shape& s) {}), env(nullptr) {}

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::~SFMLRenderer() {}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setG2d(
    std::function<void(const sf::Shape&)> g2d) {
  this->g2d = g2d;
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::drawEnv() {
  // Make sure there is an environment
  if (!env) {
    return;
  }
  std::vector<env::bodies::Polygon>& polygons = env->getPolygons();
  // Iterate over polygons and draw them
  for (env::bodies::Polygon& p : polygons) {
    drawShape(p);
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::drawShape(const env::bodies::Polygon& p) {
  sf::ConvexShape s;
  int nbVertices = p.getNbVertices();
  s.setPointCount(nbVertices);

  // Place points on polygon
  const Eigen::Matrix2Xd& globalVertices = p.getGlobalVertices();
  for (int v = 0; v < nbVertices; v++) {
    s.setPoint(v, {(float)globalVertices.col(v)[0],
                   (float)-globalVertices.col(v)[1] + windowHeight});
  }
  s.setFillColor(sf::Color(255, 255, 0));

  // Draw shape
  g2d(s);
}

////////////////////////////////////////////////////////////
env::Environment* ui::frame::SFMLRenderer::getEnv() { return env; }

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setEnv(env::Environment* env) { this->env = env; }

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setWindowHeight(int windowHeight) {
  this->windowHeight = windowHeight;
}
