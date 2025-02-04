#include "ui/frame/SFMLRenderer.hpp"
#include "SFML/Graphics/CircleShape.hpp"
#include "SFML/Graphics/Color.hpp"
#include "SFML/Graphics/ConvexShape.hpp"
#include "SFML/Graphics/Drawable.hpp"
#include "SFML/Graphics/PrimitiveType.hpp"
#include "SFML/Graphics/VertexArray.hpp"
#include "env/Environment.hpp"
#include "physics/collision/Collision.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::SFMLRenderer()
    : g2d([](const sf::Drawable& s) {}), env(nullptr) {}

////////////////////////////////////////////////////////////
ui::frame::SFMLRenderer::~SFMLRenderer() {}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setG2d(
    std::function<void(const sf::Drawable&)> g2d) {
  this->g2d = g2d;
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::drawEnv() {
  // Make sure there is an environment
  if (!env) {
    return;
  }
  std::vector<env::bodies::Polygon>& polygons = env->getPolygons();
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
  env->unlockPolygons();
  // Draw contact points of collision manifold.
  if (env->getCollisionsP()) {
    const std::vector<physics::collision::Collision>& cols =
        *env->getCollisionsP();
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
env::Environment* ui::frame::SFMLRenderer::getEnv() { return env; }

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setEnv(env::Environment* env) { this->env = env; }

////////////////////////////////////////////////////////////
void ui::frame::SFMLRenderer::setWindowHeight(int windowHeight) {
  this->windowHeight = windowHeight;
}
