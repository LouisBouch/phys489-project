#include "ui/frame/SFMLWindow.hpp"
#include "SFML/Graphics/Drawable.hpp"
#include "SFML/System/Vector2.hpp"
#include "SFML/Window/Keyboard.hpp"
#include "SFML/Window/Mouse.hpp"
#include "env/Environment.hpp"
#include "env/bodies/Polygon.hpp"
#include "physics/forces/Force.hpp"
#include "ui/frame/SFMLRenderer.hpp"
#include "utils/geo/geoUtils.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window/Event.hpp>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <thread>

#define FPS 60

////////////////////////////////////////////////////////////
ui::frame::SFMLWindow::SFMLWindow(physics::PhysicsEngine& engine)
    : running(false), paused(false), engine(engine), renderer(engine),
      dragging(false), dragPId(-1) {}

////////////////////////////////////////////////////////////
ui::frame::SFMLWindow::~SFMLWindow() {
  window.close();
  stopWindowLoop();
  joinThread();
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::create(const std::string& title) {
  // Get default fullscreen mode
  sf::VideoMode mode = sf::VideoMode::getFullscreenModes()[1];

  // OpenGL settings
  sf::ContextSettings settings;
  settings.antiAliasingLevel = 8;

  // Create a window with the same pixel depth as the desktop
  sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
  window.create(sf::VideoMode({mode.size.x - 100, mode.size.y - 150},
                              desktop.bitsPerPixel),
                title, sf::Style::Default, sf::State::Windowed, settings);
  window.setFramerateLimit(FPS);

  // Setup renderer
  renderer.setG2d([this](const sf::Drawable& d) { window.draw(d); });
  renderer.setWindowHeight(window.getSize().y);
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::startWindowLoop() {
  if (running) {
    return;
  }
  running = true;
  if (window.setActive(false)) {
    windowThread = std::thread(&ui::frame::SFMLWindow::windowLoop, this);
  } else {
    std::cout << "Could not activate window\n";
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::stopWindowLoop() { running = false; }

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::windowLoop() {
  // Activate the window
  if (!window.setActive(true)) {
    std::cout << "Could not activate window\n";
  }
  sf::Clock clock;
  while (running) {
    // handle events
    handleEvents();

    // draw...
    window.clear();
    if (dragging) {
      drawDragLine();
      updateDragForce();
    }

    int deltaTime = clock.restart().asSeconds() *
                    1e6; // Time since last render in microseconds.
    renderer.drawEnv();
    renderer.showTime();
    if (!paused && engine.getEnv()) {
      engine.getEnv()->addToTimeBuffer(deltaTime);
    }

    window.display();
  }
  // Deactivate window once the rendering loop ends.
  if (!window.setActive(false)) {
    std::cout << "Could not deactivate window\n";
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::joinThread() {
  if (windowThread.joinable()) {
    windowThread.join();
  }
}

////////////////////////////////////////////////////////////

ui::frame::SFMLRenderer& ui::frame::SFMLWindow::getRenderer() {
  return renderer;
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::handleEvents() {
  while (const std::optional event = window.pollEvent()) {
    if (event->is<sf::Event::Closed>()) {
      // end the program
      running = false;
    } else if (const auto* resized = event->getIf<sf::Event::Resized>()) {
      // Adjust the viewport when the window is resized.
      // Update the view to the new size of the window.
      sf::FloatRect visibleArea({0.f, 0.f}, sf::Vector2f(resized->size));
      window.setView(sf::View(visibleArea));
      // Update renderer.
      renderer.setWindowHeight(window.getSize().y);
    } else if (const auto* mousePressed =
                   event->getIf<sf::Event::MouseButtonPressed>()) {
      int x = mousePressed->position.x;
      int y =
          -mousePressed->position.y +
          window.getSize().y; // Shifts y axis to bottom and make it point up.
      std::cout << "x: " << x << ", y:" << y << "\n";
      // Create dragging force if you are dragging a polygon.
      if (int id = clickedInsidePolygon({x, y}); id != -1) {
        createForce(id, {x, y});
      }
    } else if (const auto* mouseRelease =
                   event->getIf<sf::Event::MouseButtonReleased>()) {
      // Remove force once you stop dragging.
      removeForce();
    } else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
      // Pause the sim on space.
      if (keyPressed->scancode == sf::Keyboard::Scancode::Space) {
        paused = !paused;
      }
      // Forward 5 steps.
      else if (keyPressed->scancode == sf::Keyboard::Scancode::Right &&
               keyPressed->control) {
        engine.getEnv()->addToTimeBuffer(engine.getDt() * 1e6 * 5);
      }
      // Forward one step.
      else if (keyPressed->scancode == sf::Keyboard::Scancode::Right) {
        engine.getEnv()->addToTimeBuffer(engine.getDt() * 1e6);
      }
    }
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::setPaused(bool paused) { this->paused = paused; }

////////////////////////////////////////////////////////////
int ui::frame::SFMLWindow::clickedInsidePolygon(sf::Vector2i cursor) {
  std::vector<env::bodies::Polygon>& polys = engine.getEnv()->getPolygons();
  for (env::bodies::Polygon& p : polys) {
    if (utils::geo::pointInPolygon(p.getGlobalVertices(),
                                   {cursor.x, cursor.y})) {
      int id = p.getId();
      engine.getEnv()->unlockPolygons();
      return id;
    }
  }
  engine.getEnv()->unlockPolygons();
  return -1;
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::createForce(int pId, sf::Vector2i cursor) {
  // Last force was not properly deleted, so delete it now.
  if (dragging) {
    removeForce();
  }
  dragging = true;
  dragPId = pId;
  auto pp = engine.getEnv()->getPolyById(dragPId);
  // Invalid id
  if (!pp.has_value()) {
    return;
  }
  env::bodies::Polygon& p = *pp.value();

  // Pulling force.
  p.addForce(physics::forces::ForceSource::UserPull,
             Eigen::Vector2d{cursor.x, cursor.y} - p.getCentroid(), {1, 0},
             0);
  // Force of drag.
  p.addForce(physics::forces::ForceSource::UserInducedDrag, {0, 0}, {1, 0}, 0);
  // Unlock polygons
  engine.getEnv()->unlockPolygons();
  return;
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::removeForce() {
  if (!dragging) {
    return;
  }
  dragging = false;
  auto pp = engine.getEnv()->getPolyById(dragPId);
  // Invalid id
  if (!pp.has_value()) {
    return;
  }
  env::bodies::Polygon& p = *pp.value();
  p.removeForce(physics::forces::ForceSource::UserPull);
  p.removeForce(physics::forces::ForceSource::UserInducedDrag);
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::drawDragLine() {
  // Get force
  auto pp = engine.getEnv()->getPolyById(dragPId);
  // Invalid id
  if (!pp.has_value()) {
    return;
  }
  env::bodies::Polygon& p = *pp.value();
  physics::forces::ForceSource s = physics::forces::ForceSource::UserPull;
  physics::forces::Force& polyForcePoint = p.getForceBySource(s);

  // Get point on screen where force is applied
  Eigen::Vector2d a = p.getCentroid() + polyForcePoint.getForcePos();
  // Get mouse position
  sf::Vector2i b = sf::Mouse::getPosition(window);
  // Draw line from force point to mouse.
  renderer.drawLine({(float)a.x(), (float)a.y()},
                    {(float)b.x, (float)window.getSize().y - b.y}, 2);

  engine.getEnv()->unlockPolygons();
}
////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::updateDragForce() {
  // Get force
  auto pp = engine.getEnv()->getPolyById(dragPId);
  // Invalid id
  if (!pp.has_value()) {
    return;
  }

  env::bodies::Polygon& p = *pp.value();
  physics::forces::Force& polyForcePoint =
      p.getForceBySource(physics::forces::ForceSource::UserPull);
  physics::forces::Force& polyForceDrag =
      p.getForceBySource(physics::forces::ForceSource::UserInducedDrag);

  // Get point on screen where force is applied
  Eigen::Vector2d a = p.getCentroid() + polyForcePoint.getForcePos();
  // Get mouse position
  sf::Vector2i b = sf::Mouse::getPosition(window);
  b.y = window.getSize().y - b.y; // Adjust to physical coordinate system.
  Eigen::Vector2d dir = {b.x - a.x(), b.y - a.y()};
  double k = 1e2 * p.getArea(); // Spring coefficient
  double pullAmp = k * dir.norm();
  double d =
      2 * std::sqrt(p.getArea() * k); // Damping factor (critical damping)
  double dragAmp = d * p.getVelocity().norm();
  ;
  // Updates pulling force.
  polyForcePoint.setForceD(dir);
  polyForcePoint.setAmplitude(pullAmp);

  // Updates force of drag.
  polyForceDrag.setForceD(-p.getVelocity());
  polyForceDrag.setAmplitude(dragAmp);

  engine.getEnv()->unlockPolygons();
}
