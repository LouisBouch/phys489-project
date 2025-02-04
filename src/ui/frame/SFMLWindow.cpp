#include "ui/frame/SFMLWindow.hpp"
#include "SFML/Graphics/Drawable.hpp"
#include "SFML/Window/Keyboard.hpp"
#include "ui/frame/SFMLRenderer.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window/Event.hpp>
#include <iostream>
#include <thread>

#define FPS 60

////////////////////////////////////////////////////////////
ui::frame::SFMLWindow::SFMLWindow() : running(false), paused(false) {}

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

    int deltaTime = clock.restart().asSeconds() *
                    1e6; // Time since last render in microseconds.
    renderer.drawEnv();
    if (!paused && renderer.getEnv()) {
      renderer.getEnv()->addToTimeBuffer(deltaTime);
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
      std::cout << "x: " << mousePressed->position.x
                << ", y:" << -mousePressed->position.y + window.getSize().y
                << "\n";
    } else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
      // Pause the sim on space.
      if (keyPressed->scancode == sf::Keyboard::Scancode::Space) {
        paused = !paused;
      }
    }
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::setPaused(bool paused) { this->paused = paused; }
