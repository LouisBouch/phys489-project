#include "ui/frame/SFMLWindow.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window/Event.hpp>
#include <iostream>
#include <thread>

////////////////////////////////////////////////////////////
ui::frame::SFMLWindow::SFMLWindow() {}

////////////////////////////////////////////////////////////
ui::frame::SFMLWindow::~SFMLWindow() {
  window.close();
  stopRenderLoop();
  joinThread();
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::create(const std::string& title) {
  // Setup renderer
  getRenderer().setG2d([this](const sf::Shape& s) { window.draw(s); });

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
  window.setFramerateLimit(60);

  // Starts listening for events
  startRenderLoop();
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::startRenderLoop() {
  if (isRunning()) {
    return;
  }
  setRunning(true);
  if (window.setActive(false)) {
    renderThread = std::thread(&ui::frame::SFMLWindow::renderLoop, this);
  } else {
    std::cout << "Could not activate window\n";
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::stopRenderLoop() {
  if (!isRunning()) {
    return;
  }
  setRunning(false);
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::renderLoop() {
  // Activate the window
  if (!window.setActive(true)) {
    std::cout << "Could not activate window\n";
  }
  sf::Clock clock;
  while (isRunning()) {
    // handle events
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        // end the program
        setRunning(false);
      } else if (const auto* resized = event->getIf<sf::Event::Resized>()) {
        // adjust the viewport when the window is resized
        // update the view to the new size of the window
        sf::FloatRect visibleArea({0.f, 0.f}, sf::Vector2f(resized->size));
        window.setView(sf::View(visibleArea));
      } else if (const auto* mousePressed =
                     event->getIf<sf::Event::MouseButtonPressed>()) {
        std::cout << "x: " << mousePressed->position.x
                  << ", y:" << mousePressed->position.y << "\n";
      }
    }

    // draw...
    /*int dx = 0;*/
    /*float deltaTime = clock.restart().asSeconds();*/
    /*dx = (int)(dx + deltaTime * 500) % 1500;*/
    window.clear();
    getRenderer().drawEnv();

    // end the current frame (internally swaps the front and back buffers)
    window.display();
  }
  if (!window.setActive(false)) {
    std::cout << "Could not deactivate window\n";
  }
}

////////////////////////////////////////////////////////////
void ui::frame::SFMLWindow::joinThread() {
  if (renderThread.joinable()) {
    renderThread.join();
  }
}
