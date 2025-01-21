#include "ui/frame/IWindow.hpp"
#include "ui/frame/SFMLRenderer.hpp"

// Constructor
ui::frame::IWindow::IWindow() : running(false), bufferTime(0) {}

// Getters/Setters
void ui::frame::IWindow::setRunning(bool running) { this->running = running; }
bool ui::frame::IWindow::isRunning() const { return running; }

ui::frame::SFMLRenderer& ui::frame::IWindow::getRenderer() {
  return renderer;
}
