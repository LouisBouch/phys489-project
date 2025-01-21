#pragma once
#include "IWindow.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include <thread>

namespace ui::frame {

class SFMLWindow : public IWindow {
public:
  /**
   * @brief Default constructor.
   */
  SFMLWindow();
  /**
   * @brief The deconstructor.
   */
  ~SFMLWindow() override;

  /**
   * @brief Initializes the window.
   *
   * @param title Name of the window.
   */
  void create(const std::string& title) override;
  /**
   * @brief Starts the loop that renders the screen.
   */
  void startRenderLoop() override;
  /**
   * @brief Stops the loop that renders the screen.
   */
  void stopRenderLoop() override;
  /**
   * @brief Joins the calling thread to the thread that runs the render loop.
   */
  void joinThread() override;
  /**
   * @brief Gets the renderer.
   *
   * @return Renderer.
   */

private:
  sf::RenderWindow window;  //< The window frame.
  std::thread renderThread; //< Thread rendering the screen.
  /**
   * @brief Code that is run when rendering the window.
   */
  void renderLoop() override;
};

} // namespace ui::frame
