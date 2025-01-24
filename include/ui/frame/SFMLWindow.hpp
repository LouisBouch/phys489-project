#pragma once
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFMLRenderer.hpp"
#include <thread>

namespace ui::frame {

class SFMLWindow {
public:
  /**
   * @brief Default constructor.
   */
  SFMLWindow();
  /**
   * @brief The deconstructor.
   */
  ~SFMLWindow();

  /**
   * @brief Initializes the window.
   *
   * @param title Name of the window.
   */
  void create(const std::string& title);

  /**
   * @brief Starts the loop that renders the screen.
   */
  void startWindowLoop();

  /**
   * @brief Sets the pause value of the render loop.
   */
  void setPaused(bool paused);

  /**
   * @brief Stops the loop that renders the screen.
   */
  void stopWindowLoop();

  /**
   * @brief Joins the calling thread to the thread that runs the render loop.
   */
  void joinThread();

  /**
   * @brief Gets the renderer.
   *
   * @return Renderer.
   */
  SFMLRenderer& getRenderer();

private:
  bool running;             //< Whether the rendering is running or not.
  bool paused;              //< Whether the rendering is paused or not.
  SFMLRenderer renderer;    //< Renderer for the window.
  sf::RenderWindow window;  //< The window frame.
  std::thread windowThread; //< Thread rendering the screen.

  /**
   * @brief Code that is run when rendering the window.
   */
  void windowLoop();

  /**
   * @brief Hanldes the events.
   */
  void handleEvents();
};

////////////////////////////////////////////////////////////
// enum class windowMode {
//   Listen,
//   ListenAndRender,
//   Exit,
// };
} // namespace ui::frame
