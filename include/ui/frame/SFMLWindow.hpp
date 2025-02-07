#pragma once
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFMLRenderer.hpp"
#include "physics/PhysicsEngine.hpp"
#include <thread>

namespace ui::frame {

class SFMLWindow {
public:
  /**
   * @brief Parameterized constructor.
   *
   * @param engine Engine that will simulate the environment.
   */
  SFMLWindow(physics::PhysicsEngine& engine);
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
  bool dragging;            //< Whether the user is dragging a polygon or not.
  int dragPId;              //< Id of the oplygon being dragged.
  bool paused;              //< Whether the rendering is paused or not.
  SFMLRenderer renderer;    //< Renderer for the window.
  sf::RenderWindow window;  //< The window frame.
  std::thread windowThread; //< Thread rendering the screen.
  physics::PhysicsEngine&
      engine; //< Physics engine that simulates the environment.

  /**
   * @brief Code that is run when rendering the window.
   */
  void windowLoop();

  /**
   * @brief Hanldes the events.
   */
  void handleEvents();

  /**
   * @brief Check if the click is inside a polygon.
   *
   * @return Id of the polygon that was clicked. (-1 if no polygon was clicked)
   */
  int clickedInsidePolygon(sf::Vector2i cursor);

  /**
   * @brief Creates a dragging force for a polygon.
   *
   * @param pId Id of the polygon for which the force will be created.
   * @param cursor Current position of the cursor.
   */
  void createForce(int pId, sf::Vector2i cursor);

  /**
   * @brief Deletes the current drag force.
   */
  void removeForce();

  /**
   * @brief Draws a line from the dragging polygon to the cursor.
   *
   */
  void drawDragLine();
};
} // namespace ui::frame
