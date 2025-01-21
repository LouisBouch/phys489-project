#pragma once
#include "SFMLRenderer.hpp"
#include <string>

namespace ui::frame {

class IWindow {
public:
  /**
   * @brief Default constructor.
   */
  IWindow();
  /**
   * @brief The deconstructor.
   */
  virtual ~IWindow() = default;

  /**
   * @brief Creates window with title.
   */
  virtual void create(const std::string& title) = 0;

  /**
   * @brief Starts the render loop.
   */
  virtual void startRenderLoop() = 0;
  /**
   * @brief Stops the render loop.
   */
  virtual void stopRenderLoop() = 0;
  /**
   * @brief Joins current thread with thread that renders.
   */
  virtual void joinThread() = 0;

  /**
   * @brief Obtains the running state of the simulation.
   *
   * @return Whether the simulation is running or not.
   */
  bool isRunning() const;

  /**
   * @brief Getter for renderer.
   *
   * @return The renderer.
   */
  SFMLRenderer& getRenderer();

  /**
   * @brief Obtains reference to the bufferTIme field.
   *
   * @return Reference to bufferTime field.
   */
  double& getBufferTime() {
    return bufferTime;
  }

private:
  bool running;      //< Whether the rendering is running or not.
  double bufferTime; //< Time to be consumed by physics engine.;

  SFMLRenderer renderer;         //< Renderer for the window. TODO: Make it more
                                 // general, like the IWindow.
  virtual void renderLoop() = 0; //< Method which contains the rendering loop.

protected:
  /**
   * @brief Setter for running value.
   */
  void setRunning(bool running);
};
} // namespace ui::frame
