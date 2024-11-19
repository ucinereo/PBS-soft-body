/**
 * @file GuiController.h
 * @brief Adds an IMGUI menu on top of the current simulator.
 */
#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>

/**
 * @class GuiController
 * @brief Manages the user inputs on the GUI menu and maps it to the
 * corresponding functions in the SimulationController.
 */
class GuiController {
public:
  /**
   * @brief Construct a new Gui Controller object
   * @param viewer Libigl viewer from renderer
   */
  GuiController(igl::opengl::glfw::Viewer &viewer);

private:
  /**
   * @brief Draws the actual imgui menu and links it to the controller.
   * @param viewer Libigl viewer (TODO: might not be necessary)
   * @param menu_width Width of the menu
   */
  void drawMenu(igl::opengl::glfw::Viewer &viewer, float menu_width);

  igl::opengl::glfw::imgui::ImGuiPlugin plugin; ///< plugin to register the menu
  igl::opengl::glfw::imgui::ImGuiMenu menu; ///< the actual imgui menu component
};