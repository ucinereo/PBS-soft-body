/**
 * @file GuiController.cpp
 * @brief Defintions of GuiController
 */
#include "GuiController.h"
#include "SimulationController.h"

GuiController::GuiController(SimulationController *controller, igl::opengl::glfw::Viewer &viewer) {

  this->controller = controller;

  // Link the menu to the viewer
  viewer.plugins.push_back(&plugin);
  plugin.widgets.push_back(&menu);

  // TODO: Adapt size etc such that it looks good :)
  menu.callback_draw_viewer_menu = [&]() {
    float menu_width = 300.0f; // Fixed menu width

    // Set the window position and size constraints
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f),
                            ImGuiCond_::ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(menu_width, -1.0f), // Fixed width, flexible height
        ImVec2(menu_width, -1.0f)  // Fixed width, flexible height
    );

    bool _viewer_menu_visible = true;

    ImGui::Begin("Viewer", &_viewer_menu_visible,
                 ImGuiWindowFlags_NoSavedSettings);

    drawMenu(viewer, menu_width);

    ImGui::End();
  };
}

// void GuiController::updateParameters() {
//   time_step->updateTimestep(time_step);
// }

void GuiController::drawMenu(igl::opengl::glfw::Viewer &viewer,
                             float menu_width) {
  float button_width = menu_width - ImGui::GetStyle().WindowPadding.x * 2;

  float timeStep = this->controller->getTimeStep();
  float stiffness = this->controller->getStiffness();
  float pressure = this->controller->getPressure();

  if (ImGui::Button("Test Button :-)", ImVec2(button_width, 0))) {
    std::cout << "lol\n";
  }
  if (ImGui::CollapsingHeader("Simulation settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Run Simulation", ImVec2(button_width, 0))) {
      std::cout << "not implemented yet \n";
    }

    if (ImGui::Button("Single step", ImVec2(button_width, 0))) {
      std::cout << "not implemented yet \n";
    }

    if (ImGui::Button("Reset Simulation", ImVec2(button_width, 0))) {
      std::cout << "not implemented yet \n";
    }

  }
  if (ImGui::CollapsingHeader("Soft body parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::InputFloat("Time step", &timeStep)) {
      std::cout << "current selected time step size is " << timeStep << "\n";
      this->controller->setTimeStep(timeStep);
    }

    if (ImGui::SliderFloat("stiffness", &stiffness, 0.0f, 1.0f)) {
      std::cout << "current selected stiffness value is " << stiffness << "\n";
      this->controller->setStiffness(stiffness);
    }

    if (ImGui::SliderFloat("pressure", &pressure, 0.0f, 10.0f)) {
      std::cout << "current selected pressure value is " << pressure << "\n";
      this->controller->setPressure(pressure);
    }
  }
}
