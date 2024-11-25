/**
 * @file GuiController.cpp
 * @brief Defintions of GuiController
 */
#include "GuiController.h"
#include "SimulationController.h"

GuiController::GuiController(SimulationController *controller,
                             igl::opengl::glfw::Viewer &viewer) {

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

void GuiController::drawMenu(igl::opengl::glfw::Viewer &viewer,
                             float menu_width) {
  float button_width = menu_width - ImGui::GetStyle().WindowPadding.x * 2;

  // get Simulation parameter values
  int timeStep = this->controller->getTimeStep();
  float compliance = this->controller->getCompliance();
  float pressure = this->controller->getPressure();

  bool running = this->controller->getIsSimulationRunning();

  if (ImGui::CollapsingHeader("Simulation settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {

    if (ImGui::Button("Single step", ImVec2(button_width, 0))) {
      std::cout << "not implemented yet \n";
      this->controller->singleStep();
    }

    if (ImGui::Button("Reset Simulation", ImVec2(button_width, 0))) {
      std::cout << "not implemented yet \n";
      this->controller->resetSimulation();
    }

    if (ImGui::Button("Start/Stop Simulation", ImVec2(button_width, 0))) {
      if (running) {
        this->controller->stopSimulation();
      } else {
        this->controller->startSimulation();
      }
    }
  }
  if (ImGui::CollapsingHeader("Soft body parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::InputInt("Time step", &timeStep)) {
      std::cout << "current selected time step size is " << timeStep << "\n";
      this->controller->setTimeStep(timeStep);
    }

    if (ImGui::SliderFloat("inverse stiffness", &compliance, 0.0f, 1.0f)) {
      std::cout << "current selected stiffness value is " << compliance << "\n";
      this->controller->setCompliance(compliance);
    }

    if (ImGui::SliderFloat("pressure", &pressure, 0.0f, 10.0f)) {
      std::cout << "current selected pressure value is " << pressure << "\n";
      this->controller->setPressure(pressure);
    }
  }
}
