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

    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

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
  float complianceDistance = this->controller->getComplianceDistance();
  float complianceStaticPlane = this->controller->getComplianceStaticPlane();
  float compliancePlaneFriction =
      this->controller->getCompliancePlaneFriction();
  float complianceVolume = this->controller->getComplianceVolume();
  float pressure = this->controller->getPressure();
  float friction = this->controller->getFriction();

  bool running = this->controller->getIsSimulationRunning();

  if (ImGui::CollapsingHeader("Simulation settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {

    if (ImGui::Button("Single step", ImVec2(button_width, 0))) {
      std::cout << "executing single time step \n";
      this->controller->singleStep();
    }

    if (ImGui::Button("Reset Simulation", ImVec2(button_width, 0))) {
      std::cout << "not implemented yet \n";
      this->controller->resetSimulation();
      // this->controller->resetSimulation();
    }

    if (ImGui::Button("Start/Stop Simulation", ImVec2(button_width, 0))) {
      if (running) {
        this->controller->stopSimulation();
      } else {
        this->controller->startSimulation();
      }
    }

    if (ImGui::Button("Export Object", ImVec2(button_width, 0))) {
      this->controller->exportObj();
    }
  }
  if (ImGui::CollapsingHeader("Soft body parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {

    if (ImGui::SliderInt("Time step", &timeStep, 20, 100)) {
      std::cout << "current selected time step size is " << timeStep << "\n";
      this->controller->setTimeStep(timeStep);
    }

    if (ImGui::SliderFloat("friction", &friction, 0.0f, 1.0f)) {
      std::cout << "current selected friction value is " << friction << "\n";
      this->controller->setFriction(friction);
    }

    if (ImGui::SliderFloat("pressure", &pressure, 0.0f, 10.0f)) {
      std::cout << "current selected pressure value is " << pressure << "\n";
      this->controller->setPressure(pressure);
    }

    if (ImGui::CollapsingHeader("Distance constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::SliderFloat("cdistance ompliance", &complianceDistance, 0.0f,
                             10000.0f)) {
        std::cout << "current selected compliance value is "
                  << complianceDistance << "\n";
        this->controller->setComplianceDistance(complianceDistance);
      }
    }

    if (ImGui::CollapsingHeader("StaticPlaneCollision constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::SliderFloat("static compliance", &complianceStaticPlane, 0.0f,
                             1.0f)) {
        std::cout << "current selected compliance value is "
                  << complianceStaticPlane << "\n";
        this->controller->setComplianceStaticPlane(complianceStaticPlane);
      }
    }

    if (ImGui::CollapsingHeader("PlaneFriction constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::SliderFloat("Plane friction compliance",
                             &compliancePlaneFriction, 0.0f, 1.0f)) {
        std::cout << "current selected compliance value is "
                  << compliancePlaneFriction << "\n";
        this->controller->setCompliancePlaneFriction(compliancePlaneFriction);
      }
    }

    if (ImGui::CollapsingHeader("Volume constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::SliderFloat("Volume compliance", &complianceVolume, 0.0f,
                             1.0f)) {
        std::cout << "current selected compliance value is " << complianceVolume
                  << "\n";
        this->controller->setComplianceVolume(complianceVolume);
      }
    }
  }
}
