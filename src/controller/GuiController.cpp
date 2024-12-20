/**
 * @file GuiController.cpp
 * @brief Defintions of GuiController
 */
#include "GuiController.h"
#include "../model/Constraint.h"
#include "SimulationController.h"

GuiController::GuiController(SimulationController *controller,
                             igl::opengl::glfw::Viewer &viewer) {

  this->m_controller = controller;

  // Link the menu to the viewer
  viewer.plugins.push_back(&m_plugin);
  m_plugin.widgets.push_back(&m_menu);

  m_menu.callback_draw_viewer_menu = [&]() {
    float menuWidth = 300.0f; // Fixed menu width

    // Set the window position and size constraints
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f),
                            ImGuiCond_::ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(menuWidth, -1.0f), // Fixed width, flexible height
        ImVec2(menuWidth, -1.0f)  // Fixed width, flexible height
    );

    bool _viewer_menu_visible = true;

    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

    ImGui::Begin("Viewer", &_viewer_menu_visible,
                 ImGuiWindowFlags_NoSavedSettings);

    drawMenu(viewer, menuWidth);

    ImGui::End();
  };
}

void GuiController::drawMenu(igl::opengl::glfw::Viewer &viewer,
                             float menuWidth) {
  float buttonWidth = menuWidth - ImGui::GetStyle().WindowPadding.x * 2;

  // get Simulation parameter values
  ESceneType sceneType = this->m_controller->getSceneType();
  int timeStep = this->m_controller->getTimeStep();
  float complianceDistance = this->m_controller->getCompliance(EDistance);
  float complianceVolume = this->m_controller->getCompliance(ETetVolume);
  float pressure = this->m_controller->getPressure();
  float staticFriction = this->m_controller->getStaticFriction();
  float kineticFriction = this->m_controller->getKineticFriction();

  bool running = this->m_controller->getIsSimulationRunning();

  static bool activeVolume = true;
  static bool activeDistance = true;
  static bool activeStaticPlane = true;
  static bool activeFriction = true;

  ImGui::Text("FPS: %d", (int)this->m_controller->getFPS());

  std::vector<std::string> sceneTypes{"Palm Trees", "Cuboids, BVH: off",
                                      "Cuboids: BVH: on"};

  if (ImGui::Combo("Scene Type", reinterpret_cast<int *>(&sceneType),
                   sceneTypes)) {
    std::cout << "Selected scene type " << sceneType << std::endl;
    this->m_controller->replaceScene(sceneType);
  }

  if (ImGui::CollapsingHeader("Simulation settings",
                              ImGuiTreeNodeFlags_DefaultOpen)) {

    if (ImGui::Button("Single step", ImVec2(buttonWidth, 0))) {
      //      std::cout << "executing single time step \n";
      this->m_controller->singleStep();
    }

    if (ImGui::Button("Reset Simulation", ImVec2(buttonWidth, 0))) {
      //      std::cout << "simulation is reset";
      this->m_controller->resetSimulation();
    }

    if (ImGui::Button("Start/Stop Simulation", ImVec2(buttonWidth, 0))) {
      if (running) {
        this->m_controller->stopSimulation();
      } else {
        this->m_controller->startSimulation();
      }
    }

    if (ImGui::Button("Export Object", ImVec2(buttonWidth, 0))) {
      this->m_controller->exportObj();
    }
  }
  if (ImGui::CollapsingHeader("Soft body parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {

    if (ImGui::SliderInt("Time step", &timeStep, 1, 60)) {
      //      std::cout << "current selected time step size is " << timeStep <<
      //      "\n";
      this->m_controller->setTimeStep(timeStep);
    }

    if (ImGui::CollapsingHeader("Distance constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {

      if (ImGui::Checkbox("Distance active", &activeDistance)) {

        //        std::cout << "Is Distance active: " << std::boolalpha <<
        //        activeDistance
        //                  << std::endl;
        this->m_controller->setState(EDistance, activeDistance);
      }
      if (ImGui::SliderFloat("distance compliance", &complianceDistance, 0.0f,
                             1.0f)) {
        //        std::cout
        //            << "current selected compliance value for distance
        //            constraint is "
        //            << complianceDistance << "\n";
        this->m_controller->setCompliance(EDistance, complianceDistance);
      }
    }

    if (ImGui::CollapsingHeader("Volume constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Checkbox("Volume active", &activeVolume)) {

        //        std::cout << "Is Volume active: " << std::boolalpha <<
        //        activeVolume
        //                  << std::endl;
        this->m_controller->setState(ETetVolume, activeVolume);
      }

      if (ImGui::SliderFloat("Volume compliance", &complianceVolume, 0.0f,
                             1.0f)) {
        //        std::cout << "current selected compliance value for volume is
        //        "
        //                  << complianceVolume << "\n";
        this->m_controller->setCompliance(ETetVolume, complianceVolume);
      }

      if (ImGui::SliderFloat("pressure", &pressure, 0.0f, 10.0f)) {
        //        std::cout << "current selected pressure value is " << pressure
        //        << "\n";
        this->m_controller->setPressure(pressure);
      }
    }

    if (ImGui::CollapsingHeader("Friction constraint",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::SliderFloat("static friction", &staticFriction, 0.0f, 1.0f)) {
        //        std::cout << "current selected static friction value is "
        //                  << staticFriction << "\n";
        this->m_controller->setStaticFriction(staticFriction);
      }

      if (ImGui::SliderFloat("kinetic friction", &kineticFriction, 0.0f,
                             1.0f)) {
        //        std::cout << "current selected kinetic friction value is "
        //                  << kineticFriction << "\n";
        this->m_controller->setKineticFriction(kineticFriction);
      }
    }
  }
}
