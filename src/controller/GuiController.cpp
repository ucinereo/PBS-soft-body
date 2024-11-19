/**
 * @file GuiController.cpp
 * @brief Defintions of GuiController
 */
#include "GuiController.h"

GuiController::GuiController(igl::opengl::glfw::Viewer &viewer) {
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
  if (ImGui::Button("Test Button :-)", ImVec2(button_width, 0))) {
    std::cout << "lol\n";
  }
}
