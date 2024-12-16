cmake_minimum_required(VERSION 3.10)

project(PBS-soft-body)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external)

# Libigl
include(libigl)

# Enable the target igl::glfw
igl_include(glfw)
igl_include(imgui)
igl_include(copyleft tetgen)

# Add source files
add_executable(PBS-soft-body
  src/main.cpp
  src/controller/SimulationController.h
  src/controller/SimulationController.cpp
  src/controller/GuiController.h
  src/controller/GuiController.cpp
  src/model/SimulationModel.h
  src/model/SimulationModel.cpp
  src/model/Mesh.h
  src/model/Mesh.cpp
  src/model/Constraint.h
  src/view/RendereableMesh.h
  src/view/Renderer.h
  src/view/Renderer.cpp
  src/view/Shader.h
  src/view/Shader.cpp
  src/utils.h
  src/model/Constraint.h
  src/model/constraints/Distance.cpp
  src/model/constraints/StaticPlaneCollision.cpp
  src/model/constraints/PlaneFriction.cpp
  src/model/constraints/Volume.cpp
        src/scenes.h
        src/model/ConstraintFactory.h
)

# link libraries
target_link_libraries(
  PBS-soft-body
  igl::glfw
  igl::imgui
  igl_copyleft::tetgen
)
