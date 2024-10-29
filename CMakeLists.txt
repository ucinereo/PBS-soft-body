cmake_minimum_required(VERSION 3.10)

project(PBS-soft-body)

# opengl handling
find_package(OpenGL REQUIRED)

# error handling
if(P_BINARY_DIR STREQUAL P_SOURCE_DIR)
  message (FATAL_ERROR "Please select another Build Directory!")
endif()
if(P_SOURCE_DIR MATCHES " ")
  message("Your Source Directory contains white spaces. This can cause compiler errors.")
endif()

# Library paths
set(GLFW_SOURCE_DIR "external/glfw")
set(GLAD_SOURCE_DIR "external/glad")
set(GLM_SOURCE_DIR "external/glm")
set(EIGEN_SOURCE_DIR "external/eigen")

# Library: GLFW
# disable documentation, tests and examples of glfw
set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
set(GLFW_INSTALL OFF CACHE BOOL "")
add_subdirectory(${GLFW_SOURCE_DIR})

# Library: GLAD
add_subdirectory(${GLAD_SOURCE_DIR})

# Library: GLM
add_subdirectory(${GLM_SOURCE_DIR})

# Library: Eigen
add_subdirectory(${EIGEN_SOURCE_DIR})

include_directories(
  ${GLFW_SOURCE_DIR}/includes
  ${GLFW_SOURCE_DIR}/deps
)


# Add source files
add_executable(PBS-soft-body
  src/main.cpp
  src/controller/SimulationController.h
  src/controller/SimulationController.cpp
  src/model/SimulationModel.h
  src/model/SimulationModel.cpp
  src/model/Mesh.h
  src/model/Mesh.cpp
  src/view/Renderer.h
  src/view/Renderer.cpp
)

# link libraries
target_link_libraries(
  PBS-soft-body
  glad
  glfw
  glm
  Eigen3::Eigen
)
