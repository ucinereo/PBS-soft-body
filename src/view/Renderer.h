/**
 * @file Renderer.h
 * @brief Defines the renderer class which connects the physical simulation with
 * libigl's viewer object. Furthermore allows to add custom shaders to overwrite
 * the default libigl ones.
 */
#pragma once

#include "../model/Mesh.h"
#include "RendereableMesh.h"
#include "Shader.h"
#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <mutex>

/**
 * @class Renderer
 * @brief Renderer class which is responsible for initializing the libigl
 * viewer, updating the libigl shaders and drawing the current simulation state.
 * Maintains a list of renderable objects, which is essentially an extension of
 * the mesh class.
 */
class Renderer {
public:
  /**
   * @brief Construct a new Renderer object.
   */
  Renderer(){};

  /**
   * @brief Initializes the libigl viewer and links the custom shaders for
   * dynamic and static meshes. Links the function which is called before every
   * drawing step of the libigl viewer.
   */
  void initialize();

  /**
   * @brief Adds the static objects to internal object list and sets their new
   * ids
   * @param list List of static objects
   */
  void registerStatics(std::vector<Mesh> &list);

  /**
   * @brief Adds the dynamic objects to internal object list and sets their new
   * ids
   * @param list List of dynamic objects
   */
  void registerDynamics(std::vector<Mesh> &list);

  /**
   * @brief Update the internal representation of the meshes. This depends on
   * the ID stored on each mesh. Be sure to register the objects first, else it
   * is undefined. behavior.
   * @param list Updated list of meshes
   */
  void setMeshData(std::vector<Mesh> &list);

  /**
   * @brief Should be called after all objects have been registered. This adds
   * the objects to libigl and stores the libigl id in the corresponding
   * `Renderable` objects.
   */
  void registerToLibigl();

  /**
   * @brief Get the mutex lock of the render thread
   * @return std::mutex Render lock
   */
  std::mutex *getLock();

  /**
   * @brief Get the libigl Viewer object
   * @return igl::opengl::glfw::Viewer& reference to viewer
   */
  igl::opengl::glfw::Viewer &
  getViewer(); // Access the viewer for custom configuration

private:
  /**
   * @brief Actual render function which overwrites the meshes of the libigl
   * viewer, sets the custom shaders and computes the normals of the objects.
   * This is called by the registered callback function of libigl, not manually,
   * as it does not guarantee mutual exclusion.
   */
  void render();

  /**
   * @brief Libigl drawback function which actually calls the render function,
   * but mutually exclusive, such that we don't have a read/write conflict.
   * @param viewer Libigl viewer
   * @return Currently always returns false :) @TODO: Maybe update this??
   */
  bool bgDrawCallback(igl::opengl::glfw::Viewer &viewer);

  std::mutex renderLock; ///< Lock which is necessary to avoid race conditions
  igl::opengl::glfw::Viewer viewer; ///< Libigl viewer
  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  igl::opengl::glfw::imgui::ImGuiMenu menu;

  Shader staticShader;  ///< Shader for static objects
  Shader dynamicShader; ///< Shader for dynamic objects
  Eigen::Vector3f init_light_pos;

  std::vector<Renderable>
      renderables; ///< List which stores all renderable meshes
};