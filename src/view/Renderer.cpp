/**
 * @file Renderer.cpp
 * @brief Definitions of the renderer
 */

#include "Renderer.h"
#include "Shader.h"
#include <iostream>
#include <string>

Renderer::Renderer() {
  // Basic configuration for libigl
  viewer.core().is_animating = true;
  viewer.data().show_lines = false; // disables wireframes per default
  viewer.core().background_color << 0.5f, 0.78f, 0.89f,
      1.f; // set background to blue

  viewer.launch_init();
  // Initialize the rendering buffers VAO, VBO, EBO and stuff...
  viewer.data().meshgl.init();

  // @TODO: Don't use relative paths like this. Maybe there is a clean c++
  // variant? Load the static shader source and compile it
  staticShader =
      Shader("../src/view/shaders/floor.vs", "../src/view/shaders/floor.fs");
  staticShader.linkShader(viewer);

  // Load the dynamic shader source and compile it
  dynamicShader =
      Shader("../src/view/shaders/mesh.vs", "../src/view/shaders/mesh.fs");
  dynamicShader.linkShader(viewer);

  // Load custom shaders if needed (you'll need to modify libigl to support
  // custom shaders)
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) {
    return bgDrawCallback(viewer);
  };
}

void Renderer::registerStatics(std::vector<Mesh> &list) {
  // This is separated if we want to extend the difference between static and
  // dynamics
  for (Mesh &mesh : list) {
    renderables.emplace_back(mesh, ShaderType::Static);
    // Set the ID to the last added element
    mesh.setID(renderables.size() - 1);
  }
}

void Renderer::registerDynamics(std::vector<Mesh> &list) {
  for (Mesh &mesh : list) {
    renderables.emplace_back(mesh, ShaderType::Dynamic);
    // Set the ID to the last added element
    mesh.setID(renderables.size() - 1);
  }
}

void Renderer::setMeshData(std::vector<Mesh> &list) {
  for (Mesh &mesh : list) {
    // Note that -1 is used for uninitialized elements.
    if (mesh.getID() == -1) {
      throw std::runtime_error("Mesh need to be registered before update.");
    }

    // Update renderable, note that both getters return a copy and not a
    // reference.
    renderables[mesh.getID()].V = mesh.getVertices();
    renderables[mesh.getID()].F = mesh.getFaces();
  }
}

void Renderer::registerToLibigl() {
  for (size_t i = 0; i < renderables.size(); i++) {
    Renderable &renderable = renderables[i];
    if (renderable.igl_viewer_id < 0) {
      // Quick hack to add new mesh to the render list
      // viewer.append_mesh() adds a new mesh to its mesh structure and returns
      // the new id.
      int new_id = i > 0 ? viewer.append_mesh() : 0;
      renderable.igl_viewer_id = new_id;
    }

    // @TODo: Add colors of the mesh
    size_t meshIndex = viewer.mesh_index(renderable.igl_viewer_id);
    viewer.data_list[meshIndex].show_lines = true;
    viewer.data_list[meshIndex].set_face_based(false);
    viewer.data_list[meshIndex].clear();
  }
}

void Renderer::render() {
  // Iterate over all renderables and update the libigl matrices.
  for (size_t i = 0; i < renderables.size(); i++) {
    Renderable &renderable = renderables[i];
    size_t meshIndex = viewer.mesh_index(renderable.igl_viewer_id);
    viewer.data_list[meshIndex].set_mesh(renderable.V, renderable.F);
    viewer.data_list[meshIndex].compute_normals();

    if (renderable.type == ShaderType::Static) {
      viewer.data_list[meshIndex].meshgl.shader_mesh = staticShader.getProgID();
    } else if (renderable.type == ShaderType::Dynamic) {
      viewer.data_list[meshIndex].meshgl.shader_mesh =
          dynamicShader.getProgID();
    }
  }
}

std::mutex *Renderer::getLock() { return &renderLock; }

igl::opengl::glfw::Viewer &Renderer::getViewer() { return viewer; }

void Renderer::initCustomShader() {
  // @TODO: check if the first two calls are really necessary
  viewer.data().set_face_based(false);
  viewer.launch_init();
  viewer.data().meshgl.init();
  igl::opengl::destroy_shader_program(viewer.data().meshgl.shader_mesh);
}

bool Renderer::bgDrawCallback(igl::opengl::glfw::Viewer &viewer) {
  renderLock.lock();
  render();
  renderLock.unlock();
  // Currently always return false. Might read into the doc. next
  return false;
}