/**
 * @file Renderer.cpp
 * @brief Definitions of the renderer
 */

#include "Renderer.h"
#include "Shader.h"
#include <iostream>
#include <string>

void Renderer::initialize() {
  // Basic configuration for libigl
  m_viewer.core().is_animating = true;
  m_viewer.data().show_lines = false; // disables wireframes per default
  m_viewer.core().background_color << 0.5f, 0.78f, 0.89f,
      1.f; // set background to blue
  m_viewer.core().is_directional_light = true;
  m_initLightPos << 10.f, 10.f, 10.f;
  m_viewer.core().light_position << m_initLightPos;
  m_viewer.core().is_shadow_mapping = true;

  m_viewer.core().shadow_height = 10000;
  m_viewer.core().shadow_width = 10000;

  //  viewer.launch_init(false, "PBS Soft Body", 3840, 2160);
  m_viewer.launch_init(false, "PBS Soft Body");
  // Initialize the rendering buffers VAO, VBO, EBO and stuff...
  m_viewer.data().meshgl.init();

  m_staticShader =
      Shader("../src/view/shaders/floor.vs", "../src/view/shaders/floor.fs");
  m_staticShader.linkShader(m_viewer);

  // Load the dynamic shader source and compile it
  m_dynamicShader =
      Shader("../src/view/shaders/mesh.vs", "../src/view/shaders/mesh.fs");
  m_dynamicShader.linkShader(m_viewer);

  // Load custom shaders if needed (you'll need to modify libigl to support
  // custom shaders)
  m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) {
    return bgDrawCallback(viewer);
  };
}

void Renderer::registerStatics(std::vector<Mesh> &list) {
  // This is separated if we want to extend the difference between static and
  // dynamics
  for (Mesh &mesh : list) {
    m_renderables.emplace_back(mesh, ShaderType::Static);
    // Set the ID to the last added element
    mesh.setID(m_renderables.size() - 1);
  }
}

void Renderer::registerDynamics(std::vector<Mesh> &list) {
  for (Mesh &mesh : list) {
    m_renderables.emplace_back(mesh, ShaderType::Dynamic);
    // Set the ID to the last added element
    mesh.setID(m_renderables.size() - 1);
  }

  if (list.size() > 0) {
    m_viewer.core().align_camera_center(list[0].getVertices());
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
    m_renderables[mesh.getID()].m_vertices = mesh.getVertices();
    m_renderables[mesh.getID()].m_faces = mesh.getFaces();
  }
}

void Renderer::registerToLibigl() {
  for (size_t i = 0; i < m_renderables.size(); i++) {
    Renderable &renderable = m_renderables[i];
    if (renderable.m_iglViewerID < 0) {
      // Quick hack to add new mesh to the render list
      // viewer.append_mesh() adds a new mesh to its mesh structure and returns
      // the new id.
      int new_id = i > 0 ? m_viewer.append_mesh() : 0;
      renderable.m_iglViewerID = new_id;
    }

    size_t meshIndex = m_viewer.mesh_index(renderable.m_iglViewerID);
    // Set default options for statics and dynamics
    if (renderable.type == ShaderType::Static) {
      m_viewer.data_list[meshIndex].show_lines = false;
    } else {
      m_viewer.data_list[meshIndex].show_lines = true;
    }
    m_viewer.data_list[meshIndex].set_face_based(false);
    m_viewer.data_list[meshIndex].clear();
    m_viewer.data_list[meshIndex].set_mesh(renderable.m_vertices,
                                           renderable.m_faces);
    m_viewer.data_list[meshIndex].set_colors(renderable.m_color);
  }
}

void Renderer::render() {
  // Iterate over all renderables and update the libigl matrices.
  for (size_t i = 0; i < m_renderables.size(); i++) {
    Renderable &renderable = m_renderables[i];
    size_t meshIndex = m_viewer.mesh_index(renderable.m_iglViewerID);
    m_viewer.data_list[meshIndex].set_mesh(renderable.m_vertices,
                                           renderable.m_faces);
    m_viewer.data_list[meshIndex].compute_normals();

    if (renderable.type == ShaderType::Static) {
      m_viewer.data_list[meshIndex].meshgl.shader_mesh =
          m_staticShader.getProgID();
    } else if (renderable.type == ShaderType::Dynamic) {
      m_viewer.data_list[meshIndex].meshgl.shader_mesh =
          m_dynamicShader.getProgID();
    }
  }
}

std::mutex *Renderer::getLock() { return &m_renderLock; }

igl::opengl::glfw::Viewer &Renderer::getViewer() { return m_viewer; }

bool Renderer::bgDrawCallback(igl::opengl::glfw::Viewer &viewer) {
  m_renderLock.lock();
  render();
  m_renderLock.unlock();
  // Currently always return false. Might read into the doc. next
  return false;
}

void Renderer::clear() {
  for (Renderable &r : m_renderables) {
    m_viewer.erase_mesh(m_viewer.mesh_index(r.m_iglViewerID));
  }
  m_renderables.clear();
  //  m_viewer.data().clear();
  //  m_viewer.data().meshgl.init();
}