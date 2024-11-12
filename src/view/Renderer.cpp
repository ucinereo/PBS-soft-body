#include "Renderer.h"
#include <iostream>
#include <string>
#include "Shader.h"

Renderer::Renderer() {
    viewer.core().is_animating = true;
    viewer.data().show_lines = false;
    viewer.core().background_color << 0.5f, 0.78f, 0.89f, 1.f;

    viewer.launch_init();
    viewer.data().meshgl.init();
    staticShader = Shader("../src/view/shaders/floor.vs", "../src/view/shaders/floor.fs");
    staticShader.linkShader(viewer);

    dynamicShader = Shader("../src/view/shaders/mesh.vs", "../src/view/shaders/mesh.fs");
    dynamicShader.linkShader(viewer);

    // Load custom shaders if needed (you'll need to modify libigl to support custom shaders)
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) {
		return bgDrawCallback(viewer);
	};
}


void Renderer::setMeshData(std::vector<Mesh> &list) {
    for (Mesh &mesh : list) {
        if (mesh.getID() == -1) {
            throw std::runtime_error("Mesh need to be registered before update.");
        }

        // Update renderable
        renderables[mesh.getID()].V = mesh.getVertices();
        renderables[mesh.getID()].F = mesh.getFaces();
    }
}

void Renderer::updateRenderGeometry() {}

void Renderer::registerToLibigl() {
    for(size_t i = 0; i < renderables.size(); i++) {
        Renderable &renderable = renderables[i];
        if (renderable.igl_viewer_id < 0) {
            // Quick hack to add new mesh to the render list
            int new_id = i > 0 ? viewer.append_mesh() : 0;
            renderable.igl_viewer_id = new_id;
        }
        size_t meshIndex = viewer.mesh_index(renderable.igl_viewer_id);
        viewer.data_list[meshIndex].show_lines = true;
        viewer.data_list[meshIndex].set_face_based(false);
        viewer.data_list[meshIndex].clear();
    }
}

void Renderer::render() {
    for(size_t i = 0; i < renderables.size(); i++) {
        Renderable &renderable = renderables[i];
        size_t meshIndex = viewer.mesh_index(renderable.igl_viewer_id);
        viewer.data_list[meshIndex].set_mesh(renderable.V, renderable.F);
        viewer.data_list[meshIndex].compute_normals();

        if (renderable.type == ShaderType::Static) {
            viewer.data_list[meshIndex].meshgl.shader_mesh = staticShader.getProgID();
        } else if (renderable.type == ShaderType::Dynamic) {
            viewer.data_list[meshIndex].meshgl.shader_mesh = dynamicShader.getProgID();
        }
    }
}

void Renderer::registerDynamics(std::vector<Mesh> &list) {
    for (Mesh &mesh : list) {
        renderables.emplace_back(mesh, ShaderType::Dynamic);
        mesh.setID(renderables.size() - 1);
    }
}

void Renderer::registerStatics(std::vector<Mesh> &list) {
    for (Mesh &mesh : list) {
        renderables.emplace_back(mesh, ShaderType::Static);
        mesh.setID(renderables.size() - 1);
    }
}

std::mutex *Renderer::getLock() { return &renderLock; }

igl::opengl::glfw::Viewer &Renderer::getViewer() { return viewer; }

bool Renderer::bgDrawCallback(igl::opengl::glfw::Viewer &viewer) {
    renderLock.lock();
    render();
    renderLock.unlock();
    return false;
}

void Renderer::initCustomShader() {
    viewer.data().set_face_based(false);
    viewer.launch_init();
    viewer.data().meshgl.init();
    igl::opengl::destroy_shader_program(viewer.data().meshgl.shader_mesh);

}
