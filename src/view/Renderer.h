#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <mutex>
#include "Shader.h"
#include "RendereableMesh.h"
#include "../model/Mesh.h"

class Renderer {
public:
    Renderer();

    void registerStatics(std::vector<Mesh> &list);
    void registerDynamics(std::vector<Mesh> &list);
    void setMeshData(std::vector<Mesh> &list);
    void updateRenderGeometry();
    void registerToLibigl();

    std::mutex *getLock();
    igl::opengl::glfw::Viewer &getViewer(); // Access the viewer for custom configuration
    void initCustomShader();
    void render();

private:
    std::mutex renderLock;
    igl::opengl::glfw::Viewer viewer;

    Shader staticShader;
    Shader dynamicShader;

    std::vector<Renderable> renderables;

    bool bgDrawCallback(igl::opengl::glfw::Viewer &viewer);
};
