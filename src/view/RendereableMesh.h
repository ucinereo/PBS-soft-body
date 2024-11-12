#pragma once

#include <Eigen/Core>
#include "../model/Mesh.h"

enum class ShaderType {
    Static,
    Dynamic,
};

struct Renderable {
    const ShaderType type;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::Vector4f c; // TODO: Update for matrix

    int renderable_index;
    int igl_viewer_id;

    Renderable(Mesh &mesh, const ShaderType type) : type(type) {
        V = mesh.getVertices();
        F = mesh.getFaces();
        c = mesh.getColor();
        renderable_index = -1;
        igl_viewer_id = -1;
    };
};