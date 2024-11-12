#include "Mesh.h"

#include <Eigen/Core>

Mesh::Mesh(Eigen::MatrixXd V, Eigen::MatrixXi F) : V(V), F(F) {}

const Eigen::MatrixXd Mesh::getVertices() const {
    return V;
}

const Eigen::MatrixXi Mesh::getFaces() const {
    return F;
}

void Mesh::updateVertices(Eigen::MatrixXd v) {
    V = v;
}

Eigen::Vector4f Mesh::getColor() const {
    return color;
}

void Mesh::updateColor(float r, float g, float b, float alpha) {
    color << r, g, b, alpha;
}

void Mesh::setID(size_t id) {
    ID = id;
}

size_t Mesh::getID() {
    return ID;
}
