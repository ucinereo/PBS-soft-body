/**
 * @file Mesh.cpp
 * @brief Definitions of the Mesh
 */

#include "Mesh.h"

#include <Eigen/Core>

Mesh::Mesh(Eigen::MatrixXd V, Eigen::MatrixXi F) : V(V), F(F) {}

const Eigen::MatrixXd Mesh::getVertices() const { return V; }

const Eigen::MatrixXi Mesh::getFaces() const { return F; }

void Mesh::updateVertices(Eigen::MatrixXd v) {
  // @TODO: Maybe add some validation here to ensure that the shapes are the
  // same?
  V = v;
}

Eigen::Vector4f Mesh::getColor() const { return color; }

void Mesh::updateColor(float r, float g, float b, float alpha) {
  color << r, g, b, alpha;
}

void Mesh::setID(size_t id) { ID = id; }

size_t Mesh::getID() { return ID; }