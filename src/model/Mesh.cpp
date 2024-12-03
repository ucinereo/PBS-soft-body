/**
 * @file Mesh.cpp
 * @brief Definitions of the Mesh
 */

#include "Mesh.h"

#include <Eigen/Core>

Mesh::Mesh(Eigen::MatrixX3d V, Eigen::MatrixX3i F) : V(V), F(F) {}

const Eigen::Index Mesh::numVertices() const { return V.rows(); }

const Eigen::MatrixX3d Mesh::getVertices() const { return V; }

const Eigen::MatrixX3i Mesh::getFaces() const { return F; }

const Eigen::Index Mesh::numFaces() const { return F.rows(); }

void Mesh::updateVertices(Eigen::MatrixX3d v) {
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