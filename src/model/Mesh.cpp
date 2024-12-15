/**
 * @file Mesh.cpp
 * @brief Definitions of the Mesh
 */

#include "Mesh.h"

#include <Eigen/Core>

Mesh::Mesh(Eigen::MatrixX3d V, Eigen::MatrixX3i F) : V(V), F(F) {}

Mesh::Mesh(Eigen::MatrixX3d V, Eigen::MatrixX3i F, Eigen::MatrixXi TT)
    : V(V), F(F), TT(TT) {}

const Eigen::Index Mesh::numVertices() const { return V.rows(); }

const Eigen::MatrixX3d Mesh::getVertices() const { return V; }

const Eigen::MatrixX3i Mesh::getFaces() const { return F; }

const Eigen::MatrixXi Mesh::getTetIndices() const { return TT; };

const Eigen::Index Mesh::numFaces() const { return F.rows(); }

void Mesh::updateVertices(Eigen::MatrixX3d v) {
  // @TODO: Maybe add some validation here to ensure that the shapes are the
  // same?
  V = v;
}

Eigen::RowVector3d Mesh::getColor() const { return color; }

void Mesh::updateColor(double r, double g, double b) { color << r, g, b; }

void Mesh::setID(size_t id) { ID = id; }

size_t Mesh::getID() { return ID; }