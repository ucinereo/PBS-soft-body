/**
 * @file Mesh.cpp
 * @brief Definitions of the Mesh
 */

#include "Mesh.h"
#include <Eigen/Core>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/upsample.h>

void Mesh::upsample(int nSubdivs) {
  igl::upsample(m_vertices, m_faces, nSubdivs);
}

void Mesh::tetrahedralize() {
  Eigen::MatrixX3d tetVertices; // Tetrahedral mesh vertices
  Eigen::MatrixX4i tets;        // Tetrahedral mesh tetrahedra
  Eigen::MatrixX3i tetFaces;    // Boundary faces of the tetrahedral mesh

  // Use igl::copyleft::tetgen::tetrahedralize
  std::string flags = "pq2.0Y"; // Quality tetrahedralization, adapt as needed

  igl::copyleft::tetgen::tetrahedralize(m_vertices, m_faces, flags, tetVertices,
                                        tets, tetFaces);

  m_vertices = tetVertices;
  m_initialVertices = tetVertices;
  m_faces = tetFaces;
  m_tets = tets;
}

void Mesh::buildBVH(double slack) {
  m_bvh = new BVH(m_vertices, m_faces, slack);
}

void Mesh::query(Eigen::Vector3d &q,
                 std::vector<Eigen::Index> &triangles) const {
  if (m_bvh != nullptr) {
    m_bvh->query(q, triangles);
  } else {
    triangles.resize(m_faces.rows());
    std::iota(triangles.begin(), triangles.end(), 0);
  }
}

Mesh Mesh::createFloor() {
  Eigen::MatrixX3d V(4, 3);
  Eigen::MatrixX3i F(2, 3);

  float w = 100.f;
  V << -w, 0.0, -w, // Bottom-left corner
      w, 0.0, w,    // Top-right corner
      w, 0.0, -w,   // Bottom-right corner
      -w, 0.0, w;   // Top-left corner

  // Define faces for the floor (two triangles)
  F << 0, 1, 2, // First triangle
      0, 3, 1;  // Second triangle

  return Mesh(V, F);
}

Mesh Mesh::createCube(Eigen::Affine3d &toWorld) {
  Eigen::MatrixX3d V;
  Eigen::MatrixX3i F;

  igl::readOBJ("../assets/cube_1x.obj", V, F);

  return Mesh(V, F, toWorld);
}

Mesh Mesh::createDuck(Eigen::Affine3d &toWorld) {
  Eigen::MatrixX3d V;
  Eigen::MatrixX3i F;

  igl::readOBJ("../assets/rubber_duck.obj", V, F);

  return Mesh(V, F, toWorld);
}