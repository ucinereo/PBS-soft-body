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