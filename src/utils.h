/**
 * @file utils.h
 * @brief Collection of utility function which are not directly connected to
 * other classes and methods.
 */

#pragma once

#include <Eigen/Core>

/**
 * @brief Create a floor mesh, consisting of two triangles, which spans the
 * coordinates
 * (-100, -100) to (100, 100)
 * @param V Vertex Matrix reference which is overwritten
 * @param F Fragment Matrix reference which is overwritten
 */
void createFloorMesh(Eigen::MatrixX3d &V, Eigen::MatrixX3i &F) {
  Eigen::MatrixX3d floorV(4, 3);
  Eigen::MatrixX3i floorF(2, 3);

  float w = 100.f;
  floorV << -w, 0.0, -w, // Bottom-left corner
      w, 0.0, w,         // Top-right corner
      w, 0.0, -w,        // Bottom-right corner
      -w, 0.0, w;        // Top-left corner

  // Define faces for the floor (two triangles)
  floorF << 0, 1, 2, // First triangle
      0, 3, 1;       // Second triangle

  // Set correct matrices for return
  V = floorV;
  F = floorF;
}