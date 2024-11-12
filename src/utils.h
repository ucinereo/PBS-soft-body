#pragma once

#include <Eigen/Core>

void createFloorMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
    Eigen::MatrixXd floorV(4, 3);
    Eigen::MatrixXi floorF(2, 3);

    float w = 100.f;
    floorV << -w, 0.0, -w,  // Bottom-left corner
               w, 0.0,  w,  // Top-right corner
               w, 0.0, -w,  // Bottom-right corner
              -w, 0.0,  w;  // Top-left corner

    // Define faces for the floor (two triangles)
    floorF << 0, 1, 2,  // First triangle
              0, 3, 1;  // Second triangle

    // Set correct matrices for return
    V = floorV;
    F = floorF;
}