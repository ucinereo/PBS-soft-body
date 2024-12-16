/**
 * @file ConstraintFactory.h
 * @brief Defines a constraint factory that can be used to construct different
 * constraints
 */
#pragma once

#include "../utils.h"
#include "Constraint.h"
#include "Mesh.h"
#include <Eigen/Core>

class ConstraintFactory {
public:
  using Indices = std::vector<std::pair<Eigen::Index, Eigen::Index>>;

  static void meshDistance(std::vector<Constraint *> &constraints,
                           Eigen::MatrixX3d &positions,
                           const std::vector<Mesh> &meshes,
                           const Indices &indices, double compliance) {
    for (size_t i = 0; i < meshes.size(); i++) {
      const Eigen::MatrixXi TT = meshes[i].getTets();
      Eigen::Index start, length;
      std::tie(start, length) = indices[i];
      for (Eigen::Index j = 0; j < TT.rows(); j++) {
        // Add a distance constraint for each Tet-Edge
        auto *c0 = new DistanceConstraint(compliance, positions,
                                          TT(j, 0) + start, TT(j, 1) + start);
        auto *c1 = new DistanceConstraint(compliance, positions,
                                          TT(j, 0) + start, TT(j, 2) + start);
        auto *c2 = new DistanceConstraint(compliance, positions,
                                          TT(j, 0) + start, TT(j, 3) + start);
        auto *c3 = new DistanceConstraint(compliance, positions,
                                          TT(j, 1) + start, TT(j, 2) + start);
        auto *c4 = new DistanceConstraint(compliance, positions,
                                          TT(j, 1) + start, TT(j, 3) + start);
        auto *c5 = new DistanceConstraint(compliance, positions,
                                          TT(j, 2) + start, TT(j, 3) + start);
        auto *c6 = new DistanceConstraint(compliance, positions,
                                          TT(j, 0) + start, TT(j, 1) + start);
        constraints.push_back(c0);
        constraints.push_back(c1);
        constraints.push_back(c2);
        constraints.push_back(c3);
        constraints.push_back(c4);
        constraints.push_back(c5);
        constraints.push_back(c6);
      }
    }
  }

  static void tetVolume(std::vector<Constraint *> &constraints,
                        Eigen::MatrixX3d &positions,
                        const std::vector<Mesh> &meshes, const Indices &indices,
                        double compliance, double pressure) {
    for (size_t i = 0; i < meshes.size(); i++) {
      const Eigen::MatrixXi TT = meshes[i].getTets();
      Eigen::Index start, length;
      std::tie(start, length) = indices[i];
      for (Eigen::Index j = 0; j < TT.rows(); j++) {
        auto *cTet = new TetVolumeConstraint(
            compliance, positions, pressure, TT(j, 0) + start, TT(j, 1) + start,
            TT(j, 2) + start, TT(j, 3) + start);
        constraints.push_back(cTet);
      }
    }
  }

  static void vertexStaticTriangleInteraction(
      std::vector<Constraint *> &constraints,
      const Eigen::MatrixX3d &initialPositions,
      const Eigen::MatrixX3d &positions, const std::vector<Mesh> &staticMeshes,
      const std::vector<Mesh> &meshes, const Indices &indices,
      double collisionCompliance, double frictionCompliance, double staticMu,
      double kineticMu, const std::vector<double> &slacks) {
    for (size_t i = 0; i < meshes.size(); i++) {
      Eigen::Index s, l;
      std::tie(s, l) = indices[i];

      for (size_t j = 0; j < staticMeshes.size(); j++) {
        const Mesh &obj = staticMeshes[j];
        const Eigen::MatrixX3d V = obj.getVertices();
        const Eigen::MatrixX3i F = obj.getFaces();

        for (Eigen::Index qi = s; qi < s + l; qi++) {
          Eigen::Vector3d qp = initialPositions.row(qi).transpose();
          Eigen::Vector3d q = positions.row(qi).transpose();

          for (Eigen::Index fi = 0; fi < F.rows(); fi++) {
            Eigen::Vector3d x1 = V.row(F(fi, 0)).transpose();
            Eigen::Vector3d x2 = V.row(F(fi, 1)).transpose();
            Eigen::Vector3d x3 = V.row(F(fi, 2)).transpose();

            Eigen::Vector3d n = (x2 - x1).cross(x3 - x1).normalized();
            double restDistance = n.dot(x1);
            double penetrationDepth;

            if (vertexIntersectsTriangle(qp, q, x1, x2, x3, slacks[j],
                                         penetrationDepth)) {
              auto *c0 = new StaticPlaneCollisionConstraint(
                  collisionCompliance, n, restDistance, qi);
              auto *c1 = new PlaneFrictionConstraint(frictionCompliance, n,
                                                     penetrationDepth, staticMu,
                                                     kineticMu, qi);

              constraints.push_back(c0);
              constraints.push_back(c1);
            }
          }
        }
      }
    }
  }
};