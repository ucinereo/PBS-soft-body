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

/**
 * @class ConstraintFactory
 * @brief Utility class for constructing different constraints
 */
class ConstraintFactory {
public:
  using Indices = std::vector<std::pair<Eigen::Index, Eigen::Index>>;

  /**
   * @brief Creates a distance constraint on all the edges of each triangle for
   * all meshes
   * @param constraints Will be populated with the new constraints
   * @param positions The vertex positions
   * @param meshes A list of the meshes for which to create these constraints
   * @param indices A list of index starts and lengths for each mesh which
   * define the locations in the global vertex matrix
   * @param compliance The constraint compliance value
   */
  static void meshFaceDistance(std::vector<Constraint *> &constraints,
                               Eigen::MatrixX3d &positions,
                               const std::vector<Mesh> &meshes,
                               const Indices &indices, double compliance) {
    for (size_t i = 0; i < meshes.size(); i++) {
      const Eigen::MatrixX3i F = meshes[i].getFaces();
      Eigen::Index start, length;
      std::tie(start, length) = indices[i];
      for (Eigen::Index j = 0; j < F.rows(); j++) {
        // Add a distance constraint for each Face-Edge
        auto *c0 = new DistanceConstraint(compliance, positions,
                                          F(j, 0) + start, F(j, 1) + start);
        auto *c1 = new DistanceConstraint(compliance, positions,
                                          F(j, 0) + start, F(j, 2) + start);
        auto *c2 = new DistanceConstraint(compliance, positions,
                                          F(j, 1) + start, F(j, 2) + start);
        constraints.push_back(c0);
        constraints.push_back(c1);
        constraints.push_back(c2);
      }
    }
  }

  /**
   * @brief Creates a distance constraint on all the edges of each tetrahedron
   * for all meshes
   * @param constraints Will be populated with the new constraints
   * @param positions The vertex positions
   * @param meshes A list of the meshes for which to create these constraints
   * @param indices A list of index starts and lengths for each mesh which
   * define the locations in the global vertex matrix
   * @param compliance The constraint compliance value
   */
  static void meshTetDistance(std::vector<Constraint *> &constraints,
                              Eigen::MatrixX3d &positions,
                              const std::vector<Mesh> &meshes,
                              const Indices &indices, double compliance) {
    for (size_t i = 0; i < meshes.size(); i++) {
      const Eigen::MatrixX4i TT = meshes[i].getTets();
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

  /**
   * @brief Creates a tet-volume constraint on all the tetrahedrons for all
   * meshes
   * @param constraints Will be populated with the new constraints
   * @param positions The vertex positions
   * @param meshes A list of the meshes for which to create these constraints
   * @param indices A list of index starts and lengths for each mesh which
   * define the locations in the global vertex matrix
   * @param compliance The constraint compliance value
   * @param pressure The pressure value of the constraint
   */
  static void tetVolume(std::vector<Constraint *> &constraints,
                        Eigen::MatrixX3d &positions,
                        const std::vector<Mesh> &meshes, const Indices &indices,
                        double compliance, double pressure) {
    for (size_t i = 0; i < meshes.size(); i++) {
      const Eigen::MatrixX4i TT = meshes[i].getTets();
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

  /**
   * @brief Creates a collision and a friction constraint for all
   * vertex<->triangle interactions that are detected in the scene. This
   * function handles the collision detection itself.
   * @param constraints Will be populated with the new constraints
   * @param initialPositions The initial vertex positions
   * @param positions The vertex positions
   * @param staticMeshes A list of the static meshes for which to create these
   * constraints
   * @param meshes A list of the dynamic meshes for which to create these
   * constraints
   * @param indices A list of index starts and lengths for each mesh which
   * define the locations in the global vertex matrix
   * @param collisionCompliance The collision constraint compliance value
   * @param frictionCompliance The friction constraint compliance value
   * @param staticMu The static friction coefficient
   * @param kineticMu The kinetic friction coefficient
   * @param slacks The collision slack values
   */
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
        const Eigen::MatrixX3d &V = obj.getVertices();
        const Eigen::MatrixX3i &F = obj.getFaces();

        for (Eigen::Index qi = s; qi < s + l; qi++) {
          Eigen::Vector3d qp = initialPositions.row(qi).transpose();
          Eigen::Vector3d q = positions.row(qi).transpose();

          std::vector<Eigen::Index> triangles;
          obj.query(q, triangles);

          for (Eigen::Index fi : triangles) {
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