/**
 * @file StaticPlane.h
 * @brief Defines a collision constraint between a vertex and a flat plane.
 */
#pragma once

#include "../Constraint.h"
#include <Eigen/Core>

/**
 * @class StaticPlaneConstraint
 * @brief Collision constraint between a vertex and a plane given by its normal
 * vector
 */
class StaticPlaneConstraint : public Constraint {
public:
  /**
   * @brief Create a new Static Plane Constraint.
   * @param normal The normal vector of the plane
   * @param restDistance The distance from the plane when the object is at rest
   * (i.e. if the constraint is satisfied)
   * @param indexU Index of the vertex
   * @param inverseStiffness The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm (likely want this for collisions)
   */
  StaticPlaneConstraint(Eigen::Vector3d &normal, double restDistance,
                        Eigen::Index indexU, double inverseStiffness)
      : Constraint({indexU}, inverseStiffness), m_normal(normal),
        m_restDistance(restDistance) {}

  /**
   * @brief Evaluate the static plane constraint. This will return n^T * x -
   * d_rest
   * @param x Matrix of the current positions
   */
  double operator()(const Eigen::MatrixX3d &x) const override;

  /**
   * @brief Evaluate the static plane constraint gradient with respect to the
   * positions. This will return n for the vertex u.
   * @param x Matrix of the current positions
   */
  Eigen::MatrixX3d grad(const Eigen::MatrixX3d &x) const override;

private:
  Eigen::Vector3d m_normal; /// The normal vector n of the plane
  double m_restDistance;    /// The rest distance d_rest
};