/**
 * @file Distance.h
 * @brief Defines the distance constraint used in XPBD. This constraint limits
 * the distance between two vertices.
 */
#pragma once

#include "../Constraint.h"
#include <Eigen/Core>

/**
 * @class DistanceConstraint
 * @brief Distance constraint between two vertices
 */
class DistanceConstraint : public Constraint {
public:
  /**
   * @brief Create a new Distance Constraint.
   * @param x0 Matrix of initial positions
   * @param indexU Index of the first vertex
   * @param indexV Index of the second vertex
   * @param inverseStiffness The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   */
  DistanceConstraint(Eigen::MatrixX3d &x0, Eigen::Index indexU,
                     Eigen::Index indexV, double inverseStiffness);

  /**
   * @brief Evaluate the distance constraint. This will return |u - v| - d0.
   * @param x Matrix of the current positions
   */
  double operator()(const Eigen::MatrixX3d &x) const override;

  /**
   * @brief Evaluate the distance constraint gradient with respect to the
   * positions. This will return n for the vertex u and -n for the vertex v,
   * where n is the normalized vector between u and v.
   * @param x Matrix of the current positions
   */
  Eigen::MatrixX3d grad(const Eigen::MatrixX3d &x) const override;

private:
  double m_distance; /// Initial distance d0, the constraint will try to match
                     /// this distance
};