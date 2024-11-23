/**
 * @file Constraint.h
 * @brief Defines the abstract Constraint class which is a generic description
 * of a constraint used for XPBD.
 */
#pragma once

#include <Eigen/Core>

/**
 * @class Constraint
 * @brief Abstract Constraint class, defines functions for computing the
 * constraint value, gradients of that value with respect to the positions and
 * indices of the affected vertices. Requires passing the inverse stiffness of
 * the constraint.
 */
class Constraint {
protected:
  /**
   * @brief Create a new Constraint.
   * @param indices List of indices of the affected vertices
   * @param inverseStiffness The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   */
  explicit Constraint(std::vector<Eigen::Index> indices,
                      double inverseStiffness)
      : m_indices(indices), m_inverseStiffness(inverseStiffness) {}

  std::vector<Eigen::Index> m_indices;

public:
  /**
   * @brief Evaluate the constraint
   * @return The constraint evaluated at the given positions
   */
  virtual double operator()(const Eigen::MatrixX3d &x) const = 0;

  /**
   * @brief Compute the gradients for the relevant vertices
   * @return A matrix containing the gradient of the constraint value with
   * respect to the positions
   */
  virtual Eigen::MatrixX3d grad(const Eigen::MatrixX3d &x) const = 0;

  /**
   * @brief Get the indices of the relevant vertices
   * @return A list of indices
   */
  std::vector<Eigen::Index> indices() const { return m_indices; };

  double m_inverseStiffness;
};