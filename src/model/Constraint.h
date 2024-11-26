/**
 * @file Constraint.h
 * @brief Defines the abstract Constraint class which is a generic description
 * of a constraint used for XPBD.
 */
#pragma once

#include <Eigen/Core>
#include <numeric>

/// Update strategies for the solver, defines what values should be used in the
/// update step
enum EUpdateStrategy {
  EDelta,
  EValGrad,
};

/**
 * @struct ConstraintQueryRecord
 * @brief Convenience data structure used to pass multiple parameters to the
 * constraint evaluation functions
 */
struct ConstraintQueryRecord {
  /// Positions at the beginning of the solver
  Eigen::MatrixX3d &x0;

  /// Current position candidates
  Eigen::MatrixX3d &x;

  /// Update strategy
  EUpdateStrategy strategy;

  /// Constraint Value
  double C;

  /// Constraint Gradient with respect to the positions
  Eigen::MatrixX3d dCdx;

  /// Direct position delta
  Eigen::MatrixX3d dx;

public:
  /// Create a new Constraint Query Record
  ConstraintQueryRecord(Eigen::MatrixX3d &x0, Eigen::MatrixX3d &x)
      : x0(x0), x(x) {}

  /// Update query record with the constraint value and the gradients with
  /// respect to the positions
  void updateValGrad(double C, Eigen::MatrixX3d dCdx) {
    strategy = EValGrad;
    this->C = C;
    this->dCdx = dCdx;
  }

  /// Update query record with the direct position deltas
  void updateDelta(Eigen::MatrixX3d dx) {
    strategy = EDelta;
    this->dx = dx;
  }
};

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
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   * @param indices List of indices of the affected vertices
   */
  explicit Constraint(double compliance, std::vector<Eigen::Index> indices)
      : m_compliance(compliance), m_indices(indices) {}

  /**
   * @brief Create a new Constraint.
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   * @param start Start index in the vertex list
   * @param length Number of vertices coming after the start index
   */
  explicit Constraint(double compliance, Eigen::Index start,
                      Eigen::Index length)
      : m_compliance(compliance) {
    m_indices = std::vector<Eigen::Index>(length);
    std::iota(m_indices.begin(), m_indices.end(), start);
  }

  std::vector<Eigen::Index> m_indices; /// Indices of the relevant vertices
  double
      m_compliance; /// Constraint compliance, corresponds to inverse stiffness

public:
  /**
   * @brief Evaluate the constraint and its gradient with respect to the
   * relevant vertices
   * @param cRec A Constraint Query Record
   * @return The constraint value C and the gradient dC/dx will be written to
   * the Constraint Query Record
   */
  virtual void operator()(ConstraintQueryRecord &cRec) const = 0;

  /**
   * @brief Get the indices of the relevant vertices
   * @return A list of indices
   */
  std::vector<Eigen::Index> getIndices() const { return m_indices; }

  /**
   * @brief Get the compliance value
   * @return The constraint compliance
   */
  double getCompliance() const { return m_compliance; }

  /**
   * @brief Set the compliance value
   */
  void setCompliance(double compliance) { m_compliance = compliance; }

  /**
   * @brief Get the constraint alpha
   * @return The constraint compliance divided by deltaTime^2
   */
  double getAlpha(double deltaTime) const {
    return m_compliance / (deltaTime * deltaTime);
  }
};

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
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   */
  DistanceConstraint(double compliance, Eigen::MatrixX3d &x0,
                     Eigen::Index indexU, Eigen::Index indexV);

  /**
   * @brief Evaluate the distance constraint. The value is computed as C = |u -
   * v| - d0 and the gradients are dC/du = n and dC/dv = -n. Where n = (u - v) /
   * |u - v|
   * @param cRec A Constraint Query Record
   */
  void operator()(ConstraintQueryRecord &cRec) const override;

private:
  double m_distance; /// Initial distance d0, the constraint will try to match
                     /// this distance
};

/**
 * @class StaticPlaneConstraint
 * @brief Collision constraint between a vertex and a plane given by its normal
 * vector
 */
class StaticPlaneConstraint : public Constraint {
public:
  /**
   * @brief Create a new Static Plane Constraint.
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm (likely want this for collisions)
   * @param normal The normal vector of the plane
   * @param restDistance The distance from the plane when the object is at rest
   * (i.e. if the constraint is satisfied)
   * @param indexU Index of the vertex
   */
  StaticPlaneConstraint(double compliance, Eigen::Vector3d &normal,
                        double restDistance, Eigen::Index indexU)
      : Constraint(compliance, {indexU}), m_normal(normal),
        m_restDistance(restDistance) {}

  /**
   * @brief Evaluate the static plane constraint. This will return n^T * x -
   * d_rest and the gradient is dC/du = n.
   * @param cRec A Constraint Query Record
   */
  void operator()(ConstraintQueryRecord &cRec) const override;

private:
  Eigen::Vector3d m_normal; /// The normal vector n of the plane
  double m_restDistance;    /// The rest distance d_rest
};