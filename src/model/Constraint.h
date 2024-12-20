/**
 * @file Constraint.h
 * @brief Defines the abstract Constraint class which is a generic description
 * of a constraint used for XPBD. Moreover, it declares the constraints that
 * were implemented.
 */
#pragma once

#include <Eigen/Core>
#include <numeric>

/**
 * @enum EConstraintType
 * @brief Utility for identifying constraints when searching for them in the
 * constraint pool of the simulation model
 */
enum EConstraintType {
  EDistance,
  EStaticPlaneCollision,
  EPlaneFriction,
  EShellVolume,
  ETetVolume,
};

/// Update strategies for the solver, defines what values should be used in the
/// update step. Either directly computes deltas on the position or compute
/// constraint values and gradients with respect to the positions. The former is
/// currently unused but still kept in case it might be useful in the future for
/// some other constraints.
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
  const Eigen::MatrixX3d &Xp;

  /// Current position candidates
  const Eigen::MatrixX3d &X;

  // Optionally extend this query record to support other constraint types that
  // might rely on other state variables of the simulation
  //  /// Vertex masses
  //  const Eigen::VectorXd &m;
  //
  //  /// Vertex inverse masses
  //  const Eigen::VectorXd &w;

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
  ConstraintQueryRecord(const Eigen::MatrixX3d &Xp, const Eigen::MatrixX3d &X)
      : Xp(Xp), X(X) {}

  /// Update query record with the constraint value and the gradients with
  /// respect to the positions
  void updateValGrad(double &C, Eigen::MatrixX3d &dCdx) {
    strategy = EValGrad;
    this->C = C;
    this->dCdx = dCdx;
  }

  /// Update query record with the direct position deltas
  void updateDelta(Eigen::MatrixX3d &dx) {
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
  bool m_isActive = true; /// Status of the constraint: is it enabled or not

public:
  /**
   * @brief Evaluate the constraint and its gradient with respect to the
   * relevant vertices, or compute the position deltas directly.
   * @param cRec A Constraint Query Record
   * @return The constraint value C and the gradient dC/dx or the deltaX will be
   * written to the Constraint Query Record
   */
  virtual void solve(ConstraintQueryRecord &cRec) const = 0;

  /**
   * @brief Get the indices of the relevant vertices for this constraint
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
   * @param compliance The new compliance value
   */
  void setCompliance(double compliance) { m_compliance = compliance; }

  /**
   * @brief get the current is_active status
   * @return Whether this constraint is currently active or not
   */
  bool getIsActive() const { return m_isActive; }

  /**
   * @brief Set the constraint status
   * @param active If true, the constraint will be activated, otherwise
   * deactivated
   */
  void setIsActive(bool active) { m_isActive = active; }

  /**
   * @brief Get the constraint alpha
   * @return The constraint compliance divided by deltaTime^2
   */
  double getAlpha(double deltaTime) const {
    return m_compliance / (deltaTime * deltaTime);
  }

  /**
   * @brief Get the constraint type
   * @returns The constraint type
   */
  virtual EConstraintType getType() const = 0;
};

/**
 * @class DistanceConstraint
 * @brief Distance constraint between two vertices
 */
class DistanceConstraint : public Constraint {
public:
  /**
   * @brief Create a new Distance Constraint.
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   * @param X0 Matrix of initial positions
   * @param index1 Index of the first vertex
   * @param index2 Index of the second vertex
   */
  DistanceConstraint(double compliance, Eigen::MatrixX3d &X0,
                     Eigen::Index index1, Eigen::Index index2);

  /**
   * @brief Solve the Distance Constraint.
   * @param cRec A Constraint Query Record
   */
  void solve(ConstraintQueryRecord &cRec) const override;

  /**
   * @brief Get the constraint type
   * @returns EDistance
   */
  EConstraintType getType() const override { return EDistance; }

private:
  double m_distance; /// Initial distance d0, the constraint will try to match
                     /// this distance
};

/**
 * @class StaticPlaneConstraint
 * @brief Collision constraint between a vertex and a plane given by its normal
 * vector
 */
class StaticPlaneCollisionConstraint : public Constraint {
public:
  /**
   * @brief Create a new Static Plane Constraint.
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm (likely want this for collisions)
   * @param normal The normal vector of the plane
   * @param restDistance The distance from the plane when the object is at rest
   * (i.e. if the constraint is satisfied)
   * @param index1 Index of the vertex
   */
  StaticPlaneCollisionConstraint(double compliance, Eigen::Vector3d &normal,
                                 double restDistance, Eigen::Index index1)
      : Constraint(compliance, {index1}), m_normal(normal),
        m_restDistance(restDistance) {}

  /**
   * @brief Solve the Static Plane Collision Constraint.
   * @param cRec A Constraint Query Record
   */
  void solve(ConstraintQueryRecord &cRec) const override;

  /**
   * @brief Get the constraint type
   * @returns EStaticPlaneCollision
   */
  EConstraintType getType() const override { return EStaticPlaneCollision; }

private:
  Eigen::Vector3d m_normal; /// The normal vector n of the plane
  double m_restDistance;    /// The rest distance d_rest
};

/**
 * @class PlaneFrictionConstraint
 * @brief Plane friction constraint which models static and kinetic friction
 * between a vertex and a plane according to the Coulomb friction model.
 */
class PlaneFrictionConstraint : public Constraint {
public:
  /**
   * @brief Create a new Plane Friction Constraint.
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm (likely want this for collisions)
   * @param normal The normal vector of the plane
   * @param staticMu The static friction coefficient
   * @param kineticMu The kinetic friction coefficient
   * @param index1 Index of the vertex
   */
  PlaneFrictionConstraint(double compliance, Eigen::Vector3d &normal,
                          double penetrationDepth, double staticMu,
                          double kineticMu, Eigen::Index index1)
      : Constraint(compliance, {index1}), m_normal(normal),
        m_penetrationDepth(penetrationDepth), m_staticMu(staticMu),
        m_kineticMu(kineticMu) {}

  /**
   * @brief Solve the Plane Friction Constraint.
   * @param cRec A Constraint Query Record
   */
  void solve(ConstraintQueryRecord &cRec) const override;

  /**
   * @brief Get the constraint type
   * @returns EPlaneFriction
   */
  EConstraintType getType() const override { return EPlaneFriction; }

private:
  Eigen::Vector3d &m_normal;
  double m_staticMu;
  double m_kineticMu;
  double m_penetrationDepth;
};

/**
 * @class ShellVolumeConstraint
 * @brief Constraint to enforce volume preservation for non-tet-meshes
 */
class ShellVolumeConstraint : public Constraint {
public:
  /**
   * @brief Construct a new Shell Volume Constraint
   * @param compliance The inverse of the stiffness of the constraint. 0
   * means a perfectly stiff constraint and corresponds to the behavior in the
   * regular PBD algorithm
   * @param x0 Matrix of initial positions
   * @param triangles Triangle index matrix
   * @param start Index of first vertex in simulation domain of the volume
   * @param length Number of vertices for this volume
   * @param pressure The constraint pressure value
   */
  ShellVolumeConstraint(double compliance, Eigen::MatrixX3d &x0,
                        Eigen::MatrixX3i &triangles, Eigen::Index start,
                        Eigen::Index length, double pressure);

  /**
   * @brief Evaluate the volume constraint.
   * @param cRec A Constraint Query Record
   */
  void solve(ConstraintQueryRecord &cRec) const override;

  /**
   * @brief Calcualte the volume of a triangles
   * @param x Vertex positions
   * @return double, the volume of the mesh
   */
  double calculateVolume(const Eigen::MatrixX3d &x) const;

  /**
   * @brief Get the constraint type
   * @returns EShellVolume
   */
  EConstraintType getType() const override { return EShellVolume; }

  /**
   * @brief get the current pressure value
   * @return the current pressure value
   */
  double getPressure() const { return m_pressure; }

  /**
   * @brief update the pressure value
   * @param pressure the new value the pressure gets updated to
   */
  void setPressure(double pressure) { m_pressure = pressure; }

private:
  Eigen::MatrixX3i m_triangles; /// Vertex indices of the triangles
  double m_initVolume; /// Initial volume, the constraint will try to match
  double m_pressure;   /// Pressure factor in front of the initial volume
};

/**
 * @class TetVolumeConstraint
 * @brief Constraint to enforce volume preservation for tet-meshes
 */
class TetVolumeConstraint : public Constraint {
public:
  TetVolumeConstraint(double compliance, Eigen::MatrixX3d &x0, double pressure,
                      Eigen::Index p0, Eigen::Index p1, Eigen::Index p2,
                      Eigen::Index p3);

  /**
   * @brief Evaluate the volume constraint.
   * @param cRec A Constraint Query Record
   */
  void solve(ConstraintQueryRecord &cRec) const override;

  /**
   * @brief Calculate the volume of a tet
   * @param x Vertex positions
   * @return double, the volume of the mesh
   */
  double calculateVolume(const Eigen::MatrixX3d &x) const;

  /**
   * @brief Get the constraint type
   * @returns ETetVolume
   */
  EConstraintType getType() const override { return ETetVolume; }

  /**
   * @brief get the current pressure value
   * @return the current pressure value
   */
  double getPressure() const { return m_pressure; }

  /**
   * @brief update the pressure value
   * @param pressure the new value the pressure gets updated to
   */
  void setPressure(double pressure) { m_pressure = pressure; }

private:
  double m_initVolume;     ///< Initial volume of each tet
  double m_pressure = 1.f; ///< pressure of the tets
};
