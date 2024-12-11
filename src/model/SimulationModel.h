/**
 * @file SimulationModel.h
 * @brief Defines the SimulationModel class which is responsible for the
 * physical simulation itself. It also provides an interface to access the data.
 */
#pragma once
#include "../view/RendereableMesh.h"
#include "Constraint.h"
#include "Mesh.h"
#include <Eigen/Core>
#include <vector>

/**
 * @class SimulationModel
 * @brief The physical model class of XPBD. Provides an interface to the current
 * state of the meshes, as well as an interface to update the physical model for
 * some time.
 *
 * The data is split up into two parts:
 * - Static Data:  This data is static and should not move over time. An example
 * for this is the floor. In the future, it should also be the colliders
 * - Dynamic Data: This data is the data which should be updates each time step
 * and should behave just like XPBD definition according to the constraints.
 */
class SimulationModel {
public:
  /**
   * @brief Construct a new SimulationModel object (should be singleton) and
   * calls initialize() to initialize the scene.
   */
  SimulationModel();

  /**
   * @brief Update the physical model state according to the given timestep.
   * @param deltaTime Timestep size
   */
  void update(double deltaTime); // Update the simulation state

  /**
   * @brief Get references to all static objects of the model
   * @return std::vector<Mesh>& References of static objects
   */
  std::vector<Mesh> &getStatics(); // Get positions of particles/objects
  /**
   * @brief Get references to all dynamic objects of the model
   * @return std::vector<Mesh>& References of dynamic objects
   */
  std::vector<Mesh> &getDynamics(); // Get topology information (faces)

private:
  /**
   * @brief Initialize the basic rendering scene with static and dynamic
   * objects.
   */
  void initialize();

  void
  getStaticCollConstraints(const Eigen::MatrixX3d &x,
                           std::vector<Constraint *> &collConstraints) const;

  void
  getDynamicCollConstraints(const Eigen::MatrixX3d &x,
                            std::vector<Constraint *> &collConstraints) const;

  std::vector<Mesh> staticObjs;  ///< Storage of static scene objects
  std::vector<Mesh> dynamicObjs; ///< Storage of dynamic scene objects

  /// Simulation State
  std::vector<std::pair<Eigen::Index, Eigen::Index>>
      m_indices; /// Indexing into the full vertex matrix for each object/mesh
  Eigen::VectorXd m_mass;        /// (N x 1)
  Eigen::VectorXd m_massInv;     /// (N x 1)
  Eigen::MatrixX3d m_positions;  /// (N x 3)
  Eigen::MatrixX3d m_velocities; /// (N x 3)
  std::vector<Constraint *> m_constraints;

  /// External Forces (just gravity for now)
  Eigen::MatrixX3d m_gravity; /// (N x 3)
  const double m_gravityAccel =
      -0.00001; // @TODO: change scene scale such that we can put -9.81 here

  /// Solver Arguments
  const int m_solverIterations = 10;    /// Max number of iterations
  const double m_solverResidual = 1e-9; /// Max residual (sum of delta lambda)
  double m_time = 0;
};