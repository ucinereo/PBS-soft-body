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
#include <mutex>
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
   * @brief get the compliance value of some constraint type
   */
  double getCompliance(EConstraintType cType) const {
    for (Constraint *c : m_constraints) {
      if (c->getType() == cType)
        return c->getCompliance();
    }
    return 0;
  }

  /**
   * @brief set the new compliance value to each constraint of some type
   * @param compliance new value of the constraint compliance
   */
  void setCompliance(EConstraintType cType, double compliance) {
    for (Constraint *c : m_constraints) {
      if (c->getType() == cType)
        c->setCompliance(std::max(compliance, 1e-9)); // Avoid division by zero
    }
  }

  /**
   * @brief get the pressure value
   */
  double getPressureValue() {
    for (Constraint *c : m_constraints) {
      if (c->getType() == ETetVolume)
        return ((TetVolumeConstraint *)c)->getPressure();
    }
    return 0;
  };

  /**
   * @brief set the pressure value to the new pressure
   * @param pressure new value of pressure
   */
  void setPressureValue(double pressure) {
    for (Constraint *c : m_constraints) {
      if (c->getType() == ETetVolume)
        ((TetVolumeConstraint *)c)->setPressure(pressure);
    }
  }

  /**
   * @brief set the constraint to active/inactive
   */
  void setActive(EConstraintType cType, bool state) {
    for (Constraint *c : m_constraints) {
      if (c->getType() == cType)
        c->setIsActive(state);
    }
  }

  /**
   * @brief Get the mutex lock of the render thread
   * @return std::mutex Render lock
   */
  std::mutex *getLock() { return &modelLock; }

  /**
   * @brief reset all the object models and time to the initial position
   */
  void reset();

  /**
   * @brief exports the object mesh
   */
  void exportMesh();

  /**
   * @brief Update the physical model state according to the given timestep.
   * @param deltaTime Timestep size
   */
  void update(double deltaTime); // Update the simulation state

  /**
   * @brief Get references to all static objects of the model
   * @return std::vector<Mesh>& References of static objects
   */
  std::vector<Mesh> &getStatics() { return staticObjs; }
  /**
   * @brief Get references to all dynamic objects of the model
   * @return std::vector<Mesh>& References of dynamic objects
   */
  std::vector<Mesh> &getDynamics() { return dynamicObjs; }

private:
  /**
   * @brief Initialize the basic rendering scene with static and dynamic
   * objects.
   */
  void initialize();

  void
  getStaticCollConstraints(const Eigen::MatrixX3d &Xp,
                           const Eigen::MatrixX3d &X,
                           std::vector<Constraint *> &collConstraints) const;

  //  void
  //  getDynamicCollConstraints(const Eigen::MatrixX3d &Xp,
  //                            const Eigen::MatrixX3d &X,
  //                            std::vector<Constraint *> &collConstraints)
  //                            const;

  std::mutex modelLock; ///< Lock which is necessary to avoid race condition

  std::vector<Mesh> staticObjs;  ///< Storage of static scene objects
  std::vector<Mesh> dynamicObjs; ///< Storage of dynamic scene objects
  std::vector<double> m_slacks;
  double m_staticMu = 0.000001;
  double m_kineticMu = 0.00001;

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
  const int m_solverIterations = 10; /// Max number of iterations
  const double m_solverResidual =
      1e-9; /// Max residual (sum of position delta norms)
  double m_time = 0;
};