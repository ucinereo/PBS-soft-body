/**
 * @file SimulationModel.cpp
 * @brief Definitions of the SimulationModel
 */

#include "SimulationModel.h"
#include "../scenes.h"
#include <Eigen/Core>
#include <chrono>
#include <igl/writeOBJ.h>
#include <iostream>

SimulationModel::SimulationModel() { initialize(); }

void SimulationModel::initialize() {
  // @TODO: Update this initialization with a scene parsing or something

  createDuckyScene(m_dynamicObjs, m_staticObjs, m_slacks);
  //  createDuckyScene2(m_dynamicObjs, m_staticObjs, m_slacks);

  // Compute indices in the per vertex state matrices for each object
  Eigen::Index totalNumVertices = 0;
  for (Mesh &mesh : m_dynamicObjs) {
    Eigen::Index start = totalNumVertices;
    Eigen::Index length = mesh.numVertices();
    m_indices.push_back(std::pair<Eigen::Index, Eigen::Index>(start, length));
    totalNumVertices += length;
  }

  // Initialize position state matrix
  m_positions = Eigen::MatrixX3d::Zero(totalNumVertices, 3);
  for (size_t i = 0; i < m_dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    m_positions.block(start, 0, length, 3) = m_dynamicObjs[i].getVertices();
  }

  // Initialize velocity state matrix with zeros
  m_velocities = Eigen::MatrixX3d::Zero(m_positions.rows(), m_positions.cols());

  // Diagonal of the mass matrix, currently 1 for all vertices
  m_mass = Eigen::VectorXd::Ones(m_positions.rows());
  m_massInv = m_mass.cwiseInverse();

  // Gravity external force
  m_gravity = m_mass.asDiagonal() * Eigen::Vector3d(0, m_gravityAccel, 0)
                                        .transpose()
                                        .replicate(m_positions.rows(), 1);

  //   Distance Constraints for all Triangles
  double distanceCompliance = 500;
  ConstraintFactory::meshFaceDistance(m_constraints, m_positions, m_dynamicObjs,
                                      m_indices, distanceCompliance);
  //  double distanceCompliance = 10000;
  //  ConstraintFactory::meshTetDistance(m_constraints, m_positions,
  //  m_dynamicObjs,
  //                                  m_indices, distanceCompliance);

  // Tet-Volume-Constraint
  double tetCompliance = 1.0;
  double tetPressure = 1;
  ConstraintFactory::tetVolume(m_constraints, m_positions, m_dynamicObjs,
                               m_indices, tetCompliance, tetPressure);
}

void SimulationModel::reset() {
  // Reset all the positions
  for (size_t i = 0; i < m_dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];

    m_dynamicObjs[i].resetVertices();
    m_positions.block(start, 0, length, 3) = m_dynamicObjs[i].getVertices();
  }

  // Reset the velocities
  m_velocities.setZero();

  std::cout << "Simulation has been reset to its initial state." << std::endl;
}

void SimulationModel::exportMesh() {
  // get the current dynamic objects
  auto &dynamicObjects = this->getDynamics();
  Mesh dynamicMeshs = dynamicObjects[0];
  // get Vertices and faces of mesh
  Eigen::MatrixX3d V_export = dynamicMeshs.getVertices();
  Eigen::MatrixX3i F_export = dynamicMeshs.getFaces();

  // @todo: implement an export Mesh that handles more than one object using a
  // loop
  // @todo: check that implementation is correct
  // @todo: check if static objects also have to be exported
  // int index_V = 0;
  // int index_F = 0;
  // for (Mesh m : dynamicObjects) {
  //   // get current Vertices and Faces
  //   Eigen::MatrixX3d m_V = m.getVertices();
  //   Eigen::MatrixX3i m_F = m.getFaces();

  //   // add Vertices and Faces to the Matrix
  //   const Eigen::Index num_rows_V = m.numVertices();
  //   V_export.middleRows(num_rows_V, index_V) = m_V;

  //   const Eigen::Index num_rows_F = m.numFaces();
  //   F_export.middleRows(num_rows_F, index_F) = m_F;

  //   // update index
  //   index_V += num_rows_V;
  //   index_F += num_rows_F;
  // }
  // auto &staticObjects = this->getStatics();
  // for (Mesh m : staticObjects) {
  //   Eigen::MatrixX3d m_V = m.getVertices();
  //   Eigen::MatrixX3i m_F = m.getFaces();

  //   // add Vertices and Faces to the Matrix
  //   const Eigen::Index num_rows_V = m.numVertices();
  //   V_export.middleRows(num_rows_V, index_V) = m_V;

  //   const Eigen::Index num_rows_F = m.numFaces();
  //   F_export.middleRows(num_rows_F, index_F) = m_F;

  //   //update index
  //   index_V += num_rows_V;
  //   index_F += num_rows_F;
  // }

  // export to output file
  igl::writeOBJ("output.obj", V_export, F_export);

  std::cout << "export object as obj file \n";
}

void SimulationModel::getStaticCollConstraints(
    const Eigen::MatrixX3d &Xp, const Eigen::MatrixX3d &X,
    std::vector<Constraint *> &collConstraints) const {
  // Hardcoded because it makes no sense to change these, we want collisions and
  // friction to be as stiff as possible
  double collisionCompliance = 1e-9;
  double frictionCompliance = 1e-9;

  ConstraintFactory::vertexStaticTriangleInteraction(
      collConstraints, Xp, X, m_staticObjs, m_dynamicObjs, m_indices,
      collisionCompliance, frictionCompliance, m_staticMu, m_kineticMu,
      m_slacks);
}

void SimulationModel::update(double deltaTime) {
  // Update simulation state using positional-based dynamics
  // Implementation follows Algorithm 1 in the XPBD paper:
  // https://matthias-research.github.io/pages/publications/XPBD.pdf

  // Get external forces
  Eigen::MatrixX3d f_ext = m_gravity;

  // @TODO: Remove this, only used for testing
  // Reverse Direction of external force every 3 seconds for 0.5s
  //  m_time += deltaTime;
  //  if (m_time >= 10000 && m_time < 10250) {
  //    f_ext *= -2;
  //    //    f_ext += Eigen::MatrixX3d::Random(f_ext.rows(), f_ext.cols()) *
  //    0.005;
  //    //    f_ext(Eigen::all, 0) += Eigen::RowVectorXd::Ones(f_ext.rows()) *
  //    //    0.000035;
  //  } else if (m_time >= 10250) {
  //    m_time = 0;
  //  }

  // Predict positions
  Eigen::MatrixX3d x = m_positions;
  x += deltaTime * m_velocities;
  x += deltaTime * deltaTime * m_massInv.asDiagonal() * f_ext;

  // Collect collision constraints (dynamically, will require more logic once we
  // do collisions between different objects)
  std::vector<Constraint *> collConstraints;
  this->getStaticCollConstraints(m_positions, x, collConstraints);
  //  this->getDynamicCollConstraints(m_positions, x, collConstraints);

  std::vector<size_t> indices(m_constraints.size() + collConstraints.size());
  std::iota(indices.begin(), indices.end(), 0);
  // Maybe shuffle indices for more stability?

  // Initialize Lagrange multipliers for each constraint
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero((Eigen::Index)indices.size());

  // Solve Constraints
  double residual;
  int iter = 0;
  do {
    residual = 0;

    for (size_t i : indices) {
      Constraint *c;
      if (i < m_constraints.size()) {
        c = m_constraints[i];
      } else {
        c = collConstraints[i - m_constraints.size()];
      }

      // if the constraint is not enabled, skip the update step
      if (!c->getIsActive()) {
        continue;
      }

      Eigen::VectorXd Minv = m_massInv(c->getIndices());
      ConstraintQueryRecord cRec(m_positions, x);
      c->solve(cRec);

      if (cRec.strategy == EDelta) {
        x(c->getIndices(), Eigen::all) += cRec.dx;
        residual += cRec.dx.rowwise().norm().sum();
      } else if (cRec.strategy == EValGrad) {
        double alpha = c->getAlpha(deltaTime);
        double dCdxTMinvdCdx = cRec.dCdx.rowwise().squaredNorm().dot(Minv);
        double dl = (-cRec.C - alpha * lambda(i)) / (dCdxTMinvdCdx + alpha);

        Eigen::MatrixX3d dx = dl * Minv.asDiagonal() * cRec.dCdx;
        x(c->getIndices(), Eigen::all) += dx;
        lambda(i) += dl;

        residual += dx.rowwise().norm().sum();
      }
    }

    iter++;
  } while (residual > m_solverResidual && iter < m_solverIterations);

  // Update Positions and Velocities
  m_velocities = (x - m_positions) / deltaTime;
  m_positions = x;

  for (size_t i = 0; i < m_dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    Eigen::MatrixX3d nextVertices = m_positions.block(start, 0, length, 3);
    m_dynamicObjs[i].updateVertices(nextVertices);
  }
}