/**
 * @file SimulationModel.cpp
 * @brief Definitions of the SimulationModel
 */

#include "SimulationModel.h"
#include "../utils.h"
#include <Eigen/Core>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/upsample.h>
#include <iostream>

SimulationModel::SimulationModel() { initialize(); }

void SimulationModel::initialize() {
  // @TODO: Update this initialization with a scene parsing or something

  // Load the mesh information of the cube
  // - V is a matrix of shape (N x 3) to store vertex positions
  // - F is a matrix of shape (N x 3) to store the face indices
  Eigen::MatrixX3d V;
  Eigen::MatrixX3i F;
  //  igl::readOFF("../cube.off", V, F);
  igl::readOBJ("../assets/cube_1x.obj", V, F);

  igl::upsample(V, F, 1);
  std::cout << "V: " << V.rows() << " " << V.cols() << std::endl;

  // At the beginning the model is flipped by 90 degrees, thus rotate back.
  Eigen::Matrix3d Rx, Ry, Rz;
  Rx = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX());
  Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  Rz = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d R = Rx * Ry * Rz;
  V = (R * V.transpose()).transpose();
  //  V *= 0.1; // Scale the bunny for better scene representation
  for (int i = 0; i < V.rows(); i++) {
    V(i, 1) += 5;
  }

  //  // Load the mesh information of the bunny
  //  // - V is a matrix of shape (N x 3) to store vertex positions
  //  // - F is a matrix of shape (N x 3) to store the face indices
  //  Eigen::MatrixXd V;
  //  Eigen::MatrixXi F;
  //  igl::readOFF("../bunny.off", V, F);
  //
  //  // At the beginning the model is flipped by 90 degrees, thus rotate back.
  //  Eigen::Matrix3d rotationMatrix;
  //  rotationMatrix = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
  //  V = (rotationMatrix * V.transpose()).transpose();
  //  V *= 0.1; // Scale the bunny for better scene representation

  // Create the actual bunny object
  Mesh bunny(V, F);
  bunny.updateColor(1.f, 0.77f, 0.82f, 1.0f);
  dynamicObjs.push_back(bunny); // Add the object to the dynamics.

  // Create a second bunny which is in the x-direction
  //  Eigen::Vector3d pos;
  //  pos << 18.f, 0.f, 0.f;
  //  Eigen::MatrixXd V2 = V.rowwise() + pos.transpose();
  //  Mesh bunny2(V2, F);
  //  bunny2.updateColor(1.f, 0.77f, 0.82f, 1.0f);
  //  dynamicObjs.push_back(bunny2);

  // Initialize a basic floor mesh as static
  Eigen::MatrixX3d floorV;
  Eigen::MatrixX3i floorF;
  createFloorMesh(floorV, floorF);
  staticObjs.push_back(Mesh(floorV, floorF));

  // Compute indices in the per vertex state matrices for each object
  Eigen::Index totalNumVertices = 0;
  for (Mesh &mesh : dynamicObjs) {
    Eigen::Index start = totalNumVertices;
    Eigen::Index length = mesh.numVertices();
    m_indices.push_back(std::pair<Eigen::Index, Eigen::Index>(start, length));
    totalNumVertices += length;
  }

  // Initialize position state matrix
  m_positions = Eigen::MatrixX3d::Zero(totalNumVertices, 3);
  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    m_positions.block(start, 0, length, 3) = dynamicObjs[i].getVertices();
  }

  // Initialize velocity state matrix with zeros
  m_velocities = Eigen::MatrixX3d::Zero(m_positions.rows(), m_positions.cols());

  // Diagonal of the mass matrix, currently 1 for all vertices
  m_mass = Eigen::VectorXd::Ones(m_positions.rows());

  // Gravity external force
  m_gravity = m_mass.asDiagonal() * Eigen::Vector3d(0, m_gravityAccel, 0)
                                        .transpose()
                                        .replicate(m_positions.rows(), 1);

  // Distance constraints between all pairs of vertices
  double crossDistanceCompliance = 30000;
  for (Eigen::Index i = 0; i < m_positions.rows(); i++) {
    for (Eigen::Index j = i + 1; j < m_positions.rows(); j++) {
      // @TODO: Remove hard-coded inverse stiffness
      auto *c =
          new DistanceConstraint(crossDistanceCompliance, m_positions, i, j);
      m_constraints.push_back(c);
    }
  }

  //   Distance Constraints for all Triangles
  double distanceCompliance = 5000;
  for (Eigen::Index i = 0; i < F.rows(); i++) {
    auto *c0 = new DistanceConstraint(distanceCompliance, m_positions, F(i, 0),
                                      F(i, 1));
    auto *c1 = new DistanceConstraint(distanceCompliance, m_positions, F(i, 0),
                                      F(i, 2));
    auto *c2 = new DistanceConstraint(distanceCompliance, m_positions, F(i, 1),
                                      F(i, 2));

    m_constraints.push_back(c0);
    m_constraints.push_back(c1);
    m_constraints.push_back(c2);
  }
}

void SimulationModel::update(double deltaTime) {
  // Update simulation state using positional-based dynamics or other physics
  // Implementation follows Algorithm 1 in the XPBD paper:
  // https://matthias-research.github.io/pages/publications/XPBD.pdf

  // Get external forces
  Eigen::MatrixX3d f_ext = m_gravity;

  // @TODO: Remove this, only used for testing
  // Reverse Direction of external force every 3 seconds for 0.5s
  m_time += deltaTime;
  if (m_time >= 3000 && m_time < 3500) {
    f_ext *= -1;
  } else if (m_time >= 3500) {
    m_time = 0;
  }

  // Predict positions
  Eigen::MatrixX3d x = m_positions;
  x += deltaTime * m_velocities;
  x += deltaTime * deltaTime * (m_mass.cwiseInverse()).asDiagonal() * f_ext;

  Eigen::MatrixX3d x0 = x;

  // Collect collision constraints (dynamically, will require more logic once we
  // do collisions between different objects)
  std::vector<Constraint *> collConstraints;

  Eigen::Vector3d floorNormal(0, 1, 0);
  for (Eigen::Index i = 0; i < m_positions.rows(); i++) {
    // Check if the vertex is penetrating the floor, and if so add a static
    // plane collision constraint
    if (m_positions(i, 1) < 0) {
      auto *c = new StaticPlaneConstraint(0.0, floorNormal, 0.0, i);
      collConstraints.push_back(c);
    }
  }

  // Initialize Lagrange multipliers for each constraint
  Eigen::VectorXd l =
      Eigen::VectorXd::Zero(m_constraints.size() + collConstraints.size());

  // Solve Constraints
  double residual;
  int iter = 0;
  do {
    residual = 0;

    for (size_t i = 0; i < m_constraints.size() + collConstraints.size(); i++) {
      Constraint *c;
      if (i < m_constraints.size()) {
        c = m_constraints[i];
      } else {
        c = collConstraints[i - m_constraints.size()];
      }

      auto Minv = m_mass(c->getIndices()).cwiseInverse();

      ConstraintQueryRecord cRec(x0, x);
      (*c)(cRec);

      if (cRec.strategy == EDelta) {
        x(c->getIndices(), Eigen::all) += cRec.dx;
        residual += cRec.dx.rowwise().norm().sum();
      } else if (cRec.strategy == EValGrad) {
        double alpha = c->getAlpha(deltaTime);
        double dCdxTMinvdCdx = cRec.dCdx.rowwise().squaredNorm().dot(Minv);
        double dl = (-cRec.C - alpha * l(i)) / (dCdxTMinvdCdx + alpha);

        Eigen::MatrixX3d dx = Minv.asDiagonal() * cRec.dCdx * dl;
        x(c->getIndices(), Eigen::all) += dx;
        l(i) += dl;

        residual += dx.rowwise().norm().sum();
      }
    }

    iter++;
  } while (residual > m_solverResidual && iter < m_solverIterations);

  // Update Positions and Velocities
  m_velocities = (x - m_positions) / deltaTime;
  m_positions = x;

  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    dynamicObjs[i].updateVertices(m_positions.block(start, 0, length, 3));
  }
}

std::vector<Mesh> &SimulationModel::getStatics() { return staticObjs; }

std::vector<Mesh> &SimulationModel::getDynamics() { return dynamicObjs; }