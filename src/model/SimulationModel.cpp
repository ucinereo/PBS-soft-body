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

  int number_of_subdivisions = 0;

  // Simple Rotation
  auto Rx = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX());
  auto Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  auto Rz = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());

  // Initialize Cube 1
  auto T1 = Eigen::Translation3d(Eigen::Vector3d(3, 5, 0));
  auto R1 = Rx * Ry * Rz;
  auto S1 = Eigen::Scaling(Eigen::Vector3d(1, 1, 1));
  Eigen::Affine3d M1 = T1 * R1 * S1;

  Eigen::MatrixX3d cube1V;
  Eigen::MatrixX3i cube1F;
  createCube(cube1V, cube1F, M1, number_of_subdivisions);
  Mesh cube1(cube1V, cube1F);
  cube1.updateColor(0.180392f, 0.8f, 0.4431372f);
  dynamicObjs.push_back(cube1);

  // Initialize Cube 2
  auto T2 = Eigen::Translation3d(Eigen::Vector3d(-3, 5, 0));
  auto R2 = Rx * Ry;
  auto S2 = Eigen::Scaling(Eigen::Vector3d(1, 1, 1));
  Eigen::Affine3d M2 = T2 * R2 * S2;

  Eigen::MatrixX3d cube2V;
  Eigen::MatrixX3i cube2F;
  createCube(cube2V, cube2F, M2, number_of_subdivisions);
  Mesh cube2(cube2V, cube2F);
  cube2.updateColor(0.8f, 0.4431372f, 0.180392f);
  dynamicObjs.push_back(cube2);

  // Initialize a basic floor mesh as static
  Eigen::MatrixX3d floorV;
  Eigen::MatrixX3i floorF;
  createFloorMesh(floorV, floorF);
  Mesh floor = Mesh(floorV, floorF);
  floor.updateColor(0.4, 0.4, 0.4);
  staticObjs.push_back(floor);

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
  m_massInv = m_mass.cwiseInverse();

  // Gravity external force
  m_gravity = m_mass.asDiagonal() * Eigen::Vector3d(0, m_gravityAccel, 0)
                                        .transpose()
                                        .replicate(m_positions.rows(), 1);

  //   Distance Constraints for all Triangles
  double distanceCompliance = 1; // 1e-9;
  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    const Mesh &obj = dynamicObjs[i];
    Eigen::MatrixX3d V = obj.getVertices();
    Eigen::MatrixX3i F = obj.getFaces();

    Eigen::Index start = m_indices[i].first;

    for (Eigen::Index j = 0; j < F.rows(); j++) {
      auto *c0 = new DistanceConstraint(distanceCompliance, m_positions,
                                        start + F(j, 0), start + F(j, 1));
      auto *c1 = new DistanceConstraint(distanceCompliance, m_positions,
                                        start + F(j, 0), start + F(j, 2));
      auto *c2 = new DistanceConstraint(distanceCompliance, m_positions,
                                        start + F(j, 1), start + F(j, 2));

      m_constraints.push_back(c0);
      m_constraints.push_back(c1);
      m_constraints.push_back(c2);
    }
  }

  // Volume constraint
  double volumeCompliance = 0.1;
  double pressure = 1;
  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    const Mesh &obj = dynamicObjs[i];
    Eigen::MatrixX3i F = obj.getFaces();

    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    std::cout << start << std::endl;
    auto *cV = new ShellVolumeConstraint(volumeCompliance, m_positions, F,
                                         start, length, pressure);
    m_constraints.push_back(cV);
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
  if (m_time >= 10000 && m_time < 10250) {
    f_ext *= -2;
    //    f_ext += Eigen::MatrixX3d::Random(f_ext.rows(), f_ext.cols()) * 0.005;
    f_ext(Eigen::all, 0) += Eigen::RowVectorXd::Ones(f_ext.rows()) * 0.000035;
  } else if (m_time >= 10250) {
    m_time = 0;
  }

  // Predict positions
  Eigen::MatrixX3d x = m_positions;
  x += deltaTime * m_velocities;
  x += deltaTime * deltaTime * m_massInv.asDiagonal() * f_ext;

  // Collect collision constraints (dynamically, will require more logic once we
  // do collisions between different objects)
  std::vector<Constraint *> collConstraints;

  Eigen::Vector3d floorNormal(0, 1, 0);
  double staticMu = 0.9;
  double kineticMu = 0.9;
  double collisionCompliance = 1e-9;
  double frictionCompliance = 1e-9;
  for (Eigen::Index i = 0; i < x.rows(); i++) {
    // Check if the vertex is penetrating the floor, and if so add a static
    // plane collision constraint
    double penetrationDepth = floorNormal.dot(x.row(i).transpose());
    if (penetrationDepth < 0) {
      auto *c0 = new StaticPlaneCollisionConstraint(collisionCompliance,
                                                    floorNormal, 0.0, i);
      auto *c1 = new PlaneFrictionConstraint(frictionCompliance, floorNormal,
                                             staticMu, kineticMu, i);

      collConstraints.push_back(c0);
      collConstraints.push_back(c1);
    }
  }

  std::vector<size_t> indices(m_constraints.size() + collConstraints.size());
  std::iota(indices.begin(), indices.end(), 0);

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

  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    dynamicObjs[i].updateVertices(m_positions.block(start, 0, length, 3));
  }
}

std::vector<Mesh> &SimulationModel::getStatics() { return staticObjs; }

std::vector<Mesh> &SimulationModel::getDynamics() { return dynamicObjs; }