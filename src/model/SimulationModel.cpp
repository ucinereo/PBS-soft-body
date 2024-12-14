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
#include <igl/writeOBJ.h>
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

  igl::upsample(V, F, 3);

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

  m_initialpositions = Eigen::MatrixX3d(V);

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
  bunny.updateColor(0.180392f, 0.8f, 0.4431372f);
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
  double distanceCompliance = 10000; // 1e-9;
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

  // Volume constraint
  double volumeCompliance = 0.1;
  double pressure = 1;
  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    std::cout << start << std::endl;
    auto *cV = new ShellVolumeConstraint(volumeCompliance, m_positions, F,
                                         start, length, pressure);
    m_constraints.push_back(cV);
  }
}

double SimulationModel::getComplianceEDistance() {
  double result = 0.0;
  for (Constraint *c : m_constraints) {
    result = c->getCompliance();
    return result;
  }
  return result;
}

void SimulationModel::setComplianceEDistance(double compliance) {
  for (Constraint *c : m_constraints) {
    if (c->getType() == EDistance) {
      c->setCompliance(compliance);
    }
  }
}

double SimulationModel::getComplianceEStaticPlaneCollision() {
  double result = 0.0;
  for (Constraint *c : m_constraints) {
    if (c->getType() == EStaticPlaneCollision) {
      result = c->getCompliance();
      return result;
    }
  }
  return result;
}

void SimulationModel::setComplianceEstaticPlaneCollision(double compliance) {
  for (Constraint *c : m_constraints) {
    if (c->getType() == EStaticPlaneCollision) {
      c->setCompliance(compliance);
    }
  }
}

double SimulationModel::getComplianceEPlaneFriction() {
  double result = 0.0;
  for (Constraint *c : m_constraints) {
    if (c->getType() == EPlaneFriction) {
      result = c->getCompliance();
      return result;
    }
  }
  return result;
}

void SimulationModel::setComplianceEPlaneFriction(double compliance) {
  for (Constraint *c : m_constraints) {
    if (c->getType() == EPlaneFriction) {
      c->setCompliance(compliance);
    }
  }
}

double SimulationModel::getComplianceVolume() {
  double result = 0.0;
  for (Constraint *c : m_constraints) {
    if (c->getType() == EShellVolume) {
      result = c->getCompliance();
      return result;
    }
  }
  return result;
}

void SimulationModel::setComplianceVolume(double compliance) {
  for (Constraint *c : m_constraints) {
    if (c->getType() == EShellVolume) {
      c->setCompliance(compliance);
    }
  }
}

double SimulationModel::getPressureValue() {
  double pressure = 0.0;
  for (Constraint *c : m_constraints) {
    if (c->getType() == EShellVolume) {
      pressure = ((ShellVolumeConstraint *)c)->getPressure();
      return pressure;
    }
  }
  return pressure;
}

void SimulationModel::setPressureValue(double pressure) {
  for (Constraint *c : m_constraints) {
    if (c->getType() == EShellVolume) {
      ((ShellVolumeConstraint *)c)->setPressure(pressure);
    }
  }
}

void SimulationModel::reset() {

  V = m_initialpositions;

  m_indices.clear();
  m_time = 0;

  // Reset positions to the initial state
  m_positions = m_initialpositions;

  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    dynamicObjs[i].updateVertices(m_positions.block(start, 0, length, 3));
  }

  Eigen::Index totalNumVertices = 0;
  for (Mesh &mesh : dynamicObjs) {
    Eigen::Index start = totalNumVertices;
    Eigen::Index length = mesh.numVertices();
    m_indices.push_back(std::pair<Eigen::Index, Eigen::Index>(start, length));
    totalNumVertices += length;
  }

  // Reset velocities to zero
  m_velocities = Eigen::MatrixX3d::Zero(m_positions.rows(), m_positions.cols());

  // Reset forces such as gravity
  m_gravity = m_mass.asDiagonal() * Eigen::Vector3d(0, m_gravityAccel, 0)
                                        .transpose()
                                        .replicate(m_positions.rows(), 1);

  // Reinitialize constraints
  m_constraints.clear(); // Clear the existing constraints

  // Distance Constraints for all Triangles
  double distanceCompliance = 10000; // 1e-9;
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

  // Volume constraint
  double volumeCompliance = 0.1;
  double pressure = 1;
  for (size_t i = 0; i < dynamicObjs.size(); i++) {
    Eigen::Index start, length;
    std::tie(start, length) = m_indices[i];
    std::cout << start << std::endl;
    auto *cV = new ShellVolumeConstraint(volumeCompliance, m_positions, F,
                                         start, length, pressure);
    m_constraints.push_back(cV);
  }

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
    f_ext *= -0.2;
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

std::mutex *SimulationModel::getLock() { return &modelLock; }