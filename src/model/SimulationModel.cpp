/**
 * @file SimulationModel.cpp
 * @brief Definitions of the SimulationModel
 */

#include "SimulationModel.h"
#include "../utils.h"
#include <Eigen/Core>
#include <igl/readOFF.h>

SimulationModel::SimulationModel() { initialize(); }

void SimulationModel::initialize() {
  // @TODO: Update this initialization with a scene parsing or something

  // Load the mesh information of the bunny
  // - V is a matrix of shape (N x 3) to store vertex positions
  // - F is a matrix of shape (N x 3) to store the face indices
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::readOFF("../bunny.off", V, F);

  // At the beginning the model is flipped by 90 degrees, thus rotate back.
  Eigen::Matrix3d rotationMatrix;
  rotationMatrix = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
  V = (rotationMatrix * V.transpose()).transpose();
  V *= 0.1; // Scale the bunny for better scene representation

  // Create the actual bunny object
  Mesh bunny(V, F);
  bunny.updateColor(1.f, 0.77f, 0.82f, 1.0f);
  dynamicObjs.push_back(bunny); // Add the object to the dynamics.

  // Create a second bunny which is in the x-direction
  Eigen::Vector3d pos;
  pos << 18.f, 0.f, 0.f;
  Eigen::MatrixXd V2 = V.rowwise() + pos.transpose();
  Mesh bunny2(V2, F);
  bunny2.updateColor(1.f, 0.77f, 0.82f, 1.0f);
  dynamicObjs.push_back(bunny2);

  // Initialize a basic floor mesh as static
  Eigen::MatrixXd floorV;
  Eigen::MatrixXi floorF;
  createFloorMesh(floorV, floorF);
  staticObjs.push_back(Mesh(floorV, floorF));
}

void SimulationModel::update(double deltaTime) {
  // Update simulation state using positional-based dynamics or other physics
  // @TODO: Add XPBD logic here, currently just rotates the bunnies

  // Setup rotation matrices for the bunnies
  Eigen::Matrix3d rotation1, rotation2;
  rotation1 = Eigen::AngleAxisd(-M_PI / 16, Eigen::Vector3d::UnitY());
  rotation2 = Eigen::AngleAxisd(M_PI / 64, Eigen::Vector3d::UnitY());

  // Update the first bunny
  Mesh &o1 = dynamicObjs[0];
  auto V1 = o1.getVertices();
  o1.updateVertices((rotation1 * V1.transpose()).transpose());

  // Update the second bunny
  Mesh &o2 = dynamicObjs[1];
  auto V2 = o2.getVertices();
  o2.updateVertices((rotation2 * V2.transpose()).transpose());
}

std::vector<Mesh> &SimulationModel::getStatics() { return staticObjs; }

std::vector<Mesh> &SimulationModel::getDynamics() { return dynamicObjs; }