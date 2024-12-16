/**
 * @file scenes.h
 * @brief Collection of scenes to swap in and out
 */

#pragma once

#include "model/Mesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/readOBJ.h>
#include <igl/upsample.h>

void createDuckyScene(std::vector<Mesh> &dynamicObjs,
                      std::vector<Mesh> &staticObjs,
                      std::vector<double> &slacks) {
  // Create both Ducks
  Eigen::Matrix3d Rx, Ry, Rz;
  Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

  auto T1 = Eigen::Translation3d(Eigen::Vector3d(-5, 20, 0));
  Eigen::Affine3d M1 = T1 * Ry;

  auto T2 = Eigen::Translation3d(Eigen::Vector3d(-5, 30, 0));
  Eigen::Affine3d M2 = T2 * Ry;

  Mesh duck1 = Mesh::createDuck(M1);
  duck1.tetrahedralize();
  duck1.updateColor(0.94509804, 0.76862745, 0.05882353);
  dynamicObjs.push_back(duck1); // Add the object to the dynamics.

  Mesh duck2 = Mesh::createDuck(M2);
  duck2.tetrahedralize();
  duck2.updateColor(0.94509804, 0.76862745, 0.65882353);
  dynamicObjs.push_back(duck2); // Add the object to the dynamics.

  // Create static cuboid
  Rx = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX());
  Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  Rz = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());

  auto T3 = Eigen::Translation3d(Eigen::Vector3d(-5, 5, 3));
  auto R3 = Rz * Rx * Ry;
  auto S3 = Eigen::Scaling(Eigen::Vector3d(1, 3, 3));
  Eigen::Affine3d M3 = T3 * R3 * S3;

  Mesh cube1 = Mesh::createCube(M3);
  cube1.updateColor(0.8f, 0.4431372f, 0.180392f);
  staticObjs.push_back(cube1);
  slacks.push_back(1e-1);

  auto T4 = Eigen::Translation3d(Eigen::Vector3d(2, 10, 0));
  auto R4 = Rz.transpose() * Rx * Ry;
  auto S4 = Eigen::Scaling(Eigen::Vector3d(1, 3, 3));
  Eigen::Affine3d M4 = T4 * R4 * S4;

  Mesh cube2 = Mesh::createCube(M4);
  cube2.updateColor(0.4431372f, 0.8f, 0.180392f);
  staticObjs.push_back(cube2);
  slacks.push_back(1e-1);

  auto T5 = Eigen::Translation3d(Eigen::Vector3d(-5, 15, 0));
  auto R5 = Rz * Rx.transpose() * Ry;
  auto S5 = Eigen::Scaling(Eigen::Vector3d(1, 3, 3));
  Eigen::Affine3d M5 = T5 * R5 * S5;

  Mesh cube3 = Mesh::createCube(M5);
  cube3.updateColor(0.4431372f, 0.180392f, 0.8f);
  staticObjs.push_back(cube3);
  slacks.push_back(1e-1);

  // Initialize a basic floor mesh as static
  Mesh floor = Mesh::createFloor();
  floor.updateColor(0.4, 0.4, 0.4);
  staticObjs.push_back(floor);
  slacks.push_back(std::numeric_limits<double>::infinity());
}

void createCubeCollisionScene(std::vector<Mesh> &dynamicObjs,
                              std::vector<Mesh> &staticObjs,
                              std::vector<double> &slacks) {
  //  int number_of_subdivisions = 0;
  //
  //  // Simple Rotation
  //  auto Rx = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX());
  //  auto Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  //  auto Rz = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());
  //
  //  // Initialize Cube 1
  //  auto T1 = Eigen::Translation3d(Eigen::Vector3d(5, 10, 0));
  //  auto R1 = Rx * Ry * Rz;
  //  auto S1 = Eigen::Scaling(Eigen::Vector3d(1, 1, 1));
  //  Eigen::Affine3d M1 = T1 * R1 * S1;
  //
  //  Eigen::MatrixX3d cube1V;
  //  Eigen::MatrixX3i cube1F;
  //  createCube(cube1V, cube1F, M1, number_of_subdivisions);
  //  Mesh cube1(cube1V, cube1F);
  //  cube1.updateColor(0.180392f, 0.8f, 0.4431372f);
  //  dynamicObjs.push_back(cube1);
  //
  //  // Initialize Cube 2
  //  auto T2 = Eigen::Translation3d(Eigen::Vector3d(-5, 5, 0));
  //  auto R2 = Rz * Rx * Ry;
  //  auto S2 = Eigen::Scaling(Eigen::Vector3d(1, 3, 3));
  //  Eigen::Affine3d M2 = T2 * R2 * S2;
  //
  //  Eigen::MatrixX3d cube2V;
  //  Eigen::MatrixX3i cube2F;
  //  createCube(cube2V, cube2F, M2, 0);
  //  Mesh cube2(cube2V, cube2F);
  //  cube2.updateColor(0.8f, 0.4431372f, 0.180392f);
  //  staticObjs.push_back(cube2);
  //  slacks.push_back(1e-1);
}