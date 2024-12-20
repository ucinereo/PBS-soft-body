/**
 * @file scenes.h
 * @brief Defines and implements the SceneFactory class which can be used to
 * build a set of scenes
 */

#pragma once

#include "model/Mesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <igl/decimate.h>
#include <igl/readOBJ.h>
#include <igl/upsample.h>

/**
 * @enum ESceneType
 * @brief EPalmTree creates the scene which the two duckies falling onto a palm
 * tree. ECuboidBVHoff and ECuboidBVHon are used to benchmark the BVH
 * implementation, two duckies fall onto three cuboids.
 */
enum ESceneType { EPalmTree, ECuboidBVHoff, ECuboidBVHon };

/**
 * @class SceneFactory
 * @brief Utility class for managing different scenes
 */
class SceneFactory {
public:
  /**
   * @brief Creates a scene according to the given sceneType
   * @param sceneType Scene to be created
   * @param dynamicObjs Will be populated with the dynamic objects in the scene
   * @param staticObjs Will be populated with the static objects in the scene
   * @param slacks Will be populated with the collision slack variables for the
   * static objects in the scene
   */
  static void createScene(ESceneType sceneType, std::vector<Mesh> &dynamicObjs,
                          std::vector<Mesh> &staticObjs,
                          std::vector<double> &slacks) {
    switch (sceneType) {
    case EPalmTree:
      SceneFactory::createPalmTreeScene(dynamicObjs, staticObjs, slacks);
      break;
    case ECuboidBVHoff:
      SceneFactory::createCuboidBVHScene(dynamicObjs, staticObjs, slacks,
                                         false);
      break;
    case ECuboidBVHon:
      SceneFactory::createCuboidBVHScene(dynamicObjs, staticObjs, slacks, true);
      break;
    default:
      throw std::runtime_error("Scene type is unknown!");
    }
  }

private:
  /**
   * @brief Creates the palm tree scene
   * @param dynamicObjs Will be populated with the dynamic objects in the scene
   * @param staticObjs Will be populated with the static objects in the scene
   * @param slacks Will be populated with the collision slack variables for the
   * static objects in the scene
   */
  static void createPalmTreeScene(std::vector<Mesh> &dynamicObjs,
                                  std::vector<Mesh> &staticObjs,
                                  std::vector<double> &slacks) {
    // Create both Ducks
    Eigen::Matrix3d Rx, Ry, Rz;
    Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    auto T1 = Eigen::Translation3d(Eigen::Vector3d(-5, 20, 0));
    Eigen::Affine3d M1 = T1 * Ry;

    auto T2 = Eigen::Translation3d(Eigen::Vector3d(-5, 30, 0));
    Eigen::Affine3d M2 = T2 * Ry;

    Mesh duck1 = Mesh::createOBJ("../assets/rubber_duck.obj", M1);
    duck1.tetrahedralize();
    duck1.updateColor(0.94509804, 0.76862745, 0.05882353);
    dynamicObjs.push_back(duck1); // Add the object to the dynamics.

    Mesh duck2 = Mesh::createOBJ("../assets/rubber_duck.obj", M2);
    duck2.tetrahedralize();
    duck2.updateColor(0.94509804, 0.76862745, 0.65882353);
    dynamicObjs.push_back(duck2); // Add the object to the dynamics.

    // Create Palm Trees
    auto Tpalme = Eigen::Translation3d(Eigen::Vector3d(-5, 0, 0));
    Eigen::Affine3d Mpalme = Tpalme * Eigen::Affine3d::Identity();
    Mesh palme = Mesh::createOBJ("../assets/blaetter2.obj", Mpalme);
    palme.updateColor(0.149, 0.302, 0.047);
    palme.buildBVH(1e-1);
    staticObjs.push_back(palme);
    slacks.push_back(1e-1);

    Mesh stamm = Mesh::createOBJ("../assets/stamm2.obj", Mpalme);
    stamm.updateColor(0.302, 0.192, 0.063);
    stamm.buildBVH(1e-1);
    staticObjs.push_back(stamm);
    slacks.push_back(1e-1);

    Mesh wasser = Mesh::createOBJ("../assets/wasser.obj", Mpalme);
    wasser.updateColor(0.769, 0.588, 0.122);
    wasser.buildBVH(1e-1);
    staticObjs.push_back(wasser);
    slacks.push_back(1e-1);

    // Initialize a basic floor mesh as static
    Mesh floor = Mesh::createFloor();
    floor.updateColor(0.4, 0.4, 0.4);
    floor.updateColor(0.427, 0.741, 0.949);
    staticObjs.push_back(floor);
    slacks.push_back(std::numeric_limits<double>::infinity());
  }

  /**
   * @brief Creates the cuboid scene with control over whether a BVH for each
   * static object should be created. Used for benchmarking the BVH
   * implementation.
   * @param dynamicObjs Will be populated with the dynamic objects in the scene
   * @param staticObjs Will be populated with the static objects in the scene
   * @param slacks Will be populated with the collision slack variables for the
   * static objects in the scene
   */
  static void createCuboidBVHScene(std::vector<Mesh> &dynamicObjs,
                                   std::vector<Mesh> &staticObjs,
                                   std::vector<double> &slacks, bool useBVH) {
    // Create both Ducks
    Eigen::Matrix3d Rx, Ry, Rz;
    Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());

    auto T1 = Eigen::Translation3d(Eigen::Vector3d(-5, 20, 0));
    Eigen::Affine3d M1 = T1 * Ry;

    auto T2 = Eigen::Translation3d(Eigen::Vector3d(-5, 30, 0));
    Eigen::Affine3d M2 = T2 * Ry;

    Mesh duck1 = Mesh::createOBJ("../assets/rubber_duck.obj", M1);
    duck1.tetrahedralize();
    duck1.updateColor(0.94509804, 0.76862745, 0.05882353);
    dynamicObjs.push_back(duck1); // Add the object to the dynamics.

    Mesh duck2 = Mesh::createOBJ("../assets/rubber_duck.obj", M2);
    duck2.tetrahedralize();
    duck2.updateColor(0.94509804, 0.76862745, 0.65882353);
    dynamicObjs.push_back(duck2); // Add the object to the dynamics.

    // Create static cuboids
    Rx = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX());
    Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
    Rz = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());

    auto T3 = Eigen::Translation3d(Eigen::Vector3d(-5, 5, 3));
    auto R3 = Rz * Rx * Ry;
    auto s3 = Eigen::Vector3d(1, 3, 3);
    auto S3 = Eigen::Scaling(s3);
    Eigen::Affine3d M3 = T3 * R3 * S3;

    Mesh cube1 = Mesh::createOBJ("../assets/cube_1x.obj", M3);
    cube1.updateColor(0.8f, 0.4431372f, 0.180392f);
    cube1.upsample(6);
    if (useBVH)
      cube1.buildBVH(1e-1);
    staticObjs.push_back(cube1);
    slacks.push_back(1e-1);

    auto T4 = Eigen::Translation3d(Eigen::Vector3d(2, 10, 0));
    auto R4 = Rz.transpose() * Rx * Ry;
    Eigen::Affine3d M4 = T4 * R4 * S3;

    Mesh cube2 = Mesh::createOBJ("../assets/cube_1x.obj", M4);
    cube2.updateColor(0.4431372f, 0.8f, 0.180392f);
    cube2.upsample(6);
    if (useBVH)
      cube2.buildBVH(1e-1);
    staticObjs.push_back(cube2);
    slacks.push_back(1e-1);

    auto T5 = Eigen::Translation3d(Eigen::Vector3d(-5, 15, 0));
    auto R5 = Rz * Rx.transpose() * Ry;
    Eigen::Affine3d M5 = T5 * R5 * S3;

    Mesh cube3 = Mesh::createOBJ("../assets/cube_1x.obj", M5);
    cube3.updateColor(0.4431372f, 0.180392f, 0.8f);
    cube3.upsample(6);
    if (useBVH)
      cube3.buildBVH(1e-1);
    staticObjs.push_back(cube3);
    slacks.push_back(1e-1);

    // Initialize a basic floor mesh as static
    Mesh floor = Mesh::createFloor();
    floor.updateColor(0.4, 0.4, 0.4);
    staticObjs.push_back(floor);
    slacks.push_back(std::numeric_limits<double>::infinity());
  }
};
