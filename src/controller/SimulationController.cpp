/**
 * @file SimulationController.cpp
 * @brief Definitions of the SimulationController
 */

#include "SimulationController.h"
#include "../model/Constraint.h"
#include <chrono>
#include <iostream>
#include <thread>

SimulationController::SimulationController(int FPS)
    : m_model(), m_renderer(), m_guiController(this, m_renderer.getViewer()) {

  m_renderer.initialize();
  // Load the dynamic data from the model and register it at the renderer.
  // The setMeshData function connects the render data matrices with the mesh
  // objs.
  auto &dynamics = m_model.getDynamics();
  m_renderer.registerDynamics(dynamics);
  m_renderer.setMeshData(dynamics);

  // Load the static data from the model and register it at the renderer.
  auto &statics = m_model.getStatics();
  m_renderer.registerStatics(statics);
  m_renderer.setMeshData(statics);

  // Add the registered mesh data to the libigl's viewer.
  m_renderer.registerToLibigl();

  //
  m_simulationSpeed = std::round(1000 / FPS);
  m_simulationThread =
      new std::thread(&SimulationController::runSimulationThread, this);
  m_renderer.getViewer().launch_rendering(true);
}

void SimulationController::singleStep() {
  this->m_isSimulationRunning = true;
  m_model.getLock()->lock();
  m_model.update(m_timeStep);
  // Render the scene (lock render thread if necessary)
  m_renderer.getLock()->lock();
  // It's only necessary to update the dynamic meshes, as statics don't move
  auto &list = m_model.getDynamics();
  // This overwrites the libigl's meshes with the new ones.
  m_renderer.setMeshData(list);
  m_renderer.getLock()->unlock();
  m_model.getLock()->unlock();
  this->m_isSimulationRunning = false;
}

void SimulationController::replaceScene(ESceneType sceneType) {
  this->m_isSimulationRunning = false;

  m_model.getLock()->lock();
  m_renderer.getLock()->lock();

  m_model.clear();
  m_renderer.clear();
  m_model.replaceScene(sceneType);

  auto &dynamics = m_model.getDynamics();
  m_renderer.registerDynamics(dynamics);
  m_renderer.setMeshData(dynamics);

  // Load the static data from the model and register it at the renderer.
  auto &statics = m_model.getStatics();
  m_renderer.registerStatics(statics);
  m_renderer.setMeshData(statics);

  // Add the registered mesh data to the libigl's viewer.
  m_renderer.registerToLibigl();

  m_renderer.getLock()->unlock();
  m_model.getLock()->unlock();
}

void SimulationController::resetSimulation() {

  auto startTime = std::chrono::high_resolution_clock::now();

  this->m_isSimulationRunning = false;

  m_model.getLock()->lock();
  // reset the model to the initial positions
  m_model.reset();

  // Render the scene (lock render thread if necessary)
  m_renderer.getLock()->lock();
  // It's only necessary to update the dynamic meshes, as statics don't move
  auto &list = m_model.getDynamics();
  // This overwrites the libigl's meshes with the new ones.
  m_renderer.setMeshData(list);
  m_renderer.getLock()->unlock();

  m_model.getLock()->unlock();

  auto endTime = std::chrono::high_resolution_clock::now();

  auto deltaTime = endTime - startTime;

  // Sleep enough such that we hit the required FPS
  std::chrono::milliseconds sleepTime =
      std::chrono::milliseconds(m_simulationSpeed) -
      std::chrono::duration_cast<std::chrono::milliseconds>(deltaTime);

  std::this_thread::sleep_for(sleepTime);
}

void SimulationController::stopSimulation() {
  this->m_isSimulationRunning = false;
}

void SimulationController::startSimulation() {
  this->m_isSimulationRunning = true;
  m_simulationThread =
      new std::thread(&SimulationController::runSimulationThread, this);
}

void SimulationController::exportObj() {
  this->m_isSimulationRunning = false;
  m_model.exportMesh();
}

bool SimulationController::getIsSimulationRunning() {
  return this->m_isSimulationRunning;
}

void SimulationController::runSimulationThread() {
  while (m_isSimulationRunning) {
    auto startTime = std::chrono::high_resolution_clock::now();

    // Update the physical simulation model (i.e. do one XPBD step)
    // model.update(simulationSpeed);
    m_model.getLock()->lock();
    m_model.update(m_timeStep);

    // Render the scene (lock render thread if necessary)
    m_renderer.getLock()->lock();
    // It's only necessary to update the dynamic meshes, as statics don't move
    auto &list = m_model.getDynamics();
    // This overwrites the libigl's meshes with the new ones.
    m_renderer.setMeshData(list);
    m_renderer.getLock()->unlock();

    m_model.getLock()->unlock();

    auto endTime = std::chrono::high_resolution_clock::now();

    auto deltaTime = endTime - startTime;

    float newFps =
        1000000.f /
        (float)std::chrono::duration_cast<std::chrono::microseconds>(deltaTime)
            .count();
    float alpha = 0.05f;
    if (m_fps == 0) {
      m_fps = newFps;
    } else {
      m_fps = alpha * newFps + (1 - alpha) * m_fps;
    }

    // Sleep enough such that we hit the required FPS
    std::chrono::milliseconds sleepTime =
        std::chrono::milliseconds(m_simulationSpeed) -
        std::chrono::duration_cast<std::chrono::milliseconds>(deltaTime);

    std::this_thread::sleep_for(sleepTime);
  }
}