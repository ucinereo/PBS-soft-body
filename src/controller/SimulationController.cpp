/**
 * @file SimulationController.cpp
 * @brief Definitions of the SimulationController
 */

#include "SimulationController.h"
#include <chrono>
#include <iostream>
#include <thread>

SimulationController::SimulationController(int FPS)
    : model(), renderer(), guiController(this, renderer.getViewer()) {

  renderer.initialize();
  // Load the dynamic data from the model and register it at the renderer.
  // The setMeshData function connects the render data matrices with the mesh
  // objs.
  auto &dynamics = model.getDynamics();
  renderer.registerDynamics(dynamics);
  renderer.setMeshData(dynamics);

  // Load the static data from the model and register it at the renderer.
  auto &statics = model.getStatics();
  renderer.registerStatics(statics);
  renderer.setMeshData(statics);

  // Add the registered mesh data to the libigl's viewer.
  renderer.registerToLibigl();

  //
  simulationSpeed = std::round(1000 / FPS);
  simulationThread =
      new std::thread(&SimulationController::runSimulationThread, this);
  renderer.getViewer().launch_rendering(true);
}

float SimulationController::getTimeStep()
{
  return this->timeStep;
}

void SimulationController::setTimeStep(float timeStep)
{
  this->timeStep = timeStep;
  // TODO: Update whatever is necessary
}

float SimulationController::getStiffness()
{
    return this->stiffness;
}

void SimulationController::setStiffness(float stiffness)
{
  this->stiffness = stiffness;
}

float SimulationController::getPressure()
{
    return this->pressure;
}

void SimulationController::setPressure(float pressure)
{
   this->pressure = pressure;
}

void SimulationController::runSimulation()
{
}

void SimulationController::singleStep()
{
}

void SimulationController::resetSimulation()
{
}


void SimulationController::runSimulationThread()
{
    while (true)
    {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Update the physical simulation model (i.e. do one XPBD step)
        model.update(simulationSpeed);

        // Render the scene (lock render thread if necessary)
        renderer.getLock()->lock();
        // It's only necessary to update the dynamic meshes, as statics don't move
        auto &list = model.getDynamics();
        // This overwrites the libigl's meshes with the new ones.
        renderer.setMeshData(list);
        renderer.getLock()->unlock();

        auto endTime = std::chrono::high_resolution_clock::now();

        auto deltaTime = endTime - startTime;

        // Sleep enough such that we hit the required FPS
        std::chrono::milliseconds sleepTime =
            std::chrono::milliseconds(simulationSpeed) -
            std::chrono::duration_cast<std::chrono::milliseconds>(deltaTime);

        std::this_thread::sleep_for(sleepTime);
    }
}