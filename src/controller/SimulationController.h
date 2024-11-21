/**
 * @file SimulationController.h
 * @brief Defines the SimulationController class which manages the main
 * simulation loop.
 */
#pragma once

#include "../model/SimulationModel.h"
#include "../view/Renderer.h"
#include "GuiController.h"
#include <thread>

/**
 * @class SimulationController
 * @brief The main controller class that manages the simulation lifecycle,
 * including initialization,the main loop, input handling, updating the
 * simulation and rendering. Coordinates the simulation model with the view
 * (renderer).
 */
class SimulationController {
public:
  /**
   * @brief Construct a new SimulationController object
   * @param FPS Frames per second which should be reached by the simulator.
   */
  SimulationController(int FPS);

  /**
   * @brief Getter for time step size
   * @return Current time step setting
   */
  float getTimeStep();

  /**
   * @brief Set time step size
   * @param timeStep Time step to set
   */
  void setTimeStep(float timeStep);

  /**
   * @brief Getter for stiffness value
   * @return Current stiffness value
   */
  float getStiffness();

  /**
   * @brief Set stiffness parameter
   * @param stiffness stiffness value it gets updated to
   */
  void setStiffness(float stiffness);

  /**
   * @brief Getter for pressure value
   * @return Current pressure value
   */
  float getPressure();

  /**
   * @brief Set pressure value
   * @param pressure poressure value it gets updated to
   */
  void setPressure(float pressure);

  /**
   * @brief starts the simulation
   */
  void runSimulation();

  /**
   * @brief executes a single step of the simulation
   */
  void singleStep();

  /**
   * @brief resets the simulation
   */
  void resetSimulation();

private:
  /**
   * @brief Runs the main simulation loop, which updates the physical model,
   * sets the correct rendering data and handles the consistent FPS of the
   * simulation.
   */
  void runSimulationThread();

  int simulationSpeed; ///< Number of ms per frame
  SimulationModel
      model; ///< Store of the physical model state of the simulation
  Renderer
      renderer; ///< The renderer responsible for drawing the simulation state
  GuiController guiController;

  std::thread *simulationThread; ///< Independent physical update thread

  float timeStep = 0.5f; // time step of the simulation
  float stiffness = 0.5f; // stiffness value of the simulation
  float pressure = 1.0f; // pressure value of the simulation
};