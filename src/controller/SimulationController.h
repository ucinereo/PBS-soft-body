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
  int getTimeStep();

  /**
   * @brief Set time step size
   * @param timeStep Time step to set
   */
  void setTimeStep(int timeStep);

  /**
   * @brief Getter for compliance value
   * @return Current compliance value
   */
  double getCompliance();

  /**
   * @brief Set compliance parameter
   * @param compliance compliance value it gets updated to
   */
  void setCompliance(double compliance);

  /**
   * @brief Getter for pressure value
   * @return Current pressure value
   */
  float getPressure();

  /**
   * @brief Set pressure value
   * @param pressure pressure value it gets updated to
   */
  void setPressure(float pressure);

  /**
   * @brief executes a single step of the simulation
   */
  void singleStep();

  /**
   * @brief resets the simulation
   */
  void resetSimulation();

  /**
   * @brief stops the simulation
   */
  void stopSimulation();

  /**
   * @brief starts the simulation
   */
  void startSimulation();

  /**
   * @brief Getter of running status of simulation
   * @return if simulation is running or not (false = not running, true =
   * running)
   */
  bool getIsSimulationRunning();

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

  int timeStep = 24;        ///< initial value 0.5f time step of the simulation
  double compliance = 0.5f; ///< inverse stiffness value of the simulation
  float pressure = 1.0f;    ///< pressure value of the simulation

  bool isSimulationRunning =
      false; ///< says if the simulation is running or not
};