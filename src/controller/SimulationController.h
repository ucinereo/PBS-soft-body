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
   * @brief Getter for compliance value of static plane constraint
   * @return Current compliance value of static plane constraint
   */
  double getComplianceStaticPlane();

  /**
   * @brief Set compliance parameter of static plane constraint
   * @param complianceStaticPlane compliance value it gets updated to
   */
  void setComplianceStaticPlane(double complianceStaticPlane);

  /**
   * @brief Getter for compliance value of distance constraint
   * @return Current compliance value of distance constraint
   */
  double getComplianceDistance();

  /**
   * @brief Set compliance parameter of distance constraint
   * @param complianceDistance compliance value it gets updated to
   */
  void setComplianceDistance(double complianceDistance);

   /**
   * @brief Getter for compliance value of plane friction constraint
   * @return Current compliance value of plane friction constraint
   */
  double getCompliancePlaneFriction();

  /**
   * @brief Set compliance parameter of plane friction constraint
   * @param compliancePlaneFriction compliance value it gets updated to
   */
  void setCompliancePlaneFriction(double compliancePlaneFriction);



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
   * @brief Getter for friction value
   * @return current friction value
   */
  float getFriction();

  /**
   * @brief Set friction value
   * @param friction friction value it gets updated to
   */
  void setFriction(float friction);

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
   * @brief exports current obj
   */
  void exportObj();

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

  int timeStep = 24; ///< initial value 0.5f time step of the simulation
  double complianceDistance =
      0.5f; ///< compliance value of the distance constraint
  double complianceStaticPlane =
      0.5f;              ///< compliance value of the static plane constraint
  double compliancePlaneFriction = 0.5f; ///< compliance value of the plane friction constraint
  float pressure = 1.0f; ///< pressure value of the simulation
  float friction = 0.0f;

  bool isSimulationRunning =
      false; ///< says if the simulation is running or not
};