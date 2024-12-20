/**
 * @file SimulationController.h
 * @brief Defines the SimulationController class which manages the main
 * simulation loop.
 */
#pragma once

#include "../model/Constraint.h"
#include "../model/SimulationModel.h"
#include "../view/Renderer.h"
#include "GuiController.h"
#include <chrono>
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
  int getTimeStep() { return this->timeStep; }

  /**
   * @brief Set time step size
   * @param timeStep Time step to set
   */
  void setTimeStep(int timeStep) { this->timeStep = timeStep; }

  /**
   * @brief get the compliance value of some constraint type
   */
  double getCompliance(EConstraintType cType) const {
    double compliance = model.getCompliance(cType);
    return (std::log10(compliance) + 5) / 10;
  }

  /**
   * @brief set the new compliance value to each constraint of some type
   * @param compliance new value of the constraint compliance
   */
  void setCompliance(EConstraintType cType, double compliance) {
    model.setCompliance(cType, std::pow(10, compliance * 10 - 5));
  }

  /**
   * @brief Getter for pressure value
   * @return Current pressure value
   */
  double getPressure() { return model.getPressureValue(); }

  /**
   * @brief Set pressure value
   * @param pressure pressure value it gets updated to
   */
  void setPressure(double pressure) { model.setPressureValue(pressure); }

  /**
   * @brief Getter for the static friction value
   * @return current static friction value
   */
  double getStaticFriction() {
    double staticMu = model.getStaticMu();
    return (std::log10(staticMu) + 9) / 10;
  }

  /**
   * @brief Getter for the kinetic friction value
   * @return current kinetic friction value
   */
  double getKineticFriction() {
    double kineticMu = model.getKineticMu();
    return (std::log10(kineticMu) + 9) / 10;
  }

  /**
   * @brief Set the static friction value
   * @param staticFriction static friction value it gets updated to
   */
  void setStaticFriction(double staticFriction) {
    model.setStaticMu(std::pow(10, staticFriction * 10 - 9));
  }

  /**
   * @brief Set the kinetic friction value
   * @param kineticFriction kinetic friction value it gets updated to
   */
  void setKineticFriction(double kineticFriction) {
    model.setKineticMu(std::pow(10, kineticFriction * 10 - 9));
  }

  float getFPS() const { return m_fps; }

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

  /**
   * @brief set if the constraint is enabled or not
   */
  void setState(bool state, EConstraintType type);

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

  int timeStep = 5; ///< initial time step value of the simulation
  float m_fps = 0;

  bool isSimulationRunning =
      false; ///< says if the simulation is running or not
};