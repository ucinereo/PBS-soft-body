/**
 * @file SimulationController.h
 * @brief Defines the SimulationController class which manages the main
 * simulation loop.
 */
#pragma once

#include "../model/Constraint.h"
#include "../model/SimulationModel.h"
#include "../scenes.h"
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
  int getTimeStep() { return this->m_timeStep; }

  /**
   * @brief Set time step size
   * @param timeStep Time step to set
   */
  void setTimeStep(int timeStep) { this->m_timeStep = timeStep; }

  /**
   * @brief Get the compliance value of some constraint type
   */
  double getCompliance(EConstraintType cType) const {
    double compliance = m_model.getCompliance(cType);
    return (std::log10(compliance) + 5) / 10;
  }

  /**
   * @brief Set the new compliance value to each constraint of some type
   * @param compliance new value of the constraint compliance
   */
  void setCompliance(EConstraintType cType, double compliance) {
    m_model.setCompliance(cType, std::pow(10, compliance * 10 - 5));
  }

  /**
   * @brief Getter for pressure value
   * @return Current pressure value
   */
  double getPressure() { return m_model.getPressureValue(); }

  /**
   * @brief Set pressure value
   * @param pressure pressure value it gets updated to
   */
  void setPressure(double pressure) { m_model.setPressureValue(pressure); }

  /**
   * @brief Getter for the static friction value
   * @return current static friction value
   */
  double getStaticFriction() {
    double staticMu = m_model.getStaticMu();
    return (std::log10(staticMu) + 9) / 10;
  }

  /**
   * @brief Getter for the kinetic friction value
   * @return current kinetic friction value
   */
  double getKineticFriction() {
    double kineticMu = m_model.getKineticMu();
    return (std::log10(kineticMu) + 9) / 10;
  }

  /**
   * @brief Set the static friction value
   * @param staticFriction static friction value it gets updated to
   */
  void setStaticFriction(double staticFriction) {
    m_model.setStaticMu(std::pow(10, staticFriction * 10 - 9));
  }

  /**
   * @brief Set the kinetic friction value
   * @param kineticFriction kinetic friction value it gets updated to
   */
  void setKineticFriction(double kineticFriction) {
    m_model.setKineticMu(std::pow(10, kineticFriction * 10 - 9));
  }

  float getFPS() const { return m_fps; }

  /**
   * @brief Executes a single step of the simulation
   */
  void singleStep();

  /**
   * @brief Resets the simulation
   */
  void resetSimulation();

  /**
   * @brief Stops the simulation
   */
  void stopSimulation();

  /**
   * @brief Starts the simulation
   */
  void startSimulation();

  /**
   * @brief Exports current obj
   */
  void exportObj();

  /**
   * @brief Getter of running status of simulation
   * @return Whether simulation is running or not (false = not running, true =
   * running)
   */
  bool getIsSimulationRunning();

  /**
   * @brief Set whether the constraint is enabled or not
   * @param cType The constraint type
   * @param state true for enabling, false for disabling
   */
  void setState(EConstraintType cType, bool state) {
    m_model.setActive(cType, state);
  }

  /**
   * @brief Get the current scene type
   * @returns The current scene type
   */
  ESceneType getSceneType() const { return m_model.getSceneType(); }

  /**
   * @brief Replaces the current scene
   * @param sceneType The new scene type
   */
  void replaceScene(ESceneType sceneType);

private:
  /**
   * @brief Runs the main simulation loop, which updates the physical model,
   * sets the correct rendering data and handles the consistent FPS of the
   * simulation.
   */
  void runSimulationThread();

  int m_simulationSpeed; ///< Number of ms per frame
  SimulationModel
      m_model; ///< Store of the physical model state of the simulation
  Renderer
      m_renderer; ///< The renderer responsible for drawing the simulation state
  GuiController m_guiController;

  std::thread *m_simulationThread; ///< Independent physical update thread

  int m_timeStep = 5; ///< initial time step value of the simulation
  float m_fps = 0;    ///< the current FPS of the simulation, this value gets
                   ///< updated via computing an exponential moving average, so
                   ///< abrupt changes might not directly be observable, however
                   ///< it makes it easier to keep track of the current value

  bool m_isSimulationRunning =
      false; ///< says if the simulation is running or not
};