/**
 * @file SimulationController.h
 * @brief Defines SimulationController (class) which manages the main simulation loop.
 */
#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "../model/SimulationModel.h"
#include "../view/Renderer.h"

/**
 * @class SimulationController
 * @brief The main controller class that manages the simulation lifecycle, including
 * initialization, the main loop, input handling, updating the simulation, and rendering.
 * Coordinates the simulation model with the view (renderer).
 */
class SimulationController {
public:
    /**
     * @brief Construct a new Simulation Controller object
     * 
     * @param WIDTH Width of the initial window
     * @param HEIGHT Height of the initial window
     */
    SimulationController(const int WIDTH, const int HEIGHT);

    /**
     * @brief Runs the main simulation loop, which processes inputs, updates the
     * simulation model, and renders the scene until the window is closed.
     */
    void run();

private:
    /**
     * @brief Processes user inputs, such as key presses or mouse events.
     */
    void processInput();

    /**
     * @brief Updates the simulation model based on the elapsed time.
     * 
     * @param deltaTime The time elapsed since the last frame, used for consistent
     * simulation updates.
     */
    void update(float deltaTime);

    /**
     * @brief Start rendering the new simulation frame.
     */
    void render();

    /**
     * @brief Calculates the time elapsed between the current and previous frame.
     * 
     * @return delta time.
     */
    float calculateDeltatime();
    void keyCallback();

    GLFWwindow *window; ///< Point to the GLFW window used for OpenGL context
    Renderer renderer; ///< The renderer responsible for drawing the simulation state
    SimulationModel model; ///< Model which stores current state of simulation


    float lastFrameTime = 0.0f; ///< Time of the last frame, used for delta time calc.
};
