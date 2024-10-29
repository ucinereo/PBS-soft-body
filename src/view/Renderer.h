/**
 * @file Renderer.h
 * @brief Defines Renderer class responsible for drawing the simulation model on screen.
 */
#pragma once

#include "../model/SimulationModel.h"
#include <vector>

/**
 * @class Renderer
 * @brief Renderer class which is responsible for initializing OpenGL and drawing the
 * current simulation state from a model to the screen.
 */
class Renderer {
public:
    Renderer();
    ~Renderer();

    /**
     * @brief Initialize OpenGL
     */
    void init();

    /**
     * @brief Draw current simulation model to the screen
     * 
     * @param model Current state of the simulation model.
     */
    void render(const SimulationModel &model);

private:
    // temporary stuff which should be deleted afterwards
    std::vector<int> arr;
};
