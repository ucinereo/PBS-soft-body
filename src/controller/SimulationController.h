#pragma once
#include "../model/SimulationModel.h"
#include "../view/Renderer.h"
#include <threads.h>

class SimulationController {
public:
    SimulationController(int FPS);

    void run();

private:
    void runSimulationThread();

    int simulationSpeed;
    SimulationModel model;
    Renderer renderer;

    std::thread *simulationThread;
};