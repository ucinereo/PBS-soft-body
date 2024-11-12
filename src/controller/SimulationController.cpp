#include "SimulationController.h"
#include <chrono>
#include <thread>
#include <iostream>

SimulationController::SimulationController(int FPS) : model(), renderer() {
    auto &dynamics = model.getDynamics();
    renderer.registerDynamics(dynamics);
    renderer.setMeshData(dynamics);

    auto &statics = model.getStatics();
    renderer.registerStatics(statics);
    renderer.setMeshData(statics);

    renderer.registerToLibigl();

    // renderer.initCustomShader();
    simulationSpeed = std::round(1000 / FPS);
    simulationThread = new std::thread(&SimulationController::runSimulationThread, this);
    renderer.getViewer().launch_rendering(true);
    // renderer.getViewer().launch();
}

void SimulationController::run() {
}

void SimulationController::runSimulationThread() {
    while (true) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Update simulation
        model.update(simulationSpeed);

        // Render the scene
        renderer.getLock()->lock();
        auto &list = model.getDynamics();
        renderer.setMeshData(list);
        renderer.getLock()->unlock();

        auto endTime = std::chrono::high_resolution_clock::now();

        auto deltaTime = endTime - startTime;

        std::chrono::milliseconds sleepTime = std::chrono::milliseconds(simulationSpeed)
            - std::chrono::duration_cast<std::chrono::milliseconds>(deltaTime);

        std::this_thread::sleep_for(sleepTime);
    }
}
