/**
 * @file main.cpp
 * @brief Entry point for PBS-soft-body. The app is built on top of a MVC-architecture.
 */
#include "controller/SimulationController.h"

int main() {
    SimulationController c(600, 400);
    c.run();
}
