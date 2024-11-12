#pragma once
#include <Eigen/Core>
#include "Mesh.h"
#include <vector>
#include "../view/RendereableMesh.h"

class SimulationModel {
public:
    SimulationModel();

    void update(double deltaTime); // Update the simulation state

    std::vector<Mesh> &getStatics(); // Get positions of particles/objects
    std::vector<Mesh> &getDynamics();    // Get topology information (faces)

    std::vector<Renderable> getRenderableMeshes() const;

private:
    void initialize(); // Initialize particles or objects
    void setupRenderableMeshes();
    std::vector<Mesh> staticObjs;
    std::vector<Mesh> dynamicObjs;
    std::vector<Renderable> renderables;
};
