#include "SimulationModel.h"
#include <igl/readOFF.h>
#include <Eigen/Core>
#include "../utils.h"

SimulationModel::SimulationModel() {
    initialize();
    setupRenderableMeshes();
}

void SimulationModel::initialize() {
    // TODO: Update this initialization with a sceen parsing or something...
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOFF("../bunny.off", V, F);

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
    V = (rotationMatrix * V.transpose()).transpose();
    V *= 0.1;

    Mesh bunny(V, F);
    bunny.updateColor(1.f, 0.77f, 0.82f, 1.0f);
    dynamicObjs.push_back(bunny);

    Eigen::Vector3d pos;
    pos << 15.f, 0.f, 0.f;
    Eigen::MatrixXd V2 = V.rowwise() + pos.transpose();
    Mesh bunny2(V2, F);
    bunny2.updateColor(1.f, 0.77f, 0.82f, 1.0f);
    dynamicObjs.push_back(bunny2);

    Eigen::MatrixXd floorV;
    Eigen::MatrixXi floorF;
    createFloorMesh(floorV, floorF);
    staticObjs.push_back(Mesh(floorV, floorF));
}

void SimulationModel::update(double deltaTime) {
    // Update simulation state using positional-based dynamics or other physics

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix = Eigen::AngleAxisd(-M_PI / 16, Eigen::Vector3d::UnitY());

    Mesh &o1 = dynamicObjs[0];
    auto V = o1.getVertices();
    o1.updateVertices((rotationMatrix * V.transpose()).transpose());
}

std::vector<Mesh>& SimulationModel::getStatics() {
    return staticObjs;
}

std::vector<Mesh>& SimulationModel::getDynamics() {
    return dynamicObjs;
}

std::vector<Renderable> SimulationModel::getRenderableMeshes() const {
    return renderables;
}

void SimulationModel::setupRenderableMeshes() {
    for (auto mesh : dynamicObjs) {
        renderables.emplace_back(mesh, ShaderType::Static);
    }
    for (auto mesh : staticObjs) {
        renderables.emplace_back(mesh, ShaderType::Static);
    }
}
