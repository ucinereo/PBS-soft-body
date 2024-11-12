#pragma once

#include <Eigen/Core>

class Mesh {
public:
    Mesh(Eigen::MatrixXd V, Eigen::MatrixXi F);

    // TODO: Might be able to work with references here? 
    const Eigen::MatrixXd getVertices() const; // Get positions of particles/objects
    const Eigen::MatrixXi getFaces() const;    // Get topology information (faces)

    void updateVertices(Eigen::MatrixXd V);

    Eigen::Vector4f getColor() const;
    void updateColor(float r, float g, float b, float alpha);

    size_t getID();
    void setID(size_t id);

private:
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    // TODO: extend to color matrix
    Eigen::Vector4f color = Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1.0f);

    // this is the ID set by the renderer
    size_t ID = -1;
};
