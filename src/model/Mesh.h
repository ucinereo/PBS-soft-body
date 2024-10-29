#pragma once

#include <glm/glm.hpp>

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

class Mesh {
public:
    Mesh();
    ~Mesh();

    void draw() const;

private:
    void setupMesh();

    unsigned int VAO;
    unsigned int VBO;
    unsigned int EBO;
};