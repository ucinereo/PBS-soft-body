#pragma once

#include <iostream>
#include <string.h>
#include <igl/opengl/glfw/Viewer.h>

class Shader {
public:
    // Default Constructor
    Shader() {};
    Shader(std::string vertexFilePath, std::string fragmentFilePath);
    void linkShader(igl::opengl::glfw::Viewer &viewer);
    unsigned int getProgID();

private:
    unsigned int progId;
    std::string vertexShaderCode;
    std::string fragmentShaderCode;
};