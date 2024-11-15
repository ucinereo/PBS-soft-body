/**
 * @file Shader.h
 * @brief Loader for vertex and fragment shaders and linkage to libigl
 */

#pragma once

#include <iostream>
#include <string.h>
#include <igl/opengl/glfw/Viewer.h>

/**
 * @class Shader
 * @brief Fragment and vertex shader load with libigl linkage
 */
class Shader {
public:
    /**
     * @brief Empty dfault constructor
     */
    Shader() {};

    /**
     * @brief Construct new shader object with vertex and fragment shader
     * @param vertexFilePath File path to vertex shader
     * @param fragmentFilePath File path to fragment shader
     */
    Shader(std::string vertexFilePath, std::string fragmentFilePath);

    /**
     * @brief Link shader to libigl, which compiles it, handles the error handling and
     * stores the program id.
     * @param viewer Libigl viewer reference.
     */
    void linkShader(igl::opengl::glfw::Viewer &viewer);

    /**
     * @brief Getter for linked opengl program id
     * @return opengl program id
     */
    unsigned int getProgID();

private:
    unsigned int progId; ///< OpenGL program id
    std::string vertexShaderCode; ///< vertex shader code
    std::string fragmentShaderCode; ///< fragment shader code
};