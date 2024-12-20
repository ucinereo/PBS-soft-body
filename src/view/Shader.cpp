/**
 * @file Shader.cpp
 * @brief Shader definitions
 */

#include "Shader.h"
#include <fstream>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include <sstream>

Shader::Shader(std::string vertexFilePath, std::string fragmentFilePath) {
  std::ifstream vertexShaderFile(vertexFilePath);
  std::ifstream fragmentShaderFile(fragmentFilePath);

  if (!vertexShaderFile.is_open()) {
    throw std::runtime_error("Could not open file: " + vertexFilePath);
  }

  if (!fragmentShaderFile.is_open()) {
    throw std::runtime_error("Could not open file: " + vertexFilePath);
  }

  std::stringstream vertexBuffer, fragmentBuffer;
  vertexBuffer << vertexShaderFile.rdbuf();
  fragmentBuffer << fragmentShaderFile.rdbuf();

  m_vertexShaderCode = vertexBuffer.str();
  m_fragmentShaderCode = fragmentBuffer.str();
}

void Shader::linkShader(igl::opengl::glfw::Viewer &viewer) {
  // load the shader program
  bool compiled = igl::opengl::create_shader_program(
      m_vertexShaderCode, m_fragmentShaderCode, {}, m_progId);

  if (!compiled) {
    std::cout << "PBS::Shader Linkage failed";
  }
}

unsigned int Shader::getProgID() { return m_progId; }