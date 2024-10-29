#include "SimulationController.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

SimulationController::SimulationController(const int WIDTH, const int HEIGHT) {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // Set window hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create the GLFW window
    window = glfwCreateWindow(WIDTH, HEIGHT, "PBS-soft-body", NULL, NULL);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    // Make window the current context
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Load OpenGL functions with glad
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    renderer.init();
}

void SimulationController::run() {
    lastFrameTime = static_cast<float>(glfwGetTime());
    while (!glfwWindowShouldClose(window)) {
        float deltaTime = calculateDeltatime();

        processInput();

        update(deltaTime);

        render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clear all previously allocated GLFW resources
    glfwDestroyWindow(window);
    glfwTerminate();
}

void SimulationController::processInput() {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

void SimulationController::update(float deltaTime) {
    // update the physical model
    model.update(deltaTime);
}

void SimulationController::render() {
    // Clear buffers for a fresh render
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    // render the actual scene
    renderer.render(model); // TODO: here set the connection to the model
}

float SimulationController::calculateDeltatime() {
    float currentFrameTime = static_cast<float>(glfwGetTime());
    float deltaTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;
    return deltaTime;
}

void SimulationController::keyCallback() {}

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}
