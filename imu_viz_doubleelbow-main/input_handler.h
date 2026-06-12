#pragma once
#include <GLFW/glfw3.h>
#include "sensor_manager.h"

class InputHandler {
public:
    InputHandler(SensorManager& sensorManager);
    void handleKey(int key);

private:
    SensorManager& sensorManager;
};

void keyCallbackDispatcher(GLFWwindow* window, int key, int scancode, 
                          int action, int mods);