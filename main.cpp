#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "udp_receiver.h"
#include "sensor_manager.h"
#include "renderer.h"
#include "input_handler.h"
#include "csv_logger.h"

static GLFWwindow* g_window = nullptr;

int main()
{
    SensorManager sensorManager;
    std::thread receiver(udpReceiver, std::ref(sensorManager));

    if(!glfwInit())
        return -1;

    GLFWwindow* window = glfwCreateWindow(1400, 900,
        "IMU Visualizer - L_FA, R_FA, L_UA, R_UA, L_TH, L_SH, R_TH, R_SH", nullptr, nullptr);

    if(!window)
        return -1;

    g_window = window;
    glfwMakeContextCurrent(window);

    CsvLogger csvLogger;
    csvLogger.open();

    InputHandler inputHandler(sensorManager);
    glfwSetKeyCallback(window, keyCallbackDispatcher);
    glfwSetWindowUserPointer(window, &inputHandler);

    Renderer renderer;
    renderer.initialize();

    while(!glfwWindowShouldClose(window))
    {
        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
            renderer.setCameraView(CameraView::FRONT);
        if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
            renderer.setCameraView(CameraView::BACK);
        if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
            renderer.setCameraView(CameraView::SIDE);

        glm::quat correctedLFA   = sensorManager.getEffectiveLFAQuat();
        glm::quat correctedRFA   = sensorManager.getEffectiveRFAQuat();
        glm::quat correctedLUA   = sensorManager.getEffectiveLUAQuat();
        glm::quat correctedRUA   = sensorManager.getEffectiveRUAQuat();
        glm::quat correctedLTH   = sensorManager.getCorrectedLTHQuat();
        glm::quat correctedLSH   = sensorManager.getCorrectedLSHQuat();
        glm::quat correctedRTH   = sensorManager.getCorrectedRTHQuat();
        glm::quat correctedRSH   = sensorManager.getCorrectedRSHQuat();
        glm::quat correctedHips  = sensorManager.getCorrectedHipsQuat();
        glm::quat correctedChest = sensorManager.getCorrectedChestQuat();

        renderer.render(correctedLFA, correctedRFA, correctedLUA, correctedRUA,
                        correctedLTH, correctedRTH, correctedLSH, correctedRSH,
                        correctedHips, correctedChest);

        std::vector<SensorSample> samples = {
            { "L_FA",  correctedLFA   },
            { "R_FA",  correctedRFA   },
            { "L_UA",  correctedLUA   },
            { "R_UA",  correctedRUA   },
            { "L_TH",  correctedLTH   },
            { "L_SH",  correctedLSH   },
            { "R_TH",  correctedRTH   },
            { "R_SH",  correctedRSH   },
            { "HIPS",  correctedHips  },
            { "CHEST", correctedChest },
        };
        std::vector<bool> active = {
            sensorManager.isLFAActive(),
            sensorManager.isRFAActive(),
            sensorManager.isLUAActive(),
            sensorManager.isRUAActive(),
            sensorManager.isLTHActive(),
            sensorManager.isLSHActive(),
            sensorManager.isRTHActive(),
            sensorManager.isRSHActive(),
            sensorManager.isHipsActive(),
            sensorManager.isChestActive(),
        };
        csvLogger.log(samples, active);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    csvLogger.close();
    receiver.detach();
    glfwTerminate();
    return 0;
}