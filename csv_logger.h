#pragma once
#include <string>
#include <fstream>
#include <chrono>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct SensorSample {
    std::string label;  // e.g. "L_FA"
    glm::quat   q;
};

class CsvLogger {
public:
    CsvLogger();
    ~CsvLogger();

    bool open();

    // Pass ALL corrected quats every frame; identity = not yet received
    // Use isActive flags to tell logger which sensors are actually live
    void log(const std::vector<SensorSample>& samples,
             const std::vector<bool>& active);

    void close();

private:
    std::ofstream file;
    std::chrono::steady_clock::time_point startTime;
    int  frameCount   = 0;
    bool headerWritten = false;

    // Locked-in column order after first frame
    std::vector<int> activeIndices;  // indices into the samples vector

    static constexpr int kFlushEveryN = 60;

    static std::string makeFilepath();
    static bool        ensureDir(const std::string& dir);
};