#include "csv_logger.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>

CsvLogger::CsvLogger()  = default;
CsvLogger::~CsvLogger() { close(); }

bool CsvLogger::ensureDir(const std::string& dir)
{
    struct stat st{};
    if (stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    return mkdir(dir.c_str(), 0755) == 0;
}

std::string CsvLogger::makeFilepath()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return std::string("CSV/") + buf + ".csv";
}

bool CsvLogger::open()
{
    if (!ensureDir("CSV")) {
        std::cerr << "[CsvLogger] Failed to create CSV/ directory\n";
        return false;
    }
    std::string path = makeFilepath();
    file.open(path, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "[CsvLogger] Failed to open: " << path << "\n";
        return false;
    }
    startTime = std::chrono::steady_clock::now();
    std::cout << "[CsvLogger] Logging to: " << path << "\n";
    return true;
}

void CsvLogger::log(const std::vector<SensorSample>& samples,
                    const std::vector<bool>& active)
{
    if (!file.is_open()) return;

    // First frame: lock in which sensors are active, write header
    if (!headerWritten) {
        for (int i = 0; i < (int)active.size(); ++i) {
            if (active[i]) activeIndices.push_back(i);
        }
        if (activeIndices.empty()) return;  // nothing online yet, wait

        file << "time";
        for (int idx : activeIndices) {
            const std::string& lbl = samples[idx].label;
            file << ',' << lbl << "_w"
                 << ',' << lbl << "_x"
                 << ',' << lbl << "_y"
                 << ',' << lbl << "_z";
        }
        file << '\n';
        headerWritten = true;
    }

    auto now     = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    file << std::fixed << std::setprecision(6) << elapsed;

    for (int idx : activeIndices) {
        const glm::quat& q = samples[idx].q;
        file << ',' << q.w << ',' << q.x << ',' << q.y << ',' << q.z;
    }
    file << '\n';

    if (++frameCount % kFlushEveryN == 0)
        file.flush();
}

void CsvLogger::close()
{
    if (file.is_open()) {
        file.flush();
        file.close();
        std::cout << "[CsvLogger] File closed.\n";
    }
}