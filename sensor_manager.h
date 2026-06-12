#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <atomic>
#include <mutex>

class SensorManager {
public:
    SensorManager();
    
    void setLFAQuat(const glm::quat& q);
    void setRFAQuat(const glm::quat& q);
    void setLUAQuat(const glm::quat& q);
    void setRUAQuat(const glm::quat& q);
    
    glm::quat getLFAQuat() const;
    glm::quat getRFAQuat() const;
    glm::quat getLUAQuat() const;
    glm::quat getRUAQuat() const;
    
    void calibrateLFA();
    void calibrateRFA();
    void calibrateLUA();
    void calibrateRUA();
    void toggleQuaternionConvention();

    // ── Runtime arm-segment swap ────────────────────────────────────
    // Swaps which physical sensor drives the upper-arm vs forearm bone,
    // independently per side. Toggled live with no recompile.
    void toggleArmSwapLeft();
    void toggleArmSwapRight();
    bool isArmSwapLeft()  const { return armSwapLeft.load(); }
    bool isArmSwapRight() const { return armSwapRight.load(); }
    
    glm::quat getCorrectedLFAQuat() const;
    glm::quat getCorrectedRFAQuat() const;
    glm::quat getCorrectedLUAQuat() const;
    glm::quat getCorrectedRUAQuat() const;

    void setLTHQuat(const glm::quat& q);
    void setLSHQuat(const glm::quat& q);
    void setRTHQuat(const glm::quat& q);
    void setRSHQuat(const glm::quat& q);

    glm::quat getLTHQuat() const;
    glm::quat getLSHQuat() const;
    glm::quat getRTHQuat() const;
    glm::quat getRSHQuat() const;

    void calibrateLTH();
    void calibrateLSH();
    void calibrateRTH();
    void calibrateRSH();

    glm::quat getCorrectedLTHQuat() const;
    glm::quat getCorrectedLSHQuat() const;
    glm::quat getCorrectedRTHQuat() const;
    glm::quat getCorrectedRSHQuat() const;

    void setHipsQuat(const glm::quat& q);
    void setChestQuat(const glm::quat& q);

    glm::quat getHipsQuat()  const;
    glm::quat getChestQuat() const;

    void calibrateHips();
    void calibrateChest();

    glm::quat getCorrectedHipsQuat()  const;
    glm::quat getCorrectedChestQuat() const;

    // ── Swap-aware effective getters ────────────────────────────────
    // These return what should actually drive the "upper arm" / "forearm"
    // bones after applying the live arm-swap toggles. Use these in
    // main.cpp / renderer instead of the raw Corrected*Quat getters
    // for the four arm segments.
    glm::quat getEffectiveLUAQuat() const;
    glm::quat getEffectiveLFAQuat() const;
    glm::quat getEffectiveRUAQuat() const;
    glm::quat getEffectiveRFAQuat() const;

    // Active sensor detection
    bool isLFAActive()   const;
    bool isRFAActive()   const;
    bool isLUAActive()   const;
    bool isRUAActive()   const;
    bool isLTHActive()   const;
    bool isLSHActive()   const;
    bool isRTHActive()   const;
    bool isRSHActive()   const;
    bool isHipsActive()  const;
    bool isChestActive() const;

private:
    static constexpr float kSmoothingAlpha       = 0.35f;
    static constexpr float kFastSmoothingAlpha   = 0.70f;
    static constexpr float kDeadbandRadians      = 0.003f;
    static constexpr float kFastMotionRadians    = 0.20f;

    // ── Advanced stationary / drift-correction tuning ──────────────
    // A sensor is "stationary" once its short-term motion variance
    // (rolling RMS of frame-to-frame angle) drops below this.
    static constexpr float kStationaryThreshold  = 0.0025f;
    // Time (ms) of continuous stillness before drift correction begins.
    static constexpr float kStationaryTimeMs     = 800.0f;
    // Time (ms) of continuous stillness before the FAST drift-lock
    // kicks in (sensor is "dead still" — pull reference in hard).
    static constexpr float kDeepStillTimeMs      = 4000.0f;
    // Base drift correction strength (slerp factor per frame) once
    // kStationaryTimeMs has elapsed.
    static constexpr float kDriftCorrAlphaBase   = 0.0025f;
    // Drift correction strength once kDeepStillTimeMs has elapsed —
    // much stronger, locks the reference to current orientation.
    static constexpr float kDriftCorrAlphaDeep   = 0.05f;
    // Rolling-variance smoothing factor (EMA of |angle| per frame).
    static constexpr float kVarianceAlpha        = 0.08f;
    // Deadband adapts between these bounds based on recent jitter.
    static constexpr float kDeadbandMin          = 0.0015f;
    static constexpr float kDeadbandMax          = 0.012f;


    std::atomic<int> quaternionMode;
    std::atomic<bool> armSwapLeft{false};
    std::atomic<bool> armSwapRight{false};

    mutable std::mutex quatMutex1;
    glm::quat sensorQuatLFA;
    bool activeLFA = false;
    mutable std::mutex calibMutex1;
    mutable glm::quat calibrationReferenceLFA;
    mutable glm::quat smoothedCorrectedLFA;
    mutable bool hasSmoothedCorrectedLFA;
    mutable glm::quat lastQLFA;
    mutable float stationaryTimerLFA;
    mutable float jitterEmaLFA = 0.0f;

    mutable std::mutex quatMutex2;
    glm::quat sensorQuatRFA;
    bool activeRFA = false;
    mutable std::mutex calibMutex2;
    mutable glm::quat calibrationReferenceRFA;
    mutable glm::quat smoothedCorrectedRFA;
    mutable bool hasSmoothedCorrectedRFA;
    mutable glm::quat lastQRFA;
    mutable float stationaryTimerRFA;
    mutable float jitterEmaRFA = 0.0f;

    mutable std::mutex quatMutex3;
    glm::quat sensorQuat3;
    bool activeLUA = false;
    mutable std::mutex calibMutex3;
    mutable glm::quat calibrationReference3;
    mutable glm::quat smoothedCorrected3;
    mutable bool hasSmoothedCorrected3;
    mutable glm::quat lastQ3;
    mutable float stationaryTimer3;
    mutable float jitterEma3 = 0.0f;

    mutable std::mutex quatMutex4;
    glm::quat sensorQuatRUA;
    bool activeRUA = false;
    mutable std::mutex calibMutex4;
    mutable glm::quat calibrationReferenceRUA;
    mutable glm::quat smoothedCorrectedRUA;
    mutable bool hasSmoothedCorrectedRUA;
    mutable glm::quat lastQRUA;
    mutable float stationaryTimerRUA;
    mutable float jitterEmaRUA = 0.0f;

    mutable std::mutex quatMutex5;
    glm::quat sensorQuat5;
    bool activeLTH = false;
    mutable std::mutex calibMutex5;
    mutable glm::quat calibrationReference5;
    mutable glm::quat smoothedCorrected5;
    mutable bool hasSmoothedCorrected5;
    mutable glm::quat lastQ5;
    mutable float stationaryTimer5;
    mutable float jitterEma5 = 0.0f;

    mutable std::mutex quatMutex6;
    glm::quat sensorQuat6;
    bool activeLSH = false;
    mutable std::mutex calibMutex6;
    mutable glm::quat calibrationReference6;
    mutable glm::quat smoothedCorrected6;
    mutable bool hasSmoothedCorrected6;
    mutable glm::quat lastQ6;
    mutable float stationaryTimer6;
    mutable float jitterEma6 = 0.0f;

    mutable std::mutex quatMutex7;
    glm::quat sensorQuat7;
    bool activeRTH = false;
    mutable std::mutex calibMutex7;
    mutable glm::quat calibrationReference7;
    mutable glm::quat smoothedCorrected7;
    mutable bool hasSmoothedCorrected7;
    mutable glm::quat lastQ7;
    mutable float stationaryTimer7;
    mutable float jitterEma7 = 0.0f;

    mutable std::mutex quatMutex8;
    glm::quat sensorQuat8;
    bool activeRSH = false;
    mutable std::mutex calibMutex8;
    mutable glm::quat calibrationReference8;
    mutable glm::quat smoothedCorrected8;
    mutable bool hasSmoothedCorrected8;
    mutable glm::quat lastQ8;
    mutable float stationaryTimer8;
    mutable float jitterEma8 = 0.0f;

    mutable std::mutex quatMutex9;
    glm::quat sensorQuat9;
    bool activeHips = false;
    mutable std::mutex calibMutex9;
    mutable glm::quat calibrationReference9;
    mutable glm::quat smoothedCorrected9;
    mutable bool hasSmoothedCorrected9;
    mutable glm::quat lastQ9;
    mutable float stationaryTimer9;
    mutable float jitterEma9 = 0.0f;

    mutable std::mutex quatMutex10;
    glm::quat sensorQuat10;
    bool activeChest = false;
    mutable std::mutex calibMutex10;
    mutable glm::quat calibrationReference10;
    mutable glm::quat smoothedCorrected10;
    mutable bool hasSmoothedCorrected10;
    mutable glm::quat lastQ10;
    mutable float stationaryTimer10;
    mutable float jitterEma10 = 0.0f;

    void updateSensorQuat(glm::quat& current, const glm::quat& incoming) const;
    glm::quat neutralPose() const;
    glm::quat computeMotionDelta(const glm::quat& sensorQuat,
                                 const glm::quat& calibrationReference) const;
    glm::quat computeCorrectedQuat(const glm::quat& sensorQuat,
                                   const glm::quat& calibrationReference) const;
    glm::quat smoothCorrectedQuat(glm::quat& current,
                                  bool& initialized,
                                  const glm::quat& target) const;
    glm::quat smoothCorrectedQuatAdaptive(glm::quat& current,
                                          bool& initialized,
                                          const glm::quat& target,
                                          float jitterEMA) const;
    void autoRecalibrate(glm::quat& calibRef,
                         glm::quat& lastQ,
                         float& stationaryTimer,
                         float& jitterEMA,
                         const glm::quat& current) const;
    float adaptiveDeadband(float jitterEMA) const;
};