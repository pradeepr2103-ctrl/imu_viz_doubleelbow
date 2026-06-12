#include "sensor_manager.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/gtx/quaternion.hpp>

SensorManager::SensorManager()
    : sensorQuatLFA(1,0,0,0), sensorQuatRFA(1,0,0,0)
    , sensorQuat3(1,0,0,0),   sensorQuatRUA(1,0,0,0)
    , sensorQuat5(1,0,0,0),   sensorQuat6(1,0,0,0)
    , sensorQuat7(1,0,0,0),   sensorQuat8(1,0,0,0)
    , sensorQuat9(1,0,0,0),   sensorQuat10(1,0,0,0)
    , quaternionMode(0)
    , calibrationReferenceLFA(1,0,0,0), smoothedCorrectedLFA(1,0,0,0)
    , hasSmoothedCorrectedLFA(false),   lastQLFA(1,0,0,0), stationaryTimerLFA(0)
    , calibrationReferenceRFA(1,0,0,0), smoothedCorrectedRFA(1,0,0,0)
    , hasSmoothedCorrectedRFA(false),   lastQRFA(1,0,0,0), stationaryTimerRFA(0)
    , calibrationReference3(1,0,0,0),   smoothedCorrected3(1,0,0,0)
    , hasSmoothedCorrected3(false),     lastQ3(1,0,0,0), stationaryTimer3(0)
    , calibrationReferenceRUA(1,0,0,0), smoothedCorrectedRUA(1,0,0,0)
    , hasSmoothedCorrectedRUA(false),   lastQRUA(1,0,0,0), stationaryTimerRUA(0)
    , calibrationReference5(1,0,0,0),   smoothedCorrected5(1,0,0,0)
    , hasSmoothedCorrected5(false),     lastQ5(1,0,0,0), stationaryTimer5(0)
    , calibrationReference6(1,0,0,0),   smoothedCorrected6(1,0,0,0)
    , hasSmoothedCorrected6(false),     lastQ6(1,0,0,0), stationaryTimer6(0)
    , calibrationReference7(1,0,0,0),   smoothedCorrected7(1,0,0,0)
    , hasSmoothedCorrected7(false),     lastQ7(1,0,0,0), stationaryTimer7(0)
    , calibrationReference8(1,0,0,0),   smoothedCorrected8(1,0,0,0)
    , hasSmoothedCorrected8(false),     lastQ8(1,0,0,0), stationaryTimer8(0)
    , calibrationReference9(1,0,0,0),   smoothedCorrected9(1,0,0,0)
    , hasSmoothedCorrected9(false),     lastQ9(1,0,0,0), stationaryTimer9(0)
    , calibrationReference10(1,0,0,0),  smoothedCorrected10(1,0,0,0)
    , hasSmoothedCorrected10(false),    lastQ10(1,0,0,0), stationaryTimer10(0)
{
    std::cout << "Quaternion mode 1/4. Press M to cycle, then recalibrate.\n";
}

// ── core helpers ──────────────────────────────────────────────────────────────

void SensorManager::updateSensorQuat(glm::quat& current, const glm::quat& incoming) const
{
    glm::quat t = glm::normalize(incoming);
    if (glm::dot(current, t) < 0.0f) t = -t;
    current = t;
}

glm::quat SensorManager::neutralPose() const
{
    return glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::quat SensorManager::computeMotionDelta(const glm::quat& sensorQuat,
                                             const glm::quat& calibrationReference) const
{
    glm::quat current   = glm::normalize(sensorQuat);
    glm::quat reference = glm::normalize(calibrationReference);
    if (glm::dot(reference, current) < 0.0f) current = -current;

    switch (quaternionMode.load()) {
    case 0:  return glm::normalize(current * glm::inverse(reference));
    case 1:  return glm::normalize(glm::inverse(reference) * current);
    case 2:  return glm::normalize(glm::conjugate(current) * glm::inverse(glm::conjugate(reference)));
    default: return glm::normalize(glm::inverse(glm::conjugate(reference)) * glm::conjugate(current));
    }
}

glm::quat SensorManager::computeCorrectedQuat(const glm::quat& sensorQuat,
                                               const glm::quat& calibrationReference) const
{
    glm::quat delta = computeMotionDelta(sensorQuat, calibrationReference);
    glm::quat q = glm::normalize(delta * neutralPose());
    q.x = -q.x;
    q.z = -q.z;
    return glm::normalize(q);
}

glm::quat SensorManager::smoothCorrectedQuat(glm::quat& current,
                                              bool& initialized,
                                              const glm::quat& target) const
{
    glm::quat nt = glm::normalize(target);
    if (!initialized) { current = nt; initialized = true; return current; }
    if (glm::dot(current, nt) < 0.0f) nt = -nt;

    const float dot   = std::clamp(glm::dot(current, nt), -1.0f, 1.0f);
    const float angle = 2.0f * std::acos(std::abs(dot));
    if (angle < kDeadbandRadians) return current;

    const float t     = glm::smoothstep(0.05f, 0.35f, angle);
    const float alpha = glm::mix(kSmoothingAlpha, kFastSmoothingAlpha, t);
    current = glm::normalize(glm::slerp(current, nt, alpha));
    return current;
}

// Jitter-aware variant: deadband adapts to each sensor's recent noise
// floor (computed in autoRecalibrate), so quiet/noisy sensors are each
// filtered appropriately instead of sharing one fixed threshold.
glm::quat SensorManager::smoothCorrectedQuatAdaptive(glm::quat& current,
                                                      bool& initialized,
                                                      const glm::quat& target,
                                                      float jitterEMA) const
{
    glm::quat nt = glm::normalize(target);
    if (!initialized) { current = nt; initialized = true; return current; }
    if (glm::dot(current, nt) < 0.0f) nt = -nt;

    const float dot   = std::clamp(glm::dot(current, nt), -1.0f, 1.0f);
    const float angle = 2.0f * std::acos(std::abs(dot));

    const float deadband = adaptiveDeadband(jitterEMA);
    if (angle < deadband) return current;

    const float t     = glm::smoothstep(0.05f, 0.35f, angle);
    const float alpha = glm::mix(kSmoothingAlpha, kFastSmoothingAlpha, t);
    current = glm::normalize(glm::slerp(current, nt, alpha));
    return current;
}

float SensorManager::adaptiveDeadband(float jitterEMA) const
{
    // Sensors that are very still (low jitterEMA) get a tight deadband
    // so small intentional movements aren't swallowed. Noisy sensors
    // get a wider deadband to suppress jitter. Scales linearly between
    // kDeadbandMin and kDeadbandMax based on observed jitter level.
    const float t = std::clamp(jitterEMA / kStationaryThreshold, 0.0f, 1.0f);
    return glm::mix(kDeadbandMin, kDeadbandMax, t);
}

void SensorManager::autoRecalibrate(glm::quat& calibRef,
                                     glm::quat& lastQ,
                                     float& stationaryTimer,
                                     float& jitterEMA,
                                     const glm::quat& current) const
{
    const float angle = 2.0f * std::acos(
        std::clamp(std::abs(glm::dot(lastQ, current)), 0.0f, 1.0f));

    // Rolling exponential-moving-average of per-frame motion — a more
    // robust "stationary" signal than a single-sample threshold, since
    // it smooths out single-frame sensor spikes/outliers.
    jitterEMA = glm::mix(jitterEMA, angle, kVarianceAlpha);

    if (jitterEMA < kStationaryThreshold) {
        stationaryTimer += 16.0f;

        if (stationaryTimer > kDeepStillTimeMs) {
            // Dead still for a long time — lock reference to current
            // orientation aggressively. This fully cancels accumulated
            // drift once the user has clearly stopped moving.
            calibRef = glm::normalize(glm::slerp(calibRef, current, kDriftCorrAlphaDeep));
        } else if (stationaryTimer > kStationaryTimeMs) {
            // Mildly still — gentle continuous drift correction so slow
            // gyro bias drift never has a chance to accumulate visibly.
            calibRef = glm::normalize(glm::slerp(calibRef, current, kDriftCorrAlphaBase));
        }
    } else {
        // Real motion detected — reset stillness timer, but keep the
        // jitterEMA decaying naturally rather than snapping to zero,
        // so brief motion blips don't instantly re-arm drift correction.
        stationaryTimer = 0.0f;
    }
    lastQ = current;
}

// ── setters ───────────────────────────────────────────────────────────────────

void SensorManager::setLFAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex1);  updateSensorQuat(sensorQuatLFA, q); activeLFA   = true; }
void SensorManager::setRFAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex2);  updateSensorQuat(sensorQuatRFA, q); activeRFA   = true; }
void SensorManager::setLUAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex3);  updateSensorQuat(sensorQuat3,   q); activeLUA   = true; }
void SensorManager::setRUAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex4);  updateSensorQuat(sensorQuatRUA, q); activeRUA   = true; }
void SensorManager::setLTHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex5);  updateSensorQuat(sensorQuat5,   q); activeLTH   = true; }
void SensorManager::setLSHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex6);  updateSensorQuat(sensorQuat6,   q); activeLSH   = true; }
void SensorManager::setRTHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex7);  updateSensorQuat(sensorQuat7,   q); activeRTH   = true; }
void SensorManager::setRSHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex8);  updateSensorQuat(sensorQuat8,   q); activeRSH   = true; }
void SensorManager::setHipsQuat(const glm::quat& q) { std::lock_guard<std::mutex> l(quatMutex9);  updateSensorQuat(sensorQuat9,   q); activeHips  = true; }
void SensorManager::setChestQuat(const glm::quat& q){ std::lock_guard<std::mutex> l(quatMutex10); updateSensorQuat(sensorQuat10,  q); activeChest = true; }

// ── getters ───────────────────────────────────────────────────────────────────

glm::quat SensorManager::getLFAQuat()  const { std::lock_guard<std::mutex> l(quatMutex1);  return sensorQuatLFA; }
glm::quat SensorManager::getRFAQuat()  const { std::lock_guard<std::mutex> l(quatMutex2);  return sensorQuatRFA; }
glm::quat SensorManager::getLUAQuat()  const { std::lock_guard<std::mutex> l(quatMutex3);  return sensorQuat3;   }
glm::quat SensorManager::getRUAQuat()  const { std::lock_guard<std::mutex> l(quatMutex4);  return sensorQuatRUA; }
glm::quat SensorManager::getLTHQuat()  const { std::lock_guard<std::mutex> l(quatMutex5);  return sensorQuat5;   }
glm::quat SensorManager::getLSHQuat()  const { std::lock_guard<std::mutex> l(quatMutex6);  return sensorQuat6;   }
glm::quat SensorManager::getRTHQuat()  const { std::lock_guard<std::mutex> l(quatMutex7);  return sensorQuat7;   }
glm::quat SensorManager::getRSHQuat()  const { std::lock_guard<std::mutex> l(quatMutex8);  return sensorQuat8;   }
glm::quat SensorManager::getHipsQuat() const { std::lock_guard<std::mutex> l(quatMutex9);  return sensorQuat9;   }
glm::quat SensorManager::getChestQuat()const { std::lock_guard<std::mutex> l(quatMutex10); return sensorQuat10;  }

// ── active flags ──────────────────────────────────────────────────────────────

bool SensorManager::isLFAActive()   const { std::lock_guard<std::mutex> l(quatMutex1);  return activeLFA;   }
bool SensorManager::isRFAActive()   const { std::lock_guard<std::mutex> l(quatMutex2);  return activeRFA;   }
bool SensorManager::isLUAActive()   const { std::lock_guard<std::mutex> l(quatMutex3);  return activeLUA;   }
bool SensorManager::isRUAActive()   const { std::lock_guard<std::mutex> l(quatMutex4);  return activeRUA;   }
bool SensorManager::isLTHActive()   const { std::lock_guard<std::mutex> l(quatMutex5);  return activeLTH;   }
bool SensorManager::isLSHActive()   const { std::lock_guard<std::mutex> l(quatMutex6);  return activeLSH;   }
bool SensorManager::isRTHActive()   const { std::lock_guard<std::mutex> l(quatMutex7);  return activeRTH;   }
bool SensorManager::isRSHActive()   const { std::lock_guard<std::mutex> l(quatMutex8);  return activeRSH;   }
bool SensorManager::isHipsActive()  const { std::lock_guard<std::mutex> l(quatMutex9);  return activeHips;  }
bool SensorManager::isChestActive() const { std::lock_guard<std::mutex> l(quatMutex10); return activeChest; }

// ── calibration ───────────────────────────────────────────────────────────────

void SensorManager::calibrateLFA()  { auto q = getLFAQuat();  std::lock_guard<std::mutex> l(calibMutex1);  calibrationReferenceLFA = glm::normalize(q); hasSmoothedCorrectedLFA = false; stationaryTimerLFA = 0; std::cout << "Calibrated L_FA\n"; }
void SensorManager::calibrateRFA()  { auto q = getRFAQuat();  std::lock_guard<std::mutex> l(calibMutex2);  calibrationReferenceRFA = glm::normalize(q); hasSmoothedCorrectedRFA = false; stationaryTimerRFA = 0; std::cout << "Calibrated R_FA\n"; }
void SensorManager::calibrateLUA()  { auto q = getLUAQuat();  std::lock_guard<std::mutex> l(calibMutex3);  calibrationReference3 = glm::normalize(q); hasSmoothedCorrected3 = false; stationaryTimer3 = 0; std::cout << "Calibrated L_UA\n"; }
void SensorManager::calibrateRUA()  { auto q = getRUAQuat();  std::lock_guard<std::mutex> l(calibMutex4);  calibrationReferenceRUA = glm::normalize(q); hasSmoothedCorrectedRUA = false; stationaryTimerRUA = 0; std::cout << "Calibrated R_UA\n"; }
void SensorManager::calibrateLTH()  { auto q = getLTHQuat();  std::lock_guard<std::mutex> l(calibMutex5);  calibrationReference5 = glm::normalize(q); hasSmoothedCorrected5 = false; stationaryTimer5 = 0; std::cout << "Calibrated L_TH\n"; }
void SensorManager::calibrateLSH()  { auto q = getLSHQuat();  std::lock_guard<std::mutex> l(calibMutex6);  calibrationReference6 = glm::normalize(q); hasSmoothedCorrected6 = false; stationaryTimer6 = 0; std::cout << "Calibrated L_SH\n"; }
void SensorManager::calibrateRTH()  { auto q = getRTHQuat();  std::lock_guard<std::mutex> l(calibMutex7);  calibrationReference7 = glm::normalize(q); hasSmoothedCorrected7 = false; stationaryTimer7 = 0; std::cout << "Calibrated R_TH\n"; }
void SensorManager::calibrateRSH()  { auto q = getRSHQuat();  std::lock_guard<std::mutex> l(calibMutex8);  calibrationReference8 = glm::normalize(q); hasSmoothedCorrected8 = false; stationaryTimer8 = 0; std::cout << "Calibrated R_SH\n"; }
void SensorManager::calibrateHips() { auto q = getHipsQuat(); std::lock_guard<std::mutex> l(calibMutex9);  calibrationReference9 = glm::normalize(q); hasSmoothedCorrected9 = false; stationaryTimer9 = 0; std::cout << "Calibrated HIPS\n"; }
void SensorManager::calibrateChest(){ auto q = getChestQuat();std::lock_guard<std::mutex> l(calibMutex10); calibrationReference10 = glm::normalize(q); hasSmoothedCorrected10 = false; stationaryTimer10 = 0; std::cout << "Calibrated CHEST\n"; }

void SensorManager::toggleQuaternionConvention()
{
    int mode = (quaternionMode.load() + 1) % 4;
    quaternionMode.store(mode);
    std::scoped_lock lock(calibMutex1, calibMutex2, calibMutex3, calibMutex4);
    hasSmoothedCorrectedLFA = hasSmoothedCorrectedRFA = false;
    hasSmoothedCorrected3   = hasSmoothedCorrectedRUA = false;
    std::cout << "Quaternion mode " << (mode + 1) << "/4. Recalibrate.\n";
}

// ── Runtime arm-segment swap ────────────────────────────────────────
//
// Physically, a sensor strapped near the elbow can end up driving the
// "wrong" bone (upper-arm sensor reading mapped to the forearm bone or
// vice-versa) depending on how the suit was put on. Rather than
// recompiling with different sensor labels, these toggles swap the
// *effective* assignment live. Each side (left/right) is independent.
//
// When swapped, the getEffective*Quat() functions below also swap the
// CALIBRATION semantics implicitly — because computeCorrectedQuat is
// keyed off each sensor's own calibration reference, swapping which
// raw quaternion feeds which bone is sufficient; no extra calibration
// bookkeeping is needed.

void SensorManager::toggleArmSwapLeft()
{
    bool now = !armSwapLeft.load();
    armSwapLeft.store(now);
    std::cout << "Left arm upper/forearm sensor swap: "
              << (now ? "ON (swapped)" : "OFF (normal)") << "\n";
}

void SensorManager::toggleArmSwapRight()
{
    bool now = !armSwapRight.load();
    armSwapRight.store(now);
    std::cout << "Right arm upper/forearm sensor swap: "
              << (now ? "ON (swapped)" : "OFF (normal)") << "\n";
}

glm::quat SensorManager::getEffectiveLUAQuat() const
{
    return armSwapLeft.load() ? getCorrectedLFAQuat() : getCorrectedLUAQuat();
}
glm::quat SensorManager::getEffectiveLFAQuat() const
{
    return armSwapLeft.load() ? getCorrectedLUAQuat() : getCorrectedLFAQuat();
}
glm::quat SensorManager::getEffectiveRUAQuat() const
{
    return armSwapRight.load() ? getCorrectedRFAQuat() : getCorrectedRUAQuat();
}
glm::quat SensorManager::getEffectiveRFAQuat() const
{
    return armSwapRight.load() ? getCorrectedRUAQuat() : getCorrectedRFAQuat();
}

// ── corrected getters ─────────────────────────────────────────────────────────

glm::quat SensorManager::getCorrectedLFAQuat() const {
    auto q = getLFAQuat();
    std::lock_guard<std::mutex> l(calibMutex1);
    autoRecalibrate(calibrationReferenceLFA, lastQLFA, stationaryTimerLFA, jitterEmaLFA, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrectedLFA, hasSmoothedCorrectedLFA, computeCorrectedQuat(q, calibrationReferenceLFA), jitterEmaLFA);
}
glm::quat SensorManager::getCorrectedRFAQuat() const {
    auto q = getRFAQuat();
    std::lock_guard<std::mutex> l(calibMutex2);
    autoRecalibrate(calibrationReferenceRFA, lastQRFA, stationaryTimerRFA, jitterEmaRFA, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrectedRFA, hasSmoothedCorrectedRFA, computeCorrectedQuat(q, calibrationReferenceRFA), jitterEmaRFA);
}
glm::quat SensorManager::getCorrectedLUAQuat() const {
    auto q = getLUAQuat();
    std::lock_guard<std::mutex> l(calibMutex3);
    autoRecalibrate(calibrationReference3, lastQ3, stationaryTimer3, jitterEma3, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected3, hasSmoothedCorrected3, computeCorrectedQuat(q, calibrationReference3), jitterEma3);
}
glm::quat SensorManager::getCorrectedRUAQuat() const {
    auto q = getRUAQuat();
    std::lock_guard<std::mutex> l(calibMutex4);
    autoRecalibrate(calibrationReferenceRUA, lastQRUA, stationaryTimerRUA, jitterEmaRUA, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrectedRUA, hasSmoothedCorrectedRUA, computeCorrectedQuat(q, calibrationReferenceRUA), jitterEmaRUA);
}
glm::quat SensorManager::getCorrectedLTHQuat() const {
    auto q = getLTHQuat();
    std::lock_guard<std::mutex> l(calibMutex5);
    autoRecalibrate(calibrationReference5, lastQ5, stationaryTimer5, jitterEma5, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected5, hasSmoothedCorrected5, computeCorrectedQuat(q, calibrationReference5), jitterEma5);
}
glm::quat SensorManager::getCorrectedLSHQuat() const {
    auto q = getLSHQuat();
    std::lock_guard<std::mutex> l(calibMutex6);
    autoRecalibrate(calibrationReference6, lastQ6, stationaryTimer6, jitterEma6, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected6, hasSmoothedCorrected6, computeCorrectedQuat(q, calibrationReference6), jitterEma6);
}
glm::quat SensorManager::getCorrectedRTHQuat() const {
    auto q = getRTHQuat();
    std::lock_guard<std::mutex> l(calibMutex7);
    autoRecalibrate(calibrationReference7, lastQ7, stationaryTimer7, jitterEma7, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected7, hasSmoothedCorrected7, computeCorrectedQuat(q, calibrationReference7), jitterEma7);
}
glm::quat SensorManager::getCorrectedRSHQuat() const {
    auto q = getRSHQuat();
    std::lock_guard<std::mutex> l(calibMutex8);
    autoRecalibrate(calibrationReference8, lastQ8, stationaryTimer8, jitterEma8, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected8, hasSmoothedCorrected8, computeCorrectedQuat(q, calibrationReference8), jitterEma8);
}
glm::quat SensorManager::getCorrectedHipsQuat() const {
    auto q = getHipsQuat();
    std::lock_guard<std::mutex> l(calibMutex9);
    autoRecalibrate(calibrationReference9, lastQ9, stationaryTimer9, jitterEma9, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected9, hasSmoothedCorrected9, computeCorrectedQuat(q, calibrationReference9), jitterEma9);
}
glm::quat SensorManager::getCorrectedChestQuat() const {
    auto q = getChestQuat();
    std::lock_guard<std::mutex> l(calibMutex10);
    autoRecalibrate(calibrationReference10, lastQ10, stationaryTimer10, jitterEma10, q);
    return smoothCorrectedQuatAdaptive(smoothedCorrected10, hasSmoothedCorrected10, computeCorrectedQuat(q, calibrationReference10), jitterEma10);
}