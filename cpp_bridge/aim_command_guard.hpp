#pragma once

#include <cmath>

namespace talos_bridge {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxAbsolutePitchRad = 45.0 * kPi / 180.0;
constexpr double kMaxAcquisitionPitchJumpRad = 30.0 * kPi / 180.0;
constexpr double kMaxAcquisitionYawJumpRad = 45.0 * kPi / 180.0;

inline double normalizeYaw(double angle) {
    if (!std::isfinite(angle)) return angle;
    double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    if (wrapped < 0.0) wrapped += 2.0 * kPi;
    return wrapped - kPi;
}

inline double wrappedAngleDifference(double target, double feedback) {
    return normalizeYaw(target - feedback);
}

inline bool isPlausibleCameraAcquisition(double aim_yaw, double aim_pitch,
                                         double feedback_yaw,
                                         double feedback_pitch) {
    if (!std::isfinite(aim_yaw) || !std::isfinite(aim_pitch) ||
        !std::isfinite(feedback_yaw) || !std::isfinite(feedback_pitch)) {
        return false;
    }

    return std::abs(aim_pitch) <= kMaxAbsolutePitchRad &&
           std::abs(aim_pitch - feedback_pitch) <=
               kMaxAcquisitionPitchJumpRad &&
           std::abs(wrappedAngleDifference(aim_yaw, feedback_yaw)) <=
               kMaxAcquisitionYawJumpRad;
}

}  // namespace talos_bridge