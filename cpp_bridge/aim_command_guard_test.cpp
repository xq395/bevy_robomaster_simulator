#include "aim_command_guard.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

double radians(double degrees) {
    return degrees * talos_bridge::kPi / 180.0;
}

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    require(std::abs(talos_bridge::normalizeYaw(radians(181.0)) -
                     radians(-179.0)) < 1e-12,
            "positive yaw overflow should wrap into [-pi,pi)");
    require(std::abs(talos_bridge::normalizeYaw(radians(-181.0)) -
                     radians(179.0)) < 1e-12,
            "negative yaw overflow should wrap into [-pi,pi)");
    require(std::abs(talos_bridge::normalizeYaw(radians(180.0)) -
                     radians(-180.0)) < 1e-12,
            "positive pi should use the canonical negative endpoint");

    require(talos_bridge::isPlausibleCameraAcquisition(
                radians(-178.9), radians(2.2),
                radians(-169.1), radians(-6.0)),
            "normal in-view armor correction should pass");

    require(!talos_bridge::isPlausibleCameraAcquisition(
                radians(-9.7), radians(52.3),
                radians(-10.0), radians(0.0)),
            "outpost pitch fly-away should be rejected");

    require(talos_bridge::isPlausibleCameraAcquisition(
                radians(-179.0), 0.0, radians(179.0), 0.0),
            "yaw wraparound should use the short angular distance");

    require(!talos_bridge::isPlausibleCameraAcquisition(
                0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
            "non-finite commands should be rejected");

    std::cout << "aim command guard tests passed\n";
    return 0;
}
