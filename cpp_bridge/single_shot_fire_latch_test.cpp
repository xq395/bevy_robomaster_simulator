#include "single_shot_fire_latch.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    talos_bridge::SingleShotFireLatch latch;

    latch.observeTrackingLock(true);
    require(latch.permit(true), "first stable request in a lock should fire");
    require(!latch.permit(true), "a continuous lock must not fire twice");
    require(!latch.permit(false), "a rejected FireGate frame must not fire");
    require(latch.latched(), "brief FireGate rejection must not rearm the lock");

    latch.observeTrackingLock(false);
    require(!latch.latched(), "losing tracking must rearm single-shot mode");
    latch.observeTrackingLock(true);
    require(latch.permit(true), "a new tracking lock should permit one new shot");

    std::cout << "single-shot fire latch tests passed\n";
    return 0;
}
