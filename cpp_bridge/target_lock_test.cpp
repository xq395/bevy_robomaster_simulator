#include "target_lock.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main() {
    talos_bridge::TargetLock lock("unused.request");

    require(lock.apply("lock_current", {3, 4}, 3), "locking changes state");
    require(lock.lockedLabel() == 3, "current tracker label is locked");
    require(lock.allows(3), "locked label is accepted");
    require(!lock.allows(4), "higher-confidence competing label is rejected");

    require(lock.apply("next", {4, 3}, 3), "switching changes locked label");
    require(lock.lockedLabel() == 4, "next visible label is selected");
    require(!lock.allows(3) && lock.allows(4), "filter follows switched label");

    lock.observeVisible({});
    require(lock.lockedLabel() == 4, "temporary target loss preserves lock");
    require(lock.apply("unlock", {}, std::nullopt), "unlock changes state");
    require(lock.allows(3) && lock.allows(4), "automatic mode accepts all labels");

    std::cout << "target lock tests passed\n";
    return 0;
}
