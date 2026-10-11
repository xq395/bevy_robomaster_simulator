#pragma once

namespace talos_bridge {

class SingleShotFireLatch {
public:
    void observeTrackingLock(bool tracking_lock) noexcept {
        if (!tracking_lock) latched_ = false;
    }

    bool permit(bool fire_requested) noexcept {
        if (!fire_requested || latched_) return false;
        latched_ = true;
        return true;
    }

    bool latched() const noexcept { return latched_; }

private:
    bool latched_ = false;
};

}  // namespace talos_bridge
