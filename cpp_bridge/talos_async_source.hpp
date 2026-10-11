#pragma once

#include "talos_ipc.hpp"

#include <opencv2/core/mat.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace talos_bridge {

struct FrameBundle {
    cv::Mat image_bgr;
    RawBundle metadata;
};

struct SourceStats {
    std::uint64_t captured = 0;
    std::uint64_t overwritten = 0;
    std::uint64_t incoherent = 0;
    std::uint64_t release_conflicts = 0;
    std::uint64_t pool_starved = 0;
    double mean_copy_release_ms = 0.0;
    double max_copy_release_ms = 0.0;
    double mean_convert_ms = 0.0;
    double max_convert_ms = 0.0;
};

class TalosAsyncSource {
public:
    explicit TalosAsyncSource(std::filesystem::path shm_dir,
                              std::size_t pool_size = 3);
    ~TalosAsyncSource();
    TalosAsyncSource(const TalosAsyncSource&) = delete;
    TalosAsyncSource& operator=(const TalosAsyncSource&) = delete;

    void start();
    void stop() noexcept;
    bool waitLatest(std::shared_ptr<const FrameBundle>& output,
                    std::chrono::milliseconds timeout);
    SourceStats stats() const;
    bool autoAimEnabled() const noexcept;
    double heartbeatAgeSeconds() const noexcept;
    void writeGimbalCommand(double yaw_deg, double pitch_deg,
                            double distance_m, bool fire);

private:
    std::shared_ptr<FrameBundle> acquireFreeBundle();
    void captureLoop() noexcept;

    std::filesystem::path shm_dir_;
    std::size_t pool_size_ = 3;
    std::unique_ptr<TalosIpc> ipc_;
    std::vector<std::shared_ptr<FrameBundle>> pool_;

    mutable std::mutex latest_mutex_;
    std::condition_variable latest_ready_;
    std::shared_ptr<FrameBundle> latest_;

    mutable std::mutex stats_mutex_;
    SourceStats stats_;
    double copy_release_sum_ms_ = 0.0;
    double convert_sum_ms_ = 0.0;

    std::atomic<bool> stop_requested_{false};
    std::thread capture_thread_;
    bool started_ = false;
};

}  // namespace talos_bridge
