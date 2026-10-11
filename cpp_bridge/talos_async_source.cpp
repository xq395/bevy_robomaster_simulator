#include "talos_async_source.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace talos_bridge {

TalosAsyncSource::TalosAsyncSource(std::filesystem::path shm_dir,
                                   std::size_t pool_size)
    : shm_dir_(std::move(shm_dir)), pool_size_(std::max<std::size_t>(3, pool_size)) {}

TalosAsyncSource::~TalosAsyncSource() { stop(); }

void TalosAsyncSource::start() {
    if (started_) throw std::logic_error("TalosAsyncSource already started");
    ipc_ = std::make_unique<TalosIpc>(shm_dir_);
    pool_.clear();
    pool_.reserve(pool_size_);
    for (std::size_t i = 0; i < pool_size_; ++i) {
        auto bundle = std::make_shared<FrameBundle>();
        bundle->metadata.rgb.resize(kImageSize);
        bundle->image_bgr.create(static_cast<int>(kImageHeight),
                                 static_cast<int>(kImageWidth), CV_8UC3);
        pool_.push_back(std::move(bundle));
    }
    stop_requested_.store(false, std::memory_order_release);
    started_ = true;
    capture_thread_ = std::thread(&TalosAsyncSource::captureLoop, this);
}

void TalosAsyncSource::stop() noexcept {
    if (!started_) return;
    stop_requested_.store(true, std::memory_order_release);
    latest_ready_.notify_all();
    if (capture_thread_.joinable()) capture_thread_.join();
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_.reset();
    }
    pool_.clear();
    ipc_.reset();
    started_ = false;
}

bool TalosAsyncSource::waitLatest(std::shared_ptr<const FrameBundle>& output,
                                  std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(latest_mutex_);
    if (!latest_ready_.wait_for(lock, timeout, [this] {
            return latest_ || stop_requested_.load(std::memory_order_acquire);
        }))
        return false;
    if (!latest_) return false;
    output = std::move(latest_);
    latest_.reset();
    return true;
}

SourceStats TalosAsyncSource::stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

bool TalosAsyncSource::autoAimEnabled() const noexcept {
    return ipc_ && ipc_->autoAimEnabled();
}

double TalosAsyncSource::heartbeatAgeSeconds() const noexcept {
    return ipc_ ? ipc_->heartbeatAgeSeconds() : 1e9;
}

void TalosAsyncSource::writeGimbalCommand(double yaw_deg, double pitch_deg,
                                          double distance_m, bool fire) {
    if (!ipc_) throw std::logic_error("TalosAsyncSource is not started");
    ipc_->writeGimbalCommand(yaw_deg, pitch_deg, distance_m, fire);
}

std::shared_ptr<FrameBundle> TalosAsyncSource::acquireFreeBundle() {
    for (const auto& bundle : pool_) {
        if (bundle.use_count() == 1) return bundle;
    }
    return {};
}

void TalosAsyncSource::captureLoop() noexcept {
    try {
        while (!stop_requested_.load(std::memory_order_acquire)) {
            auto bundle = acquireFreeBundle();
            if (!bundle) {
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    ++stats_.pool_starved;
                }
                std::this_thread::yield();
                continue;
            }

            const auto before_read = std::chrono::steady_clock::now();
            const auto result = ipc_->tryRead(bundle->metadata);
            const auto after_release = std::chrono::steady_clock::now();
            if (result == ReadResult::NoFrame) {
                std::this_thread::sleep_for(std::chrono::microseconds(250));
                continue;
            }
            if (result == ReadResult::Incoherent) {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.incoherent;
                std::this_thread::yield();
                continue;
            }
            if (result == ReadResult::ReleaseConflict) {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.release_conflicts;
                std::this_thread::yield();
                continue;
            }

            const cv::Mat rgb(static_cast<int>(kImageHeight),
                              static_cast<int>(kImageWidth), CV_8UC3,
                              bundle->metadata.rgb.data());
            cv::cvtColor(rgb, bundle->image_bgr, cv::COLOR_RGB2BGR);
            const auto after_convert = std::chrono::steady_clock::now();
            const double copy_release_ms =
                std::chrono::duration<double, std::milli>(after_release - before_read).count();
            const double convert_ms =
                std::chrono::duration<double, std::milli>(after_convert - after_release).count();

            bool overwritten = false;
            {
                std::lock_guard<std::mutex> lock(latest_mutex_);
                overwritten = static_cast<bool>(latest_);
                latest_ = bundle;
            }
            latest_ready_.notify_one();

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.captured;
                if (overwritten) ++stats_.overwritten;
                copy_release_sum_ms_ += copy_release_ms;
                convert_sum_ms_ += convert_ms;
                stats_.mean_copy_release_ms = copy_release_sum_ms_ / stats_.captured;
                stats_.mean_convert_ms = convert_sum_ms_ / stats_.captured;
                stats_.max_copy_release_ms = std::max(stats_.max_copy_release_ms,
                                                      copy_release_ms);
                stats_.max_convert_ms = std::max(stats_.max_convert_ms, convert_ms);
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "[TalosCapture] fatal: " << error.what() << '\n';
        stop_requested_.store(true, std::memory_order_release);
        latest_ready_.notify_all();
    }
}

}  // namespace talos_bridge
