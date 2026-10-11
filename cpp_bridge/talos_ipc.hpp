#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace talos_bridge {

constexpr std::uint32_t kImageWidth = 1440;
constexpr std::uint32_t kImageHeight = 1080;
constexpr std::size_t kImageSize =
    static_cast<std::size_t>(kImageWidth) * kImageHeight * 3;

struct Pose {
    std::uint64_t frame_seq = 0;
    std::array<float, 3> position{};
    std::array<float, 4> quaternion_wxyz{};
    std::uint64_t timestamp_ns = 0;
};

struct CameraInfo {
    std::uint64_t timestamp_ns = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::array<double, 5> distortion{};
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

struct RawBundle {
    std::vector<std::uint8_t> rgb;
    std::uint64_t frame_seq = 0;
    std::uint64_t timestamp_ns = 0;
    Pose gimbal;
    Pose odom;
    Pose muzzle;
    Pose camera;
    CameraInfo camera_info;
    std::chrono::steady_clock::time_point captured_at{};
    std::chrono::steady_clock::time_point received_at{};
};

enum class ReadResult {
    NoFrame,
    Frame,
    Incoherent,
    ReleaseConflict,
};

class TalosIpc {
public:
    explicit TalosIpc(const std::filesystem::path& shm_dir);
    ~TalosIpc();
    TalosIpc(const TalosIpc&) = delete;
    TalosIpc& operator=(const TalosIpc&) = delete;

    ReadResult tryRead(RawBundle& output);
    bool autoAimEnabled() const noexcept;
    double heartbeatAgeSeconds() const noexcept;
    void writeGimbalCommand(double yaw_deg, double pitch_deg,
                            double distance_m, bool fire);

private:
    class Mapping;
    Mapping* meta_mapping_ = nullptr;
    Mapping* image_mapping_ = nullptr;
    std::uint8_t* meta_ = nullptr;
    const std::uint8_t* image_pool_ = nullptr;
};

}  // namespace talos_bridge
