#include "talos_ipc.hpp"

#include <windows.h>
#include <intrin.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <system_error>

namespace talos_bridge {
namespace {

constexpr std::size_t kMetaSize = 3712;
constexpr std::size_t kImagePoolSize = kImageSize * 3;
constexpr std::uint32_t kMagic = 0x54414C05;
constexpr std::uint32_t kVersion = 2;
constexpr std::uint8_t kFlagNew = 0x80;
constexpr std::uint8_t kIndexMask = 0x03;

constexpr std::size_t kImageTbOffset = 64;
constexpr std::size_t kImageSlotOffset = kImageTbOffset + 64;
constexpr std::array<std::size_t, 4> kPoseTbOffsets{256, 512, 768, 1024};
constexpr std::size_t kGimbalCmdTbOffset = 1536;
constexpr std::size_t kCameraInfoOffset = 1728;
constexpr std::size_t kRuntimeStateOffset = 3648;

template <typename T>
T loadUnaligned(const std::uint8_t* data, std::size_t offset) {
    T value{};
    std::memcpy(&value, data + offset, sizeof(value));
    return value;
}

template <typename T>
void storeUnaligned(std::uint8_t* data, std::size_t offset, const T& value) {
    std::memcpy(data + offset, &value, sizeof(value));
}

std::uint8_t atomicLoad(const std::uint8_t* address) noexcept {
    auto* value = reinterpret_cast<volatile char*>(const_cast<std::uint8_t*>(address));
    return static_cast<std::uint8_t>(_InterlockedCompareExchange8(value, 0, 0));
}

bool atomicCompareExchange(std::uint8_t* address,
                           std::uint8_t expected,
                           std::uint8_t desired) noexcept {
    auto* value = reinterpret_cast<volatile char*>(address);
    return static_cast<std::uint8_t>(
               _InterlockedCompareExchange8(value, static_cast<char>(desired),
                                             static_cast<char>(expected))) == expected;
}

std::uint8_t atomicExchange(std::uint8_t* address, std::uint8_t desired) noexcept {
    return static_cast<std::uint8_t>(
        _InterlockedExchange8(reinterpret_cast<volatile char*>(address),
                              static_cast<char>(desired)));
}

Pose readPose(const std::uint8_t* meta, std::size_t tb_offset, std::uint8_t state) {
    const std::size_t slot = tb_offset + 64 + (state & kIndexMask) * 64;
    Pose pose;
    pose.frame_seq = loadUnaligned<std::uint64_t>(meta, slot);
    std::memcpy(pose.position.data(), meta + slot + 8, sizeof(float) * 3);
    std::memcpy(pose.quaternion_wxyz.data(), meta + slot + 20, sizeof(float) * 4);
    pose.timestamp_ns = loadUnaligned<std::uint64_t>(meta, slot + 40);
    return pose;
}

CameraInfo readCameraInfo(const std::uint8_t* meta) {
    CameraInfo value;
    std::size_t offset = kCameraInfoOffset;
    value.timestamp_ns = loadUnaligned<std::uint64_t>(meta, offset);
    offset += 8;
    value.fx = loadUnaligned<double>(meta, offset); offset += 8;
    value.fy = loadUnaligned<double>(meta, offset); offset += 8;
    value.cx = loadUnaligned<double>(meta, offset); offset += 8;
    value.cy = loadUnaligned<double>(meta, offset); offset += 8;
    std::memcpy(value.distortion.data(), meta + offset, sizeof(double) * 5);
    offset += sizeof(double) * 5;
    value.width = loadUnaligned<std::uint32_t>(meta, offset); offset += 4;
    value.height = loadUnaligned<std::uint32_t>(meta, offset);
    return value;
}

bool releaseTripleBuffer(std::uint8_t* meta, std::size_t tb_offset,
                         std::uint8_t observed_state) noexcept {
    const std::uint8_t ready_idx = observed_state & kIndexMask;
    const std::uint8_t previous_read_idx = meta[tb_offset + 2] & kIndexMask;
    if (!atomicCompareExchange(meta + tb_offset, observed_state, previous_read_idx))
        return false;
    meta[tb_offset + 2] = ready_idx;
    return true;
}

std::chrono::steady_clock::time_point steadyTimeFromUnixNs(std::uint64_t timestamp_ns) {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto system_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (timestamp_ns == 0 || timestamp_ns > static_cast<std::uint64_t>(system_now_ns + 100000000))
        return steady_now;
    const auto age_ns = system_now_ns - static_cast<std::int64_t>(timestamp_ns);
    if (age_ns < 0 || age_ns > 5000000000LL) return steady_now;
    return steady_now - std::chrono::nanoseconds(age_ns);
}

std::string windowsMessage(const char* operation) {
    return std::string(operation) + " failed with Windows error " +
           std::to_string(GetLastError());
}

}  // namespace

class TalosIpc::Mapping {
public:
    Mapping(const std::filesystem::path& path, std::size_t minimum_size, bool writable) {
        const DWORD access = writable ? (GENERIC_READ | GENERIC_WRITE) : GENERIC_READ;
        file_ = CreateFileW(path.c_str(), access,
                            FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                            nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file_ == INVALID_HANDLE_VALUE) throw std::runtime_error(windowsMessage("CreateFileW"));
        LARGE_INTEGER size{};
        if (!GetFileSizeEx(file_, &size) || size.QuadPart < static_cast<LONGLONG>(minimum_size))
            throw std::runtime_error("Talos mapping has an invalid size: " + path.string());
        const DWORD protection = writable ? PAGE_READWRITE : PAGE_READONLY;
        mapping_ = CreateFileMappingW(file_, nullptr, protection, 0, 0, nullptr);
        if (!mapping_) throw std::runtime_error(windowsMessage("CreateFileMappingW"));
        const DWORD map_access = writable ? FILE_MAP_ALL_ACCESS : FILE_MAP_READ;
        data_ = static_cast<std::uint8_t*>(MapViewOfFile(mapping_, map_access, 0, 0, minimum_size));
        if (!data_) throw std::runtime_error(windowsMessage("MapViewOfFile"));
    }

    ~Mapping() {
        if (data_) UnmapViewOfFile(data_);
        if (mapping_) CloseHandle(mapping_);
        if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_);
    }

    std::uint8_t* data() noexcept { return data_; }

private:
    HANDLE file_ = INVALID_HANDLE_VALUE;
    HANDLE mapping_ = nullptr;
    std::uint8_t* data_ = nullptr;
};

TalosIpc::TalosIpc(const std::filesystem::path& shm_dir) {
    try {
        meta_mapping_ = new Mapping(shm_dir / L"talos_ipc_meta", kMetaSize, true);
        image_mapping_ = new Mapping(shm_dir / L"talos_ipc_image_pool", kImagePoolSize, false);
        meta_ = meta_mapping_->data();
        image_pool_ = image_mapping_->data();
        const auto magic = loadUnaligned<std::uint32_t>(meta_, 0);
        const auto version = loadUnaligned<std::uint32_t>(meta_, 4);
        const auto width = loadUnaligned<std::uint32_t>(meta_, 24);
        const auto height = loadUnaligned<std::uint32_t>(meta_, 28);
        if (magic != kMagic || version != kVersion)
            throw std::runtime_error("unsupported Talos shared-memory protocol");
        if (width != kImageWidth || height != kImageHeight)
            throw std::runtime_error("Talos image resolution must be 1440x1080");
    } catch (...) {
        delete image_mapping_;
        delete meta_mapping_;
        image_mapping_ = nullptr;
        meta_mapping_ = nullptr;
        throw;
    }
}

TalosIpc::~TalosIpc() {
    delete image_mapping_;
    delete meta_mapping_;
}

ReadResult TalosIpc::tryRead(RawBundle& output) {
    const std::uint8_t image_state = atomicLoad(meta_ + kImageTbOffset);
    if ((image_state & kFlagNew) == 0) return ReadResult::NoFrame;
    std::array<std::uint8_t, 4> pose_states{};
    for (std::size_t i = 0; i < pose_states.size(); ++i) {
        pose_states[i] = atomicLoad(meta_ + kPoseTbOffsets[i]);
        if ((pose_states[i] & kFlagNew) == 0) return ReadResult::NoFrame;
    }

    const std::size_t image_slot =
        kImageSlotOffset + (image_state & kIndexMask) * 32;
    const auto seq = loadUnaligned<std::uint64_t>(meta_, image_slot);
    const auto timestamp_ns = loadUnaligned<std::uint64_t>(meta_, image_slot + 8);
    const auto width = loadUnaligned<std::uint32_t>(meta_, image_slot + 16);
    const auto height = loadUnaligned<std::uint32_t>(meta_, image_slot + 20);
    const auto buffer_id = meta_[image_slot + 24];
    const auto format = meta_[image_slot + 25];
    const auto release_observed = [&] {
        bool released = true;
        for (std::size_t i = 0; i < pose_states.size(); ++i)
            released = releaseTripleBuffer(meta_, kPoseTbOffsets[i], pose_states[i]) && released;
        released = releaseTripleBuffer(meta_, kImageTbOffset, image_state) && released;
        return released;
    };
    if (width != kImageWidth || height != kImageHeight ||
        buffer_id >= 3 || format != 0) {
        release_observed();
        return ReadResult::Incoherent;
    }

    const std::array<Pose, 4> poses{
        readPose(meta_, kPoseTbOffsets[0], pose_states[0]),
        readPose(meta_, kPoseTbOffsets[1], pose_states[1]),
        readPose(meta_, kPoseTbOffsets[2], pose_states[2]),
        readPose(meta_, kPoseTbOffsets[3], pose_states[3]),
    };
    for (const auto& pose : poses) {
        if (pose.frame_seq != seq || pose.timestamp_ns != timestamp_ns) {
            release_observed();
            return ReadResult::Incoherent;
        }
    }

    if (output.rgb.size() != kImageSize) output.rgb.resize(kImageSize);
    const auto received_at = std::chrono::steady_clock::now();
    std::memcpy(output.rgb.data(), image_pool_ + buffer_id * kImageSize, kImageSize);
    output.frame_seq = seq;
    output.timestamp_ns = timestamp_ns;
    output.gimbal = poses[0];
    output.odom = poses[1];
    output.muzzle = poses[2];
    output.camera = poses[3];
    output.camera_info = readCameraInfo(meta_);
    output.captured_at = steadyTimeFromUnixNs(timestamp_ns);
    output.received_at = received_at;

    for (std::size_t i = 0; i < pose_states.size(); ++i) {
        if (!releaseTripleBuffer(meta_, kPoseTbOffsets[i], pose_states[i]))
            return ReadResult::ReleaseConflict;
    }
    if (!releaseTripleBuffer(meta_, kImageTbOffset, image_state))
        return ReadResult::ReleaseConflict;
    return ReadResult::Frame;
}

bool TalosIpc::autoAimEnabled() const noexcept {
    return meta_[kRuntimeStateOffset + 8] == 1;
}

double TalosIpc::heartbeatAgeSeconds() const noexcept {
    const auto heartbeat = loadUnaligned<std::uint64_t>(meta_, 16);
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (heartbeat == 0 || heartbeat > static_cast<std::uint64_t>(now))
        return std::numeric_limits<double>::infinity();
    return static_cast<double>(now - static_cast<std::int64_t>(heartbeat)) / 1e9;
}

void TalosIpc::writeGimbalCommand(double yaw_deg, double pitch_deg,
                                  double distance_m, bool fire) {
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg) ||
        !std::isfinite(distance_m) || distance_m <= 0.0)
        throw std::invalid_argument("invalid Talos gimbal command");
    const auto write_idx = meta_[kGimbalCmdTbOffset + 1] & kIndexMask;
    if (write_idx >= 3) throw std::runtime_error("invalid Talos gimbal write index");
    const std::size_t slot = kGimbalCmdTbOffset + 64 + write_idx * 32;
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    const float yaw = static_cast<float>(yaw_deg);
    const float pitch = static_cast<float>(pitch_deg);
    const float distance = static_cast<float>(distance_m);
    storeUnaligned(meta_, slot, now_ns);
    storeUnaligned(meta_, slot + 8, yaw);
    storeUnaligned(meta_, slot + 12, pitch);
    storeUnaligned(meta_, slot + 16, distance);
    meta_[slot + 20] = fire ? 1 : 0;
    std::memset(meta_ + slot + 21, 0, 11);
    const auto old = atomicExchange(meta_ + kGimbalCmdTbOffset,
                                    write_idx | kFlagNew);
    meta_[kGimbalCmdTbOffset + 1] = old & kIndexMask;
}

}  // namespace talos_bridge
