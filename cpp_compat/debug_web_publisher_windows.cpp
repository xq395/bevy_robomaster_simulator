#include "debug_web/debug_web_publisher.hpp"

#include "debug_web/shm_protocol.hpp"

#include <opencv2/imgcodecs.hpp>

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <new>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace debug_web {
namespace {

std::string safeName(std::string name) {
    if (!name.empty() && name.front() == '/') name.erase(name.begin());
    std::replace(name.begin(), name.end(), '/', '_');
    std::replace(name.begin(), name.end(), '\\', '_');
    return name.empty() ? "hnu_vision_debug" : name;
}

std::filesystem::path debugDirectory() {
    std::wstring value(32768, L'\0');
    const DWORD length = GetEnvironmentVariableW(
        L"HNU_DEBUG_WEB_DIR", value.data(), static_cast<DWORD>(value.size()));
    if (length > 0 && length < value.size()) {
        value.resize(length);
        return std::filesystem::absolute(std::filesystem::path(value));
    }
    return std::filesystem::temp_directory_path() / "hnu_vision_debug";
}

std::filesystem::path mappingPath(const std::string& shm_name) {
    return debugDirectory() / (safeName(shm_name) + ".mmap");
}

void atomicWrite(const std::filesystem::path& path, const std::string& content) {
    const auto temporary = path.wstring() + L".tmp";
    {
        std::ofstream output(std::filesystem::path(temporary),
                             std::ios::binary | std::ios::trunc);
        output << content;
        output.flush();
        if (!output) throw std::runtime_error("cannot write web debug state");
    }
    if (!MoveFileExW(temporary.c_str(), path.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        throw std::system_error(
            static_cast<int>(GetLastError()), std::system_category(),
            "cannot replace web debug state");
    }
}

}  // namespace

std::string statePathForShm(const std::string& shm_name) {
    return (debugDirectory() / (safeName(shm_name) + ".json")).string();
}

struct DebugWebPublisher::Impl {
    explicit Impl(Config value) : config(std::move(value)) {
        if (!config.enabled) return;
        config.shm_name = "/" + safeName(config.shm_name);
        config.quality = std::clamp(config.quality, 1, 100);

        const auto directory = debugDirectory();
        std::filesystem::create_directories(directory);
        path = mappingPath(config.shm_name);
        const std::uint64_t mapping_size =
            static_cast<std::uint64_t>(sizeof(ShmHeader) + kJpegCapacity);

        file = CreateFileW(
            path.c_str(), GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
            nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file == INVALID_HANDLE_VALUE) return;

        LARGE_INTEGER size{};
        size.QuadPart = static_cast<LONGLONG>(mapping_size);
        if (!SetFilePointerEx(file, size, nullptr, FILE_BEGIN) || !SetEndOfFile(file))
            return;
        mapping_handle = CreateFileMappingW(
            file, nullptr, PAGE_READWRITE,
            static_cast<DWORD>(mapping_size >> 32),
            static_cast<DWORD>(mapping_size & 0xffffffffULL), nullptr);
        if (!mapping_handle) return;
        mapping = MapViewOfFile(
            mapping_handle, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0,
            static_cast<SIZE_T>(mapping_size));
        if (!mapping) return;

        header = new (mapping) ShmHeader{};
        std::memcpy(header->magic, kMagic, sizeof(kMagic));
        header->version = kVersion;
        header->header_size = sizeof(ShmHeader);
        header->capacity = kJpegCapacity;
        header->sequence.store(0, std::memory_order_release);
        ready = true;
    }

    ~Impl() {
        if (mapping) UnmapViewOfFile(mapping);
        if (mapping_handle) CloseHandle(mapping_handle);
        if (file != INVALID_HANDLE_VALUE) CloseHandle(file);
    }

    void publish(const cv::Mat& frame, const std::string& state_json) {
        if (!ready || frame.empty()) return;
        const auto now = std::chrono::steady_clock::now();
        if (config.publish_hz > 0.0 &&
            last_publish.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_publish).count() <
                1.0 / config.publish_hz)
            return;

        std::vector<unsigned char> jpeg;
        if (!cv::imencode(".jpg", frame, jpeg,
                          {cv::IMWRITE_JPEG_QUALITY, config.quality}) ||
            jpeg.size() > kJpegCapacity)
            return;

        const std::uint64_t odd =
            header->sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        header->timestamp_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        header->jpeg_size = static_cast<std::uint32_t>(jpeg.size());
        header->width = static_cast<std::uint32_t>(frame.cols);
        header->height = static_cast<std::uint32_t>(frame.rows);
        std::memcpy(static_cast<unsigned char*>(mapping) + sizeof(ShmHeader),
                    jpeg.data(), jpeg.size());
        std::atomic_thread_fence(std::memory_order_release);
        header->sequence.store(odd + 1, std::memory_order_release);

        atomicWrite(std::filesystem::path(statePathForShm(config.shm_name)),
                    state_json);
        last_publish = now;
    }

    bool due() const {
        if (!ready) return false;
        if (!(config.publish_hz > 0.0) ||
            last_publish.time_since_epoch().count() == 0)
            return true;
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - last_publish).count()
            >= 1.0 / config.publish_hz;
    }

    Config config;
    std::filesystem::path path;
    HANDLE file = INVALID_HANDLE_VALUE;
    HANDLE mapping_handle = nullptr;
    void* mapping = nullptr;
    ShmHeader* header = nullptr;
    bool ready = false;
    std::chrono::steady_clock::time_point last_publish{};
};

DebugWebPublisher::DebugWebPublisher(Config config) noexcept {
    try {
        impl_ = std::make_unique<Impl>(std::move(config));
    } catch (...) {
    }
}

DebugWebPublisher::~DebugWebPublisher() = default;
DebugWebPublisher::DebugWebPublisher(DebugWebPublisher&&) noexcept = default;
DebugWebPublisher& DebugWebPublisher::operator=(DebugWebPublisher&&) noexcept =
    default;

void DebugWebPublisher::publish(const cv::Mat& frame,
                                const std::string& state_json) noexcept {
    try {
        if (impl_) impl_->publish(frame, state_json);
    } catch (...) {
    }
}

bool DebugWebPublisher::ready() const noexcept {
    return impl_ && impl_->ready;
}

bool DebugWebPublisher::due() const noexcept {
    try {
        return impl_ && impl_->due();
    } catch (...) {
        return false;
    }
}

}  // namespace debug_web
