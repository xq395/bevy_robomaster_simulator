#include "talos_async_source.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

namespace {

template <typename T>
void store(std::vector<std::uint8_t>& bytes, std::size_t offset, const T& value) {
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

template <typename T>
T load(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
    T value{};
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
}

bool require(bool condition, const char* message) {
    if (!condition) std::cerr << "FAILED: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    const auto suffix = std::to_string(GetCurrentProcessId());
    const auto directory = std::filesystem::temp_directory_path() /
                           ("talos_async_source_test_" + suffix);
    std::filesystem::create_directories(directory);
    try {
        std::vector<std::uint8_t> meta(3712, 0);
        const std::uint32_t magic = 0x54414C05;
        const std::uint32_t version = 2;
        const std::uint32_t width = talos_bridge::kImageWidth;
        const std::uint32_t height = talos_bridge::kImageHeight;
        const std::uint64_t sequence = 7;
        const auto timestamp = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        store(meta, 0, magic);
        store(meta, 4, version);
        store(meta, 16, timestamp);
        store(meta, 24, width);
        store(meta, 28, height);

        constexpr std::uint8_t ready = 0x80;
        constexpr std::size_t image_tb = 64;
        meta[image_tb] = ready;
        meta[image_tb + 1] = 1;
        meta[image_tb + 2] = 2;
        const std::size_t image_slot = image_tb + 64;
        store(meta, image_slot, sequence);
        store(meta, image_slot + 8, timestamp);
        store(meta, image_slot + 16, width);
        store(meta, image_slot + 20, height);
        meta[image_slot + 24] = 0;
        meta[image_slot + 25] = 0;

        constexpr std::size_t pose_offsets[] = {256, 512, 768, 1024};
        for (const auto pose_tb : pose_offsets) {
            meta[pose_tb] = ready;
            meta[pose_tb + 1] = 1;
            meta[pose_tb + 2] = 2;
            const std::size_t slot = pose_tb + 64;
            store(meta, slot, sequence);
            const float one = 1.0F;
            store(meta, slot + 20, one);
            store(meta, slot + 40, timestamp);
        }

        const double fx = 1000.0, fy = 1000.0, cx = 720.0, cy = 540.0;
        store(meta, 1728, timestamp);
        store(meta, 1736, fx);
        store(meta, 1744, fy);
        store(meta, 1752, cx);
        store(meta, 1760, cy);
        store(meta, 1808, width);
        store(meta, 1812, height);

        std::vector<std::uint8_t> image(talos_bridge::kImageSize * 3, 0);
        image[0] = 10;
        image[1] = 20;
        image[2] = 30;
        {
            std::ofstream output(directory / "talos_ipc_meta", std::ios::binary);
            output.write(reinterpret_cast<const char*>(meta.data()),
                         static_cast<std::streamsize>(meta.size()));
        }
        {
            std::ofstream output(directory / "talos_ipc_image_pool", std::ios::binary);
            output.write(reinterpret_cast<const char*>(image.data()),
                         static_cast<std::streamsize>(image.size()));
        }

        talos_bridge::TalosAsyncSource source(directory);
        source.start();
        std::shared_ptr<const talos_bridge::FrameBundle> frame;
        bool ok = source.waitLatest(frame, std::chrono::seconds(2));
        ok = require(ok && frame, "async frame available") && ok;
        if (frame) {
            ok = require(frame->metadata.frame_seq == sequence, "frame sequence preserved") && ok;
            const auto pixel = frame->image_bgr.at<cv::Vec3b>(0, 0);
            ok = require(pixel[0] == 30 && pixel[1] == 20 && pixel[2] == 10,
                         "RGB converted to BGR after copy") && ok;
        }
        source.writeGimbalCommand(12.5, -3.25, 4.5, false);
        source.stop();

        std::vector<std::uint8_t> released(3712, 0);
        {
            std::ifstream input(directory / "talos_ipc_meta", std::ios::binary);
            input.read(reinterpret_cast<char*>(released.data()),
                       static_cast<std::streamsize>(released.size()));
        }
        ok = require((released[image_tb] & ready) == 0, "image released immediately") && ok;
        for (const auto pose_tb : pose_offsets)
            ok = require((released[pose_tb] & ready) == 0, "pose released immediately") && ok;
        constexpr std::size_t command_tb = 1536;
        constexpr std::size_t command_slot = command_tb + 64;
        ok = require((released[command_tb] & ready) != 0, "command published") && ok;
        ok = require(load<float>(released, command_slot + 8) == 12.5F,
                     "command yaw preserved") && ok;
        ok = require(load<float>(released, command_slot + 12) == -3.25F,
                     "command pitch preserved") && ok;
        ok = require(load<float>(released, command_slot + 16) == 4.5F,
                     "command distance preserved") && ok;
        ok = require(released[command_slot + 20] == 0, "command fire disabled") && ok;
        const auto stats = source.stats();
        ok = require(stats.captured == 1, "capture statistics updated") && ok;
        std::filesystem::remove_all(directory);
        return ok ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        std::filesystem::remove_all(directory);
        return 1;
    }
}
