#pragma once

#include "talos_async_source.hpp"

#include "debug_web/debug_web_publisher.hpp"
#include "shared/types.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace talos_bridge {

class WebDebugSink {
public:
    explicit WebDebugSink(debug_web::Config config, bool auxiliary_views = false);
    ~WebDebugSink();

    WebDebugSink(const WebDebugSink&) = delete;
    WebDebugSink& operator=(const WebDebugSink&) = delete;

    void submit(std::shared_ptr<const FrameBundle> bundle,
                std::vector<hnu25::DetectedArmor> detections,
                std::string state_json,
                int binary_threshold);
    bool ready() const noexcept;

private:
    struct Packet {
        std::shared_ptr<const FrameBundle> bundle;
        std::vector<hnu25::DetectedArmor> detections;
        std::string state_json;
        int binary_threshold = 120;
    };

    void run();

    debug_web::DebugWebPublisher raw_;
    debug_web::DebugWebPublisher annotated_;
    debug_web::DebugWebPublisher gray_;
    debug_web::DebugWebPublisher binary_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::unique_ptr<Packet> latest_;
    bool auxiliary_views_ = false;
    bool stopping_ = false;
    std::thread worker_;
};

}  // namespace talos_bridge
