#include "web_debug_sink.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <utility>

namespace talos_bridge {
namespace {

debug_web::Config viewConfig(debug_web::Config config, const char* suffix) {
    config.shm_name += suffix;
    return config;
}

}  // namespace

WebDebugSink::WebDebugSink(debug_web::Config config, bool auxiliary_views)
    : raw_(config),
      annotated_(viewConfig(config, "_annotated")),
      gray_(viewConfig(config, "_gray")),
      binary_(viewConfig(config, "_binary")),
      auxiliary_views_(auxiliary_views) {
    if (config.enabled) worker_ = std::thread(&WebDebugSink::run, this);
}

WebDebugSink::~WebDebugSink() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        latest_.reset();
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void WebDebugSink::submit(std::shared_ptr<const FrameBundle> bundle,
                          std::vector<hnu25::DetectedArmor> detections,
                          std::string state_json,
                          int binary_threshold) {
    if (!worker_.joinable() || !bundle) return;
    auto packet = std::make_unique<Packet>();
    packet->bundle = std::move(bundle);
    packet->detections = std::move(detections);
    packet->state_json = std::move(state_json);
    packet->binary_threshold = std::clamp(binary_threshold, 0, 255);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(packet);
    }
    cv_.notify_one();
}

bool WebDebugSink::ready() const noexcept {
    return raw_.ready() && annotated_.ready();
}

void WebDebugSink::run() {
    for (;;) {
        std::unique_ptr<Packet> packet;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return stopping_ || latest_; });
            if (stopping_) return;
            packet = std::move(latest_);
        }

        const cv::Mat& frame = packet->bundle->image_bgr;
        raw_.publish(frame, packet->state_json);

        if (annotated_.due()) {
            cv::Mat annotated = frame.clone();
            for (const auto& detection : packet->detections) {
                const cv::Scalar color =
                    detection.color == hnu25::Color::BLUE
                        ? cv::Scalar(255, 120, 40)
                        : cv::Scalar(60, 80, 255);
                cv::rectangle(annotated, detection.box, color, 2);
                for (const auto& point : detection.keypoints)
                    cv::circle(annotated, point, 4,
                               cv::Scalar(80, 255, 150), -1);
                cv::putText(
                    annotated, cv::format("%.2f", detection.confidence),
                    detection.box.tl() + cv::Point(0, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
            }
            annotated_.publish(annotated, packet->state_json);
        }

        if (auxiliary_views_ && (gray_.due() || binary_.due())) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            gray_.publish(gray, packet->state_json);
            cv::Mat binary;
            cv::threshold(gray, binary, packet->binary_threshold, 255,
                          cv::THRESH_BINARY);
            binary_.publish(binary, packet->state_json);
        }
    }
}

}  // namespace talos_bridge
