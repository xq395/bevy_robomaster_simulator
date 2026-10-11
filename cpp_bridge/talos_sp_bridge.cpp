#include "talos_async_source.hpp"
#include "aim_command_guard.hpp"
#include "single_shot_fire_latch.hpp"
#include "target_lock.hpp"
#include "web_debug_sink.hpp"

#include "aim/fire_gate.hpp"
#include "detection/detector.hpp"
#include "pipeline/runtime_tuning.hpp"
#include "pipeline/sp_standard_config.hpp"
#include "planner/mpc_planner.hpp"
#include "sp_core/sp_core.hpp"
#include "telemetry/telemetry.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
constexpr double kRadiansToDegrees = 57.295779513082320876;

volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }

struct Options {
    std::filesystem::path config_path;
    std::filesystem::path shm_dir = L"F:\\talos_shm";
    std::filesystem::path model_path;
    bool send = false;
    bool allow_fire = false;
    bool single_shot = false;
    bool probe_hold = false;
    bool capture_only = false;
    bool web = false;
    bool web_auxiliary_views = false;
    bool telemetry = true;
    double web_hz = 15.0;
    int fire_max_age_ms = 0;
    std::uint64_t max_frames = 0;
    int processing_delay_ms = 0;
    int cpu_cores = 0;
    bool low_priority = false;
};

void usage() {
    std::cerr
        << "usage: talos_sp_bridge <config.yaml> [--shm-dir DIR] [--send] "
           "[--allow-fire] [--single-shot] [--fire-max-age-ms N] "
           "[--probe-hold] [--capture-only] "
           "[--max-frames N] [--processing-delay-ms N] "
           "[--cpu-cores N] [--low-priority] [--model PATH] "
           "[--web] [--web-aux] [--web-hz HZ] [--no-telemetry]\n";
}

Options parseOptions(int argc, char** argv) {
    if (argc < 2) {
        usage();
        throw std::invalid_argument("config path is required");
    }
    Options value;
    value.config_path = std::filesystem::absolute(argv[1]);
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("missing value after " + arg);
            return argv[i];
        };
        if (arg == "--shm-dir") value.shm_dir = next();
        else if (arg == "--model") value.model_path = std::filesystem::absolute(next());
        else if (arg == "--send") value.send = true;
        else if (arg == "--allow-fire") value.allow_fire = true;
        else if (arg == "--single-shot") value.single_shot = true;
        else if (arg == "--fire-max-age-ms")
            value.fire_max_age_ms = std::stoi(next());
        else if (arg == "--probe-hold") value.probe_hold = true;
        else if (arg == "--capture-only") value.capture_only = true;
        else if (arg == "--web") value.web = true;
        else if (arg == "--web-aux") value.web_auxiliary_views = true;
        else if (arg == "--no-telemetry") value.telemetry = false;
        else if (arg == "--web-hz") value.web_hz = std::stod(next());
        else if (arg == "--max-frames") value.max_frames = std::stoull(next());
        else if (arg == "--processing-delay-ms") value.processing_delay_ms = std::stoi(next());
        else if (arg == "--cpu-cores") value.cpu_cores = std::stoi(next());
        else if (arg == "--low-priority") value.low_priority = true;
        else throw std::invalid_argument("unknown argument: " + arg);
    }
    if (value.allow_fire && !value.send)
        throw std::invalid_argument("--allow-fire requires --send");
    if (value.single_shot && !value.allow_fire)
        throw std::invalid_argument("--single-shot requires --allow-fire");
    if (value.fire_max_age_ms < 0 || value.fire_max_age_ms > 1000)
        throw std::invalid_argument("fire max age must be in [0,1000] ms");
    if (value.fire_max_age_ms > 0 && !value.allow_fire)
        throw std::invalid_argument("--fire-max-age-ms requires --allow-fire");
    if (value.processing_delay_ms < 0 || value.processing_delay_ms > 10000)
        throw std::invalid_argument("processing delay must be in [0,10000] ms");
    if (value.cpu_cores < 0 || value.cpu_cores > 63)
        throw std::invalid_argument("cpu cores must be in [0,63]");
    if (!std::isfinite(value.web_hz) || value.web_hz <= 0.0 ||
        value.web_hz > 30.0)
        throw std::invalid_argument("web hz must be in (0,30]");
    if (value.web && value.capture_only)
        throw std::invalid_argument("--web is unavailable with --capture-only");
    return value;
}

void applyScheduling(const Options& options) {
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    const int available = static_cast<int>(std::min<DWORD>(info.dwNumberOfProcessors, 63));
    if (options.cpu_cores > 0) {
        const int count = std::min(options.cpu_cores, available);
        const DWORD_PTR low_bits = (static_cast<DWORD_PTR>(1) << count) - 1;
        const DWORD_PTR mask = low_bits << (available - count);
        if (!SetProcessAffinityMask(GetCurrentProcess(), mask))
            throw std::runtime_error("SetProcessAffinityMask failed");
        std::cout << "cpu_affinity=highest_" << count << "_of_" << available << '\n';
    }
    if (options.low_priority) {
        if (!SetPriorityClass(GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS))
            throw std::runtime_error("SetPriorityClass failed");
        std::cout << "priority=below_normal\n";
    }
}

Eigen::Quaterniond quaternion(const talos_bridge::Pose& pose) {
    const auto& q = pose.quaternion_wxyz;
    Eigen::Quaterniond value(q[0], q[1], q[2], q[3]);
    if (!value.coeffs().allFinite() || value.norm() < 1e-9)
        throw std::runtime_error("invalid Talos gimbal quaternion");
    return value.normalized();
}

std::pair<double, double> yawPitch(const Eigen::Quaterniond& value) {
    const Eigen::Vector3d forward = value.toRotationMatrix().col(0);
    return {
        std::atan2(forward.y(), forward.x()),
        std::atan2(forward.z(), std::hypot(forward.x(), forward.y())),
    };
}

sp_core::SolverConfig solverConfig(const talos_bridge::FrameBundle& bundle) {
    const auto& info = bundle.metadata.camera_info;
    if (info.width != talos_bridge::kImageWidth ||
        info.height != talos_bridge::kImageHeight ||
        !(info.fx > 0.0) || !(info.fy > 0.0))
        throw std::runtime_error("Talos camera_info is not initialized");
    sp_core::SolverConfig config;
    config.camera_matrix << info.fx, 0.0, info.cx,
                            0.0, info.fy, info.cy,
                            0.0, 0.0, 1.0;
    config.distortion.assign(info.distortion.begin(), info.distortion.end());
    // OpenCV camera (right, down, forward) -> Talos gimbal (forward, left, up).
    config.r_camera_to_gimbal << 0.0, 0.0, 1.0,
                                -1.0, 0.0, 0.0,
                                 0.0, -1.0, 0.0;
    const auto& camera = bundle.metadata.camera.position;
    config.t_camera_to_gimbal = Eigen::Vector3d(camera[0], camera[1], camera[2]);
    if (!config.t_camera_to_gimbal.allFinite() ||
        config.t_camera_to_gimbal.norm() > 1.0)
        throw std::runtime_error(
            "Talos camera extrinsic is not a plausible gimbal-relative offset");
    config.r_gimbal_to_imu_body = Eigen::Matrix3d::Identity();
    config.max_reprojection_error_px = 5.0;
    return config;
}

double milliseconds(Clock::duration value) {
    return std::chrono::duration<double, std::milli>(value).count();
}

void appendJsonNumber(std::ostringstream& out, double value) {
    if (std::isfinite(value)) out << value;
    else out << "null";
}

std::string appendAngleDiagnostics(
    std::string json, const std::vector<sp_core::Armor>& armors,
    double feedback_yaw, double feedback_pitch,
    const sp_core::AimResult& command_aim, bool command_sent,
    const talos_bridge::TargetLock& target_lock,
    std::optional<int> current_target_label) {
    if (json.empty() || json.back() != '}') return json;

    const sp_core::Armor* measurement = nullptr;
    for (const auto& armor : armors) {
        if (armor.pnp_valid &&
            (!measurement || armor.confidence > measurement->confidence))
            measurement = &armor;
    }

    json.pop_back();
    std::ostringstream out;
    out << json << std::boolalpha << std::setprecision(10)
        << ",\"angle_contract\":{"
        << "\"command_frame\":\"ros_world_absolute\","
        << "\"feedback_frame\":\"ros_world_absolute\","
        << "\"yaw_domain_deg\":\"[-180,180)\","
        << "\"pitch_domain_deg\":\"[-90,90]\","
        << "\"command_sent\":" << command_sent
        << ",\"feedback_yaw_deg\":";
    appendJsonNumber(out, feedback_yaw * kRadiansToDegrees);
    out << ",\"feedback_pitch_deg\":";
    appendJsonNumber(out, feedback_pitch * kRadiansToDegrees);
    out << ",\"command_yaw_deg\":";
    appendJsonNumber(out, command_aim.valid
        ? command_aim.yaw * kRadiansToDegrees
        : std::numeric_limits<double>::quiet_NaN());
    out << ",\"command_pitch_deg\":";
    appendJsonNumber(out, command_aim.valid
        ? command_aim.pitch * kRadiansToDegrees
        : std::numeric_limits<double>::quiet_NaN());
    out << ",\"measurement\":";

    if (!measurement) {
        out << "null";
    } else {
        const auto camera_yaw =
            std::atan2(-measurement->xyz_camera.x(), measurement->xyz_camera.z());
        const auto camera_pitch = std::atan2(
            -measurement->xyz_camera.y(),
            std::hypot(measurement->xyz_camera.z(), measurement->xyz_camera.x()));
        const auto gimbal_yaw =
            std::atan2(measurement->xyz_gimbal.y(), measurement->xyz_gimbal.x());
        const auto gimbal_pitch = std::atan2(
            measurement->xyz_gimbal.z(),
            std::hypot(measurement->xyz_gimbal.x(), measurement->xyz_gimbal.y()));
        const auto world_yaw =
            std::atan2(measurement->xyz_world.y(), measurement->xyz_world.x());
        const auto world_pitch = std::atan2(
            measurement->xyz_world.z(),
            std::hypot(measurement->xyz_world.x(), measurement->xyz_world.y()));
        out << "{\"camera_relative_yaw_deg\":";
        appendJsonNumber(out, camera_yaw * kRadiansToDegrees);
        out << ",\"camera_relative_pitch_deg\":";
        appendJsonNumber(out, camera_pitch * kRadiansToDegrees);
        out << ",\"gimbal_relative_yaw_deg\":";
        appendJsonNumber(out, gimbal_yaw * kRadiansToDegrees);
        out << ",\"gimbal_relative_pitch_deg\":";
        appendJsonNumber(out, gimbal_pitch * kRadiansToDegrees);
        out << ",\"world_absolute_yaw_deg\":";
        appendJsonNumber(out, world_yaw * kRadiansToDegrees);
        out << ",\"world_absolute_pitch_deg\":";
        appendJsonNumber(out, world_pitch * kRadiansToDegrees);
        out << ",\"distance_m\":";
        appendJsonNumber(out, measurement->xyz_camera.norm());
        out << ",\"xyz_camera_m\":[";
        appendJsonNumber(out, measurement->xyz_camera.x());
        out << ',';
        appendJsonNumber(out, measurement->xyz_camera.y());
        out << ',';
        appendJsonNumber(out, measurement->xyz_camera.z());
        out << "],\"xyz_gimbal_m\":[";
        appendJsonNumber(out, measurement->xyz_gimbal.x());
        out << ',';
        appendJsonNumber(out, measurement->xyz_gimbal.y());
        out << ',';
        appendJsonNumber(out, measurement->xyz_gimbal.z());
        out << "],\"xyz_world_m\":[";
        appendJsonNumber(out, measurement->xyz_world.x());
        out << ',';
        appendJsonNumber(out, measurement->xyz_world.y());
        out << ',';
        appendJsonNumber(out, measurement->xyz_world.z());
        out << ']';
        out << '}';
    }
    out << "},\"target_lock\":{\"locked_label\":";
    if (target_lock.lockedLabel()) out << *target_lock.lockedLabel();
    else out << "null";
    out << ",\"current_label\":";
    if (current_target_label) out << *current_target_label;
    else out << "null";
    out << ",\"visible_labels\":[";
    for (std::size_t i = 0; i < target_lock.visibleLabels().size(); ++i) {
        if (i) out << ',';
        out << target_lock.visibleLabels()[i];
    }
    out << "],\"mode\":\""
        << (target_lock.lockedLabel() ? "locked" : "automatic")
        << "\",\"last_action\":\"" << target_lock.lastAction() << "\"}}";
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        applyScheduling(options);
        auto config = hnu25::standard::loadConfig(options.config_path);
        if (options.allow_fire) {
            if (!config.fire_gate.enabled)
                std::cerr
                    << "[Safety] --allow-fire explicitly enables FireGate "
                       "despite YAML fire_gate.enabled=false\n";
            config.fire_gate.enabled = true;
            if (options.fire_max_age_ms > 0)
                config.fire_gate.max_measurement_age =
                    static_cast<double>(options.fire_max_age_ms) / 1000.0;
        }
        if (!options.model_path.empty())
            config.detector.model_path = options.model_path.string();
        if (options.web) {
            config.web.enabled = true;
            config.web.publish_hz = options.web_hz;
        }
        if (!options.telemetry) config.telemetry.enabled = false;
        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);

        std::cout << "=== deploy_dev C++ Talos bridge ===\n"
                  << "config=" << options.config_path.string() << '\n'
                  << "model=" << config.detector.model_path << '\n'
                  << "shm_dir=" << options.shm_dir.string() << '\n'
                  << "capture=async release=after_rgb_copy queue=latest(1)\n"
                  << "send=" << std::boolalpha << options.send
                  << " allow_fire=" << options.allow_fire
                  << " single_shot=" << options.single_shot
                  << " fire_max_age_ms="
                  << config.fire_gate.max_measurement_age * 1000.0
                  << " capture_only=" << options.capture_only
                  << " web=" << config.web.enabled
                  << " telemetry_udp=" << config.telemetry.enabled << '\n';
        if (options.send && !config.command_enabled)
            std::cerr << "[Safety] --send explicitly overrides YAML command_enabled=false\n";
        std::unique_ptr<hnu25::Detector> detector;
        std::unique_ptr<sp_core::Tracker> tracker;
        std::unique_ptr<sp_core::Aimer> aimer;
        std::unique_ptr<planner::MpcPlanner> mpc;
        std::unique_ptr<aim::FireGate> fire_gate;
        if (!options.capture_only) {
            detector = hnu25::makeDetector(config.detector);
            tracker = std::make_unique<sp_core::Tracker>(options.config_path.string());
            aimer = std::make_unique<sp_core::Aimer>(config.aimer);
            mpc = std::make_unique<planner::MpcPlanner>(config.planner);
            fire_gate = std::make_unique<aim::FireGate>(config.fire_gate);
        }
        hnu25::standard::RuntimeParameters runtime_parameters{
            config.detector.conf_threshold, config.binary_threshold,
            config.bullet_speed, config.aimer};
        std::unique_ptr<hnu25::standard::RuntimeTuning> tuning;
        std::unique_ptr<talos_bridge::WebDebugSink> web;
        if (config.web.enabled) {
            tuning = std::make_unique<hnu25::standard::RuntimeTuning>(
                options.config_path, config.web.shm_name, runtime_parameters);
            web = std::make_unique<talos_bridge::WebDebugSink>(
                config.web, options.web_auxiliary_views);
            if (!web->ready())
                throw std::runtime_error(
                    "Windows web debug shared memory could not be initialized");
        }
        telemetry::UdpPublisher telemetry(config.telemetry);
        talos_bridge::TalosAsyncSource source(options.shm_dir);
        talos_bridge::TargetLock target_lock(options.shm_dir / "target_lock.request");
        source.start();

        std::unique_ptr<sp_core::Solver> solver;
        Eigen::Vector3d solver_camera_translation =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        std::uint64_t processed = 0;
        std::uint64_t sequence_gaps = 0;
        std::uint64_t last_sequence = 0;
        bool have_last_sequence = false;
        bool warned_f5 = false;
        bool warned_implausible_command = false;
        bool warned_transient_camera_extrinsic = false;
        std::optional<int> previous_target_label;
        talos_bridge::SingleShotFireLatch single_shot_latch;
        auto first_capture = Clock::time_point{};
        auto previous_capture = Clock::time_point{};
        auto report_at = Clock::now() + std::chrono::seconds(1);
        std::uint64_t report_processed = 0;
        std::uint64_t report_captured = 0;
        double age_sum_ms = 0.0;
        double total_sum_ms = 0.0;
        double detection_sum_ms = 0.0;

        while (!stop_requested &&
               (options.max_frames == 0 || processed < options.max_frames)) {
            if (tuning) {
                if (const auto request = tuning->poll()) {
                    try {
                        detector->setConfidenceThreshold(
                            request->parameters.conf_threshold);
                        aimer = std::make_unique<sp_core::Aimer>(
                            request->parameters.aimer);
                        runtime_parameters = request->parameters;
                        if (request->save) tuning->save(runtime_parameters);
                        tuning->acknowledge(
                            *request, runtime_parameters, true,
                            request->save ? "applied and saved"
                                          : "temporarily applied");
                    } catch (const std::exception& error) {
                        tuning->acknowledge(
                            *request, runtime_parameters, false, error.what());
                    }
                }
            }
            std::shared_ptr<const talos_bridge::FrameBundle> bundle;
            if (!source.waitLatest(bundle, std::chrono::milliseconds(2000))) {
                if (stop_requested) break;
                if (source.heartbeatAgeSeconds() > 2.0)
                    throw std::runtime_error("Talos heartbeat is stale; simulator is not publishing");
                continue;
            }
            const auto dequeue_at = Clock::now();
            const auto capture_at = bundle->metadata.captured_at;
            if (first_capture.time_since_epoch().count() == 0) first_capture = capture_at;
            const double dt_s = previous_capture.time_since_epoch().count() == 0
                ? config.planner.dt_s
                : std::chrono::duration<double>(capture_at - previous_capture).count();
            previous_capture = capture_at;
            if (have_last_sequence && bundle->metadata.frame_seq > last_sequence + 1)
                sequence_gaps += bundle->metadata.frame_seq - last_sequence - 1;
            last_sequence = bundle->metadata.frame_seq;
            have_last_sequence = true;
            ++processed;

            if (options.processing_delay_ms > 0)
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(options.processing_delay_ms));

            const double dequeue_age_ms = milliseconds(dequeue_at - capture_at);
            age_sum_ms += dequeue_age_ms;
            if (options.capture_only) {
                total_sum_ms += milliseconds(Clock::now() - capture_at);
            } else {
                const auto& camera = bundle->metadata.camera.position;
                const Eigen::Vector3d camera_translation(
                    camera[0], camera[1], camera[2]);
                if (!camera_translation.allFinite() ||
                    camera_translation.norm() > 1.0) {
                    if (!warned_transient_camera_extrinsic) {
                        std::cerr
                            << "[Startup] waiting for mounted camera extrinsic; "
                            << "received gimbal-relative offset="
                            << camera_translation.transpose() << '\n';
                        warned_transient_camera_extrinsic = true;
                    }
                    continue;
                }
                warned_transient_camera_extrinsic = false;
                if (!solver ||
                    (camera_translation - solver_camera_translation).norm() >
                        1e-3) {
                    solver =
                        std::make_unique<sp_core::Solver>(solverConfig(*bundle));
                    solver_camera_translation = camera_translation;
                }
                const Eigen::Quaterniond gimbal_q = quaternion(bundle->metadata.gimbal);
                solver->setImuQuaternion(gimbal_q);

                const auto before_detection = Clock::now();
                const auto detected = detector->detect(bundle->image_bgr);
                const auto after_detection = Clock::now();
                std::vector<const hnu25::DetectedArmor*> detections_by_confidence;
                detections_by_confidence.reserve(detected.size());
                for (const auto& detection : detected)
                    if (static_cast<int>(detection.label) >= 0 &&
                        static_cast<int>(detection.label) <= 7)
                        detections_by_confidence.push_back(&detection);
                std::sort(
                    detections_by_confidence.begin(), detections_by_confidence.end(),
                    [](const auto* a, const auto* b) {
                        return a->confidence > b->confidence;
                    });
                std::vector<int> visible_labels_by_priority;
                visible_labels_by_priority.reserve(detections_by_confidence.size());
                for (const auto* detection : detections_by_confidence)
                    visible_labels_by_priority.push_back(
                        static_cast<int>(detection->label));
                if (const auto action = target_lock.pollRequest()) {
                    if (target_lock.apply(
                            *action, visible_labels_by_priority,
                            previous_target_label)) {
                        tracker->reset();
                        fire_gate->reset();
                    }
                } else {
                    target_lock.observeVisible(visible_labels_by_priority);
                }
                std::vector<sp_core::Armor> armors;
                armors.reserve(detected.size());
                int pnp_valid = 0;
                int pnp_rejected = 0;
                double confidence_max = 0.0;
                double reprojection_sum = 0.0;
                double reprojection_max = 0.0;
                for (const auto& detection : detected) {
                    confidence_max = std::max(confidence_max,
                                              static_cast<double>(detection.confidence));
                    if (!target_lock.allows(static_cast<int>(detection.label)))
                        continue;
                    auto armor = sp_core::adapt(detection);
                    if (solver->solve(armor)) {
                        ++pnp_valid;
                        reprojection_sum += armor.reprojection_error;
                        reprojection_max = std::max(reprojection_max,
                                                    armor.reprojection_error);
                    } else {
                        ++pnp_rejected;
                    }
                    armors.push_back(std::move(armor));
                }
                const auto after_pnp = Clock::now();
                const auto target = tracker->update(armors, capture_at);
                const std::optional<int> current_target_label = target
                    ? std::optional<int>(static_cast<int>(target->name()))
                    : std::nullopt;
                previous_target_label = current_target_label;
                const auto after_tracker = Clock::now();
                sp_core::AimResult geometric_aim;
                if (target) {
                    geometric_aim =
                        aimer->aim(*target, runtime_parameters.bullet_speed);
                    if (geometric_aim.valid)
                        geometric_aim.yaw =
                            talos_bridge::normalizeYaw(geometric_aim.yaw);
                }
                const auto after_aim = Clock::now();

                const auto [feedback_yaw, feedback_pitch] = yawPitch(gimbal_q);
                planner::PlannedCommand planned;
                planned.reference_yaw = geometric_aim.yaw;
                planned.reference_pitch = geometric_aim.pitch;
                planned.yaw = geometric_aim.yaw;
                planned.pitch = geometric_aim.pitch;
                planned.solver_status = config.planner.enabled
                    ? planner::SolverStatus::InvalidInput
                    : planner::SolverStatus::Disabled;
                if (config.planner.enabled) {
                    planner::FeedbackState feedback;
                    feedback.yaw_rad = feedback_yaw;
                    feedback.pitch_rad = feedback_pitch;
                    planned = mpc->plan(geometric_aim, feedback,
                                        dt_s > 0.0 ? dt_s : config.planner.dt_s);
                }
                auto command_aim = planner::selectAim(geometric_aim, planned);
                if (command_aim.valid)
                    command_aim.yaw =
                        talos_bridge::normalizeYaw(command_aim.yaw);
                aim::FireGateInput gate_input;
                gate_input.aim = command_aim;
                gate_input.track_state = tracker->state();
                gate_input.target_valid = target.has_value();
                gate_input.pose_valid = true;
                gate_input.feedback_valid = true;
                gate_input.measurement_age =
                    std::chrono::duration<double>(after_aim - capture_at).count();
                gate_input.reprojection_error = reprojection_max;
                gate_input.nis = tracker->lastNis();
                gate_input.feedback_yaw = feedback_yaw;
                gate_input.feedback_pitch = feedback_pitch;
                if (target) gate_input.armor_type = target->type();
                const auto fire_decision = fire_gate->evaluate(gate_input);
                const bool fire_requested =
                    options.allow_fire && fire_decision.allowed;
                const bool tracking_lock =
                    target.has_value() &&
                    tracker->state() == sp_core::TrackState::TRACKING;
                single_shot_latch.observeTrackingLock(tracking_lock);

                bool command_sent = false;
                bool fire = false;
                if (options.send && source.autoAimEnabled() && command_aim.valid) {
                    if (!talos_bridge::isPlausibleCameraAcquisition(
                            command_aim.yaw, command_aim.pitch,
                            feedback_yaw, feedback_pitch)) {
                        if (!warned_implausible_command) {
                            std::cerr
                                << "[Safety] rejected out-of-view aim command: "
                                << "aim_yaw="
                                << command_aim.yaw * kRadiansToDegrees
                                << " aim_pitch="
                                << command_aim.pitch * kRadiansToDegrees
                                << " feedback_yaw="
                                << feedback_yaw * kRadiansToDegrees
                                << " feedback_pitch="
                                << feedback_pitch * kRadiansToDegrees << '\n';
                            warned_implausible_command = true;
                        }
                    } else {
                        warned_implausible_command = false;
                        fire = options.single_shot
                            ? single_shot_latch.permit(fire_requested)
                            : fire_requested;
                        const double distance = command_aim.point.norm();
                        source.writeGimbalCommand(
                            command_aim.yaw * kRadiansToDegrees,
                            command_aim.pitch * kRadiansToDegrees,
                            distance, fire);
                        command_sent = true;
                    }
                } else if (options.send && !source.autoAimEnabled() && !warned_f5) {
                    std::cerr << "[Safety] simulator auto-aim is OFF; press F5 to accept commands\n";
                    warned_f5 = true;
                }
                if (options.send && options.probe_hold && source.autoAimEnabled() &&
                    !command_sent) {
                    source.writeGimbalCommand(feedback_yaw * kRadiansToDegrees,
                                              feedback_pitch * kRadiansToDegrees,
                                              1.0, false);
                }

                const auto completed = Clock::now();
                const double detection_ms = milliseconds(after_detection - before_detection);
                const double pnp_ms = milliseconds(after_pnp - after_detection);
                const double tracker_ms = milliseconds(after_tracker - after_pnp);
                const double aim_ms = milliseconds(after_aim - after_tracker);
                const double total_ms = milliseconds(completed - capture_at);
                detection_sum_ms += detection_ms;
                total_sum_ms += total_ms;

                telemetry::FrameSample sample;
                sample.frame_index = bundle->metadata.frame_seq;
                sample.video_time_s = std::chrono::duration<double>(capture_at - first_capture).count();
                sample.timeline_s = sample.video_time_s;
                sample.dt_s = dt_s;
                sample.pose_valid = true;
                sample.detection_count = static_cast<int>(detected.size());
                sample.detection_confidence_max = confidence_max;
                sample.pnp_valid_count = pnp_valid;
                sample.pnp_rejected_count = pnp_rejected;
                sample.reprojection_mean_px = pnp_valid ? reprojection_sum / pnp_valid : 0.0;
                sample.reprojection_max_px = reprojection_max;
                sample.tracker_state_code = static_cast<int>(tracker->state());
                sample.tracker_state_name = tracker->stateName();
                sample.target_valid = target.has_value();
                if (target) {
                    const auto& state = target->state();
                    for (int i = 0; i < state.rows(); ++i) sample.state[i] = state[i];
                }
                sample.nis = tracker->lastNis();
                sample.nis_failure_ratio = tracker->nisFailureRatio();
                sample.aim_valid = command_aim.valid;
                sample.fire = fire;
                if (!options.allow_fire) {
                    sample.fire_reason = "cli_disabled";
                } else if (options.single_shot && fire_decision.allowed &&
                           single_shot_latch.latched() && !fire) {
                    sample.fire_reason = "single_shot_latched";
                } else {
                    sample.fire_reason = aim::toString(fire_decision.reason);
                }
                sample.fire_yaw_error_rad = fire_decision.yaw_error;
                sample.fire_pitch_error_rad = fire_decision.pitch_error;
                sample.fire_yaw_tolerance_rad = fire_decision.yaw_tolerance;
                sample.fire_pitch_tolerance_rad = fire_decision.pitch_tolerance;
                sample.aim_yaw_rad = command_aim.yaw;
                sample.aim_pitch_rad = command_aim.pitch;
                sample.fly_time_s = geometric_aim.fly_time;
                sample.planner_valid = planned.valid;
                sample.planner_solver_status = static_cast<int>(planned.solver_status);
                sample.planner_reference_yaw_rad = planned.reference_yaw;
                sample.planner_reference_pitch_rad = planned.reference_pitch;
                sample.planner_yaw_rad = planned.yaw;
                sample.planner_pitch_rad = planned.pitch;
                sample.planner_yaw_velocity_rad_s = planned.yaw_vel;
                sample.planner_pitch_velocity_rad_s = planned.pitch_vel;
                sample.planner_yaw_acceleration_rad_s2 = planned.yaw_acc;
                sample.planner_pitch_acceleration_rad_s2 = planned.pitch_acc;
                sample.gimbal_feedback_valid = true;
                sample.gimbal_yaw_deg = feedback_yaw * kRadiansToDegrees;
                sample.gimbal_pitch_deg = feedback_pitch * kRadiansToDegrees;
                sample.detection_ms = detection_ms;
                sample.pnp_ms = pnp_ms;
                sample.tracker_ms = tracker_ms;
                sample.aim_ms = aim_ms;
                sample.total_ms = total_ms;
                telemetry.publish(sample);
                if (web) {
                    web->submit(
                        bundle, detected,
                        appendAngleDiagnostics(
                            telemetry::encodeJson(sample), armors,
                            feedback_yaw, feedback_pitch,
                            command_aim, command_sent, target_lock,
                            current_target_label),
                        runtime_parameters.binary_threshold);
                }
            }

            const auto now = Clock::now();
            if (now >= report_at) {
                const auto stats = source.stats();
                const auto interval_processed = processed - report_processed;
                const auto interval_captured = stats.captured - report_captured;
                std::cout << std::fixed << std::setprecision(2)
                          << "capture_fps=" << interval_captured
                          << " process_fps=" << interval_processed
                          << " processed=" << processed
                          << " overwritten=" << stats.overwritten
                          << " producer_seq_skips=" << sequence_gaps
                          << " copy_release_ms=" << stats.mean_copy_release_ms
                          << " convert_ms=" << stats.mean_convert_ms
                          << " dequeue_age_ms=" << age_sum_ms / processed
                          << " detect_ms=" << (options.capture_only ? 0.0 : detection_sum_ms / processed)
                          << " total_ms=" << total_sum_ms / processed
                          << " heartbeat_age_ms=" << source.heartbeatAgeSeconds() * 1000.0
                          << '\n';
                report_processed = processed;
                report_captured = stats.captured;
                report_at = now + std::chrono::seconds(1);
            }
        }

        source.stop();
        const auto stats = source.stats();
        std::cout << "stopped processed=" << processed
                  << " captured=" << stats.captured
                  << " overwritten=" << stats.overwritten
                  << " producer_seq_skips=" << sequence_gaps
                  << " incoherent=" << stats.incoherent
                  << " release_conflicts=" << stats.release_conflicts << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
