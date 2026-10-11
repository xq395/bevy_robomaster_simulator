#include "pipeline/runtime_tuning.hpp"

#include <yaml-cpp/yaml.h>

#include <windows.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace hnu25::standard {
namespace {

std::string safeName(std::string name) {
    if (!name.empty() && name.front() == '/') name.erase(name.begin());
    for (char& value : name) if (value == '/') value = '_';
    return name.empty() ? "hnu_vision_debug" : name;
}

void validate(const RuntimeParameters& value) {
    const auto finite = [](double v) { return std::isfinite(v); };
    if (!finite(value.conf_threshold) || value.conf_threshold < 0.01F || value.conf_threshold > 0.95F)
        throw std::invalid_argument("conf_threshold must be in [0.01,0.95]");
    if (value.binary_threshold < 0 || value.binary_threshold > 255)
        throw std::invalid_argument("binary_threshold must be in [0,255]");
    if (!finite(value.bullet_speed) || value.bullet_speed < 14.0 || value.bullet_speed > 35.0)
        throw std::invalid_argument("bullet_speed must be in [14,35]");
    if (!finite(value.aimer.yaw_offset) || std::abs(value.aimer.yaw_offset) > 10.0 ||
        !finite(value.aimer.pitch_offset) || std::abs(value.aimer.pitch_offset) > 10.0)
        throw std::invalid_argument("aim offsets must be in [-10,10] deg");
    if (!finite(value.aimer.coming_angle) || value.aimer.coming_angle < 0.0 || value.aimer.coming_angle > 90.0 ||
        !finite(value.aimer.leaving_angle) || value.aimer.leaving_angle < 0.0 || value.aimer.leaving_angle > 90.0)
        throw std::invalid_argument("selection angles must be in [0,90] deg");
    if (!finite(value.aimer.decision_speed) || value.aimer.decision_speed < 0.0 || value.aimer.decision_speed > 20.0)
        throw std::invalid_argument("decision_speed must be in [0,20]");
    if (!finite(value.aimer.high_speed_delay) || value.aimer.high_speed_delay < 0.0 || value.aimer.high_speed_delay > 0.5 ||
        !finite(value.aimer.low_speed_delay) || value.aimer.low_speed_delay < 0.0 || value.aimer.low_speed_delay > 0.5)
        throw std::invalid_argument("delay times must be in [0,0.5] s");
}

void atomicWrite(const std::filesystem::path& path, const std::string& content) {
    const auto temporary = path.wstring() + L".tmp";
    {
        std::ofstream output(std::filesystem::path(temporary), std::ios::trunc);
        output << content;
        output.flush();
        if (!output) throw std::runtime_error("cannot write tuning state");
    }
    if (!MoveFileExW(temporary.c_str(), path.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        throw std::system_error(
            static_cast<int>(GetLastError()), std::system_category(),
            "cannot replace tuning state");
    }
}

}  // namespace

RuntimeTuning::RuntimeTuning(std::filesystem::path config_path,
                             std::string shm_name,
                             RuntimeParameters initial)
    : config_path_(std::move(config_path)), initial_(initial) {
    validate(initial_);
    const std::string safe = safeName(std::move(shm_name));
    const auto temp = std::filesystem::temp_directory_path();
    request_path_ = temp / (safe + ".tuning_request.yaml");
    state_path_ = temp / (safe + ".tuning_state.json");
    std::filesystem::remove(request_path_);
    writeState(initial_, 0, true, true, "loaded from YAML");
}

std::optional<TuningRequest> RuntimeTuning::poll() {
    if (!std::filesystem::exists(request_path_)) return std::nullopt;
    try {
        const YAML::Node yaml = YAML::LoadFile(request_path_.string());
        TuningRequest request;
        request.id = yaml["id"].as<std::uint64_t>();
        if (request.id <= last_id_) return std::nullopt;
        request.save = yaml["save"].as<bool>(false);
        request.parameters.conf_threshold = yaml["conf_threshold"].as<float>();
        request.parameters.binary_threshold = yaml["binary_threshold"].as<int>();
        request.parameters.bullet_speed = yaml["bullet_speed"].as<double>();
        request.parameters.aimer.yaw_offset = yaml["yaw_offset"].as<double>();
        request.parameters.aimer.pitch_offset = yaml["pitch_offset"].as<double>();
        request.parameters.aimer.coming_angle = yaml["comming_angle"].as<double>();
        request.parameters.aimer.leaving_angle = yaml["leaving_angle"].as<double>();
        request.parameters.aimer.decision_speed = yaml["decision_speed"].as<double>();
        request.parameters.aimer.high_speed_delay = yaml["high_speed_delay_time"].as<double>();
        request.parameters.aimer.low_speed_delay = yaml["low_speed_delay_time"].as<double>();
        validate(request.parameters);
        last_id_ = request.id;
        return request;
    } catch (const std::exception& error) {
        writeState(initial_, last_id_, false, false, error.what());
        std::filesystem::remove(request_path_);
        return std::nullopt;
    }
}

void RuntimeTuning::acknowledge(const TuningRequest& request,
                                const RuntimeParameters& active,
                                bool ok,
                                const std::string& message) {
    initial_ = active;
    writeState(active, request.id, request.save && ok, ok, message);
    std::filesystem::remove(request_path_);
}

void RuntimeTuning::save(const RuntimeParameters& value) const {
    validate(value);
    YAML::Node yaml = YAML::LoadFile(config_path_.string());
    yaml["detector"]["conf_threshold"] = value.conf_threshold;
    yaml["web"]["binary_threshold"] = value.binary_threshold;
    yaml["bullet_speed"] = value.bullet_speed;
    yaml["yaw_offset"] = value.aimer.yaw_offset;
    yaml["pitch_offset"] = value.aimer.pitch_offset;
    yaml["comming_angle"] = value.aimer.coming_angle;
    yaml["leaving_angle"] = value.aimer.leaving_angle;
    yaml["decision_speed"] = value.aimer.decision_speed;
    yaml["high_speed_delay_time"] = value.aimer.high_speed_delay;
    yaml["low_speed_delay_time"] = value.aimer.low_speed_delay;
    YAML::Emitter output;
    output << yaml;
    if (!output.good()) throw std::runtime_error("cannot serialize YAML");
    atomicWrite(config_path_, output.c_str());
}

void RuntimeTuning::writeState(const RuntimeParameters& value,
                               std::uint64_t id,
                               bool saved,
                               bool ok,
                               const std::string& message) const {
    std::ostringstream out;
    out << std::boolalpha << "{\"id\":" << id << ",\"ok\":" << ok
        << ",\"saved\":" << saved << ",\"message\":\"";
    for (char c : message) out << (c == '\"' ? '\'' : c);
    out << "\",\"parameters\":{\"conf_threshold\":" << value.conf_threshold
        << ",\"binary_threshold\":" << value.binary_threshold
        << ",\"bullet_speed\":" << value.bullet_speed
        << ",\"yaw_offset\":" << value.aimer.yaw_offset
        << ",\"pitch_offset\":" << value.aimer.pitch_offset
        << ",\"comming_angle\":" << value.aimer.coming_angle
        << ",\"leaving_angle\":" << value.aimer.leaving_angle
        << ",\"decision_speed\":" << value.aimer.decision_speed
        << ",\"high_speed_delay_time\":" << value.aimer.high_speed_delay
        << ",\"low_speed_delay_time\":" << value.aimer.low_speed_delay << "}}";
    atomicWrite(state_path_, out.str());
}

}  // namespace hnu25::standard
