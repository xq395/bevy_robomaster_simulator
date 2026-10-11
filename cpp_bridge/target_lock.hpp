#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace talos_bridge {

class TargetLock {
public:
    explicit TargetLock(std::filesystem::path request_path)
        : request_path_(std::move(request_path)) {}

    std::optional<std::string> pollRequest() {
        std::ifstream input(request_path_);
        if (!input) return std::nullopt;
        std::string action;
        std::getline(input, action);
        input.close();
        std::error_code error;
        std::filesystem::remove(request_path_, error);
        return action.empty() ? std::nullopt : std::optional<std::string>(action);
    }

    bool apply(const std::string& action, const std::vector<int>& visible_by_priority,
               std::optional<int> current_target) {
        visible_labels_ = visible_by_priority;
        std::sort(visible_labels_.begin(), visible_labels_.end());
        visible_labels_.erase(
            std::unique(visible_labels_.begin(), visible_labels_.end()),
            visible_labels_.end());

        const auto old = locked_label_;
        if (action == "unlock") {
            locked_label_.reset();
            last_action_ = "unlocked";
        } else if (action == "lock_current") {
            if (current_target && validLabel(*current_target)) {
                locked_label_ = *current_target;
                last_action_ = "locked current target";
            } else {
                const auto candidate = std::find_if(
                    visible_by_priority.begin(), visible_by_priority.end(),
                    [](int label) { return validLabel(label); });
                if (candidate != visible_by_priority.end()) {
                    locked_label_ = *candidate;
                    last_action_ = "locked highest-confidence visible target";
                } else {
                    last_action_ = "no visible target to lock";
                }
            }
        } else if (action == "next") {
            if (visible_labels_.empty()) {
                last_action_ = "no visible target to switch to";
            } else {
                const std::optional<int> base = locked_label_ ? locked_label_ : current_target;
                const auto next = base
                    ? std::upper_bound(visible_labels_.begin(), visible_labels_.end(), *base)
                    : visible_labels_.begin();
                locked_label_ =
                    next == visible_labels_.end() ? visible_labels_.front() : *next;
                last_action_ = "switched target";
            }
        } else {
            last_action_ = "unknown action";
        }
        return old != locked_label_;
    }

    void observeVisible(const std::vector<int>& labels) {
        visible_labels_ = labels;
        std::sort(visible_labels_.begin(), visible_labels_.end());
        visible_labels_.erase(
            std::remove_if(visible_labels_.begin(), visible_labels_.end(),
                           [](int label) { return !validLabel(label); }),
            visible_labels_.end());
        visible_labels_.erase(
            std::unique(visible_labels_.begin(), visible_labels_.end()),
            visible_labels_.end());
    }

    bool allows(int label) const noexcept {
        return !locked_label_ || label == *locked_label_;
    }

    const std::optional<int>& lockedLabel() const noexcept { return locked_label_; }
    const std::vector<int>& visibleLabels() const noexcept { return visible_labels_; }
    const std::string& lastAction() const noexcept { return last_action_; }

private:
    static bool validLabel(int label) noexcept { return label >= 0 && label <= 7; }

    std::filesystem::path request_path_;
    std::optional<int> locked_label_;
    std::vector<int> visible_labels_;
    std::string last_action_ = "automatic";
};

}  // namespace talos_bridge
