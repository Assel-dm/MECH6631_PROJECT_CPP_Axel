#include "LaserGate.h"

LaserGate::LaserGate(int stable_frames, int cooldown_frames)
    : stable_frames_(stable_frames),
      cooldown_frames_(cooldown_frames),
      current_target_id_(std::nullopt),
      good_frames_(0),
      cooldown_left_(0)
{
}

void LaserGate::reset()
{
    current_target_id_ = std::nullopt;
    good_frames_ = 0;
    cooldown_left_ = 0;
}

bool LaserGate::update(std::optional<int> target_id, bool request_fire)
{
    if (cooldown_left_ > 0) {
        --cooldown_left_;
    }

    if (target_id != current_target_id_) {
        current_target_id_ = target_id;
        good_frames_ = 0;
    }

    if (request_fire && target_id.has_value()) {
        ++good_frames_;
    } else {
        good_frames_ = 0;
    }

    if (cooldown_left_ == 0 && good_frames_ >= stable_frames_) {
        cooldown_left_ = cooldown_frames_;
        good_frames_ = 0;
        return true;
    }

    return false;
}
