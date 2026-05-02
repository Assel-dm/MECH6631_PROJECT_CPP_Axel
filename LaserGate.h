#pragma once

#include <optional>

class LaserGate {
public:
    LaserGate(int stable_frames = 3, int cooldown_frames = 8);

    // Reset all internal state.
    void reset();

    // Update the gate.
    // target_id:
    //   id of the currently tracked enemy we want to fire at
    // request_fire:
    //   result of the geometric/logical fire request for this frame
    //
    // Returns true only when:
    // - the same target has been valid for enough consecutive frames
    // - the cooldown has expired
    bool update(std::optional<int> target_id, bool request_fire);

private:
    int stable_frames_;
    int cooldown_frames_;

    std::optional<int> current_target_id_;
    int good_frames_;
    int cooldown_left_;
};
