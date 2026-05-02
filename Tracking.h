#pragma once
#include "Types.h"
#include <vector>
#include <optional>

class Tracker {
public:
    std::vector<RobotDet> pairMarkers(
        const std::vector<Blob>& front,
        const std::vector<Blob>& rear,
        std::optional<double> expected_sep,
        double sep_tol,
        double max_pair_px);

    std::vector<RobotTrack> updateTracks(
        const std::vector<RobotTrack>& prev,
        const std::vector<RobotDet>& dets,
        double now,
        double max_match_dist,
        int max_misses);
};

std::optional<double> estimate_marker_sep_px(const std::vector<Blob>& front_blobs,
                                             const std::vector<Blob>& rear_blobs);

// Find a tracked robot by ID — returns false if not found
inline bool getTrackedRobotState(const std::vector<RobotTrack>& tracks, int id,
                                 double& x, double& y, double& theta) {
    for (const auto& t : tracks) {
        if (t.id == id) { x = t.x; y = t.y; theta = t.theta; return true; }
    }
    return false;
}
