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

// Median nearest-neighbor separation estimator (moved from MarkerPairing)
std::optional<double> estimate_marker_sep_px(const std::vector<Blob>& front_blobs,
                                             const std::vector<Blob>& rear_blobs);
