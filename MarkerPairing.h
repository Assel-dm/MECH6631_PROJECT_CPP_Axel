#pragma once
#include <vector>
#include <optional>
#include "Types.h"

// Compute robot pose (midpoint, heading, separation) from front/rear marker blobs
RobotDet compute_pose(const Blob& front, const Blob& rear);

// Estimate marker separation (px) as the median of nearest-front->rear distances.
// Returns std::nullopt if not enough data.
std::optional<double> estimate_marker_sep_px(const std::vector<Blob>& front_blobs,
                                             const std::vector<Blob>& rear_blobs);

// Pair blue(front) and red(rear) blobs into RobotDet list.
// - expected_sep_px: if provided uses distance gating around expected separation
// - sep_tol: fractional tolerance (e.g., 0.55 -> ±55%)
// - max_pair_px: hard maximum pairing distance (fallback pairing)
std::vector<RobotDet> pair_markers(const std::vector<Blob>& front_blobs,
                                   const std::vector<Blob>& rear_blobs,
                                   std::optional<double> expected_sep_px,
                                   double sep_tol,
                                   double max_pair_px);
