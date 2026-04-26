#pragma once

#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include "MarkerDetector.h" // for HSVRange

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ColorProfiles {

// -----------------------------------------------------------------------------
// ColorSpec
// -----------------------------------------------------------------------------
// Full HSV description of one marker color.
//
// - hue_ranges: one or more hue intervals (useful for wrapped hues such as red)
// - s_min / v_min: minimum saturation and value thresholds
// -----------------------------------------------------------------------------
struct ColorSpec {
    std::string name;
    std::vector<HSVRange> hue_ranges;
    double s_min;
    int v_min;
};

// -----------------------------------------------------------------------------
// RobotColorProfile
// -----------------------------------------------------------------------------
// Explicit robot visual profile defined by an ordered front/rear marker pair.
// Example:
//   GR = green front, red rear
//   OB = orange front, blue rear
// -----------------------------------------------------------------------------
struct RobotColorProfile {
    std::string name;
    std::string front_color;
    std::string rear_color;
};

// Richer detection type than the legacy RobotDet.
struct ProfiledRobotDet {
    std::string profile_name;
    std::string front_color;
    std::string rear_color;
    Blob front;
    Blob rear;
    double x;
    double y;
    double theta;
    double sep_px;
    double pair_score;
};

// Richer track type than the legacy RobotTrack.
struct ProfiledRobotTrack {
    int id;
    std::string profile_name;
    std::string front_color;
    std::string rear_color;
    double x;
    double y;
    double theta;
    double sep_px;
    double last_seen;
    int misses;
    int stable_hits;
};

// Build the default marker color models used by the Python reference pipeline.
std::map<std::string, ColorSpec> buildDefaultColorSpecs();

// Build the two required robot visual profiles.
std::map<std::string, RobotColorProfile> buildDefaultRobotProfiles();

// Optional convenience override so thresholds can be tuned from elsewhere.
void setColorSpec(std::map<std::string, ColorSpec>& specs,
                  const std::string& name,
                  const std::vector<HSVRange>& hue_ranges,
                  double s_min,
                  int v_min);

// Segment all configured marker colors in one pass.
// Returned masks are GREY_IMAGE images and must be freed with freeMaskMap(...).
std::map<std::string, image> segmentAllMarkerColors(
    image& rgb,
    const std::map<std::string, ColorSpec>& color_specs,
    int morph_open_iters = 1,
    int morph_close_iters = 2,
    int morph_repeat = 1);

// Free all images created by segmentAllMarkerColors(...).
void freeMaskMap(std::map<std::string, image>& masks);

// Extract connected-component blobs for each color mask.
std::map<std::string, std::vector<Blob>> extractBlobsByColor(
    image& rgb,
    const std::map<std::string, image>& masks,
    int min_blob_area,
    int max_blob_area,
    double min_area_ratio);

// Estimate marker separation (px) for a specific front/rear marker set.
std::optional<double> estimateMarkerSepPxForProfile(
    const std::vector<Blob>& front_blobs,
    const std::vector<Blob>& rear_blobs);

// Estimate separation for each named robot profile.
std::map<std::string, std::optional<double>> estimateProfileSeparations(
    const std::map<std::string, RobotColorProfile>& robot_profiles,
    const std::map<std::string, std::vector<Blob>>& blobs_by_color);

// Pair front/rear blobs into robot detections using color + geometry.
std::vector<ProfiledRobotDet> pairProfileMarkers(
    const std::vector<Blob>& front_blobs,
    const std::vector<Blob>& rear_blobs,
    const RobotColorProfile& profile,
    std::optional<double> expected_sep_px,
    double sep_tol,
    double max_pair_px,
    double area_ratio_tol);

// Full multi-profile detection pipeline.
std::vector<ProfiledRobotDet> detectProfileRobots(
    image& rgb,
    const std::map<std::string, ColorSpec>& color_specs,
    const std::map<std::string, RobotColorProfile>& robot_profiles,
    const std::map<std::string, std::optional<double>>& expected_sep_px_map,
    int blob_min_area,
    int blob_max_area,
    double min_area_ratio,
    double sep_tol,
    double max_pair_px,
    double area_ratio_tol,
    int morph_open_iters,
    int morph_close_iters,
    int morph_repeat,
    std::map<std::string, image>* out_masks = nullptr,
    std::map<std::string, std::vector<Blob>>* out_blobs_by_color = nullptr,
    std::map<std::string, std::optional<double>>* out_sep_estimates = nullptr);

// Track profile-aware detections over time.
std::vector<ProfiledRobotTrack> updateProfileTracks(
    const std::vector<ProfiledRobotTrack>& prev,
    const std::vector<ProfiledRobotDet>& dets,
    double now,
    double max_match_dist_px,
    int max_misses);

} // namespace ColorProfiles

