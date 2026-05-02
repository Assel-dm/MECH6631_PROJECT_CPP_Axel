#pragma once
#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include <vector>
#include <map>
#include <string>

// HSV range with saturation/value thresholds
struct HSVRange {
    double h_lo;   // Hue low (degrees 0-360)
    double h_hi;   // Hue high (degrees 0-360)
    double s_min;  // Minimum saturation (0.0-1.0)
    int v_min;     // Minimum value (0-255)
};

// Marker color profile: defines front and rear marker colors
struct MarkerProfile {
    std::string name;                       // Profile name (e.g., "BR", "GR", "OB")
    std::vector<HSVRange> front_ranges;     // Front marker HSV ranges
    std::vector<HSVRange> rear_ranges;      // Rear marker HSV ranges
};

class MarkerDetector {
public:
    MarkerDetector();

    // ============ Profile Management ============
    void setProfiles(const std::vector<MarkerProfile>& profiles);
    void addProfile(const MarkerProfile& profile);
    
    // ============ Detection APIs ============
    
    // Original API (backward compatible): detects first profile only
    void detect_markers(
        image& rgb,
        std::vector<Blob>& front_blobs,
        std::vector<Blob>& rear_blobs
    );
    
    // New API: detect all configured profiles
    std::map<std::string, std::pair<std::vector<Blob>, std::vector<Blob>>> 
    detect_all_profiles(image& rgb);

    // ============ Public Parameters (tunable) ============
    int morph_iters_open;
    int morph_iters_close;
    int morph_repeat;
    int min_blob_area;
    int max_blob_area;
    double min_area_ratio;
    bool debug_dump_masks;

private:
    std::vector<MarkerProfile> profiles_;
    std::vector<HSVRange> default_blue_ranges_;
    std::vector<HSVRange> default_red_ranges_;
    
    // Helper functions
    void rgb_to_hsv(ibyte R, ibyte G, ibyte B, double& h, double& s, double& v);
    bool hue_in_ranges(double h, const std::vector<HSVRange>& ranges);
    void build_mask(image& rgb, image& mask_out, const std::vector<HSVRange>& hue_ranges);
    void clean_mask(image& mask, int open_iters, int close_iters, int repeat);
    void extract_blobs_filtered(image& mask, image& grey_for_centroid, std::vector<Blob>& out_blobs);
};
