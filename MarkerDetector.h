#pragma once
#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include <vector>

/*
 MarkerDetector - robust HSV-based marker segmentation using existing
 vision primitives (no OpenCV). Public API kept for compatibility.
*/

// HSV range with saturation/value thresholds included for compatibility
struct HSVRange {
    double h_lo;   // degrees [0..360)
    double h_hi;   // degrees [0..360)
    double s_min;  // saturation floor (0..1)
    int v_min;     // value floor (0..255)
};

// MarkerDetector configuration + API
class MarkerDetector {
public:
    MarkerDetector();

    // Backwards-compatible single-range members (used by older callers)
    HSVRange blue_range;   // primary blue range
    HSVRange red_range;    // primary red range

    // Newer API: allow multiple hue ranges per color (e.g. red wraps)
    std::vector<HSVRange> blue_ranges;
    std::vector<HSVRange> red_ranges;

    // Morphology and filtering params (public for tuning)
    int morph_iters_open = 1;
    int morph_iters_close = 2;
    int morph_repeat = 1;

    int min_blob_area = 60;
    int max_blob_area = 3000;
    double min_area_ratio = 0.20;

    // Debug flag
    bool debug_dump_masks = false;

    // Main detection function (keeps original signature)
    void detect_markers(
        image& rgb,
        std::vector<Blob>& front_blobs,   // BLUE
        std::vector<Blob>& rear_blobs     // RED
    );

private:
    // internal helpers
    void rgb_to_hsv(ibyte R, ibyte G, ibyte B, double& h, double& s, double& v);
    bool hue_in_ranges(double h, const std::vector<HSVRange>& ranges);
    void build_mask(image& rgb, image& mask_out, const std::vector<HSVRange>& hue_ranges, double s_min, int v_min);
    void clean_mask(image& mask, int open_iters, int close_iters, int repeat);
    void extract_blobs_filtered(image& mask, image& grey_for_centroid, std::vector<Blob>& out_blobs);
};
