#pragma once
#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include <vector>

struct HSVRange {
    double h_lo, h_hi;   // degrees
    double s_min;        // 0–1
    double v_min;        // 0–255
};

class MarkerDetector {
public:
    MarkerDetector();

    // Adjustable thresholds
    HSVRange blue_range;
    HSVRange red_range;

    // Main detection function
    void detect_markers(
        image& rgb,
        std::vector<Blob>& front_blobs,   // BLUE
        std::vector<Blob>& rear_blobs     // RED
    );

private:
    void rgb_to_hsv(ibyte R, ibyte G, ibyte B, double& h, double& s, double& v);
    bool in_range(double h, double s, double v, const HSVRange& range);

    void extract_blobs(
        image& binary,
        image& grey_for_centroid,
        std::vector<Blob>& out_blobs
    );
};
