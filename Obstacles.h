#pragma once
#include "Types.h"
#include "image_transfer.h"
#include "vision.h"
#include <vector>

// forward-declare to avoid including MarkerDetector.h in this header
struct Blob;      // forward-declare
struct HSVRange;  // forward-declare (defined in MarkerDetector.h)

// ensure this typedef is available before the declaration:
using ibyte = unsigned char;   // or include the header that defines ibyte/ebyte

class Obstacles {
public:
    Obstacles();

    // Main function: detect obstacles from RGB image (existing single-arg)
    std::vector<Obstacle> detect(image& rgb);

    // Overload: allow caller to pass marker HSV ranges & min area for filtering
    std::vector<Obstacle> detect(image& rgb, const HSVRange& blue_range, const HSVRange& red_range, int min_area = 50);

    // New overload: detect obstacles considering robot blobs
    std::vector<Obstacle> detect(image& rgb,
                                 const std::vector<Blob>& robot_blobs,
                                 const HSVRange& blue_range,
                                 const HSVRange& red_range,
                                 int min_area = 50);

    // New: floor-model detector (Lab z-score) with optional robot exclusion mask
    // - robot_mask may be nullptr
    std::vector<Obstacle> detect_floor_model(image& rgb,
                                             image* robot_mask,
                                             float kL, float ka, float kb,
                                             int min_area = 50);

    // New overload: floor-model taking robot blobs (builds mask internally)
    std::vector<Obstacle> detect_floor_model(image& rgb,
                                             const std::vector<Blob>& robot_blobs,
                                             float kL, float ka, float kb,
                                             int min_area = 50);

private:
    // HSV conversion (professor’s code)
    void rgb_to_hsv(ibyte r, ibyte g, ibyte b,
                    double &h, double &s, double &v);

    // Build histogram of saturation (0–1 mapped to 0–255)
    void build_s_histogram(image& rgb, std::vector<int>& hist);

    // Automatic threshold selection from histogram
    int find_s_threshold(const std::vector<int>& hist);

    // Extract obstacles from binary mask
    std::vector<Obstacle> extract_obstacles(image& binary, image& grey);

    // Utility: allocate GREY image
    void alloc_grey(image& img, int w, int h);
};