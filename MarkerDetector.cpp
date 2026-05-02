#include "MarkerDetector.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>

// ============================================================================
// Constructor: Initialize with default Blue-Red profile
// ============================================================================
MarkerDetector::MarkerDetector()
{
    // Default blue ranges (for backward compatibility)
    default_blue_ranges_ = { {200.0, 250.0, 0.60, 40} };
    
    // Default red ranges with wrap-around
    default_red_ranges_ = { 
        {0.0, 10.0, 0.35, 60}, 
        {335.0, 360.0, 0.35, 60} 
    };
    
    // Set default BR (Blue-Red) profile
    MarkerProfile default_profile;
    default_profile.name = "BR";
    default_profile.front_ranges = default_blue_ranges_;
    default_profile.rear_ranges = default_red_ranges_;
    profiles_.push_back(default_profile);
    
    // Default settings
    morph_iters_open = 1;
    morph_iters_close = 2;
    morph_repeat = 1;
    min_blob_area = 60;
    max_blob_area = 3000;
    min_area_ratio = 0.20;
    debug_dump_masks = false;
}

// ============================================================================
// Profile Management
// ============================================================================
void MarkerDetector::setProfiles(const std::vector<MarkerProfile>& profiles)
{
    profiles_ = profiles;
}

void MarkerDetector::addProfile(const MarkerProfile& profile)
{
    profiles_.push_back(profile);
}

// ============================================================================
// RGB to HSV Conversion
// ============================================================================
void MarkerDetector::rgb_to_hsv(ibyte R, ibyte G, ibyte B, double& h, double& s, double& v)
{
    int maxv = R, minv = R;
    if (G > maxv) maxv = G;
    if (B > maxv) maxv = B;
    if (G < minv) minv = G;
    if (B < minv) minv = B;

    int delta = maxv - minv;
    v = maxv;

    if (delta == 0) {
        s = 0.0;
        h = 0.0;
        return;
    }

    s = (maxv == 0) ? 0.0 : (delta / static_cast<double>(maxv));

    double H;
    if (maxv == R)      H = static_cast<double>(G - B) / delta;
    else if (maxv == G) H = static_cast<double>(B - R) / delta + 2.0;
    else                H = static_cast<double>(R - G) / delta + 4.0;

    h = 60.0 * H;
    if (h < 0.0) h += 360.0;
}

// ============================================================================
// Check if hue falls within any HSV range (handles wrap-around)
// ============================================================================
bool MarkerDetector::hue_in_ranges(double h, const std::vector<HSVRange>& ranges)
{
    for (const auto& r : ranges) {
        // Check saturation and value thresholds first
        // Note: We can't check s and v here since we only have h
        // The caller (build_mask) will check s and v
        
        if (r.h_lo <= r.h_hi) {
            // Normal range (e.g., 35-95 for green)
            if (h >= r.h_lo && h <= r.h_hi) return true;
        } else {
            // Wrap-around range (e.g., 335-10 for red)
            if (h >= r.h_lo || h <= r.h_hi) return true;
        }
    }
    return false;
}

// ============================================================================
// Build binary mask for specified HSV ranges
// ============================================================================
void MarkerDetector::build_mask(image& rgb, image& mask_out, const std::vector<HSVRange>& hue_ranges)
{
    const int W = rgb.width;
    const int H = rgb.height;
    std::memset(mask_out.pdata, 0, static_cast<size_t>(W) * H);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            const int idx = 3 * (j * W + i);
            ibyte B = rgb.pdata[idx + 0];
            ibyte G = rgb.pdata[idx + 1];
            ibyte R = rgb.pdata[idx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            // Check each HSV range
            bool match = false;
            for (const auto& range : hue_ranges) {
                bool h_match = false;
                if (range.h_lo <= range.h_hi) {
                    h_match = (h >= range.h_lo && h <= range.h_hi);
                } else {
                    h_match = (h >= range.h_lo || h <= range.h_hi);
                }
                
                if (h_match && s >= range.s_min && v >= range.v_min) {
                    match = true;
                    break;
                }
            }

            mask_out.pdata[j * W + i] = match ? 255 : 0;
        }
    }
}

// ============================================================================
// Morphological cleaning: open then close operations
// ============================================================================
void MarkerDetector::clean_mask(image& mask, int open_iters, int close_iters, int repeat)
{
    if (repeat <= 0) repeat = 1;

    image tmp;
    tmp.type = GREY_IMAGE;
    tmp.width = mask.width;
    tmp.height = mask.height;
    allocate_image(tmp);

    for (int rep = 0; rep < repeat; ++rep) {
        // Opening: erode then dilate (removes small noise)
        for (int it = 0; it < open_iters; ++it) {
            erode(mask, tmp);
            copy(tmp, mask);
        }
        for (int it = 0; it < open_iters; ++it) {
            dialate(mask, tmp);
            copy(tmp, mask);
        }
        
        // Closing: dilate then erode (fills small holes)
        for (int it = 0; it < close_iters; ++it) {
            dialate(mask, tmp);
            copy(tmp, mask);
        }
        for (int it = 0; it < close_iters; ++it) {
            erode(mask, tmp);
            copy(tmp, mask);
        }
    }

    free_image(tmp);
}

// ============================================================================
// Extract filtered blobs from binary mask
// ============================================================================
void MarkerDetector::extract_blobs_filtered(image& mask, image& grey_for_centroid, std::vector<Blob>& out_blobs)
{
    out_blobs.clear();

    image label;
    label.type = LABEL_IMAGE;
    label.width = mask.width;
    label.height = mask.height;
    allocate_image(label);

    int nlabels = 0;
    label_image(mask, label, nlabels);

    i2byte* pl = reinterpret_cast<i2byte*>(label.pdata);
    const int W = label.width;
    const int H = label.height;

    for (int L = 1; L <= nlabels; ++L) {
        int area = 0;
        int minx = W, miny = H, maxx = 0, maxy = 0;

        // Compute bounding box and area
        for (int j = 0; j < H; ++j) {
            for (int i = 0; i < W; ++i) {
                if (pl[j * W + i] == L) {
                    ++area;
                    if (i < minx) minx = i;
                    if (i > maxx) maxx = i;
                    if (j < miny) miny = j;
                    if (j > maxy) maxy = j;
                }
            }
        }

        // Filter by area
        if (area < min_blob_area) continue;
        if (max_blob_area > 0 && area > max_blob_area) continue;

        // Filter by area ratio (compactness)
        const int bw = std::max(1, maxx - minx + 1);
        const int bh = std::max(1, maxy - miny + 1);
        const double area_ratio = static_cast<double>(area) / static_cast<double>(bw * bh);
        if (area_ratio < min_area_ratio) continue;

        // Compute centroid
        double ic, jc;
        centroid(grey_for_centroid, label, L, ic, jc);

        Blob b;
        b.x = ic;
        b.y = jc;
        b.area = area;
        out_blobs.push_back(b);
    }

    free_image(label);
    
    // Sort by area (largest first)
    std::sort(out_blobs.begin(), out_blobs.end(), [](const Blob& a, const Blob& b) {
        return a.area > b.area;
    });
}

// ============================================================================
// ORIGINAL API: Backward-compatible single-profile detection
// ============================================================================
void MarkerDetector::detect_markers(
    image& rgb,
    std::vector<Blob>& front_blobs,
    std::vector<Blob>& rear_blobs)
{
    front_blobs.clear();
    rear_blobs.clear();
    
    if (profiles_.empty()) {
        std::cerr << "MarkerDetector: No profiles configured!" << std::endl;
        return;
    }
    
    const auto& profile = profiles_[0];  // Use first profile for backward compatibility
    
    // Allocate grey image for centroid calculation
    image grey;
    grey.type = GREY_IMAGE;
    grey.width = rgb.width;
    grey.height = rgb.height;
    allocate_image(grey);
    copy(rgb, grey);
    
    // Detect front markers
    image front_mask;
    front_mask.type = GREY_IMAGE;
    front_mask.width = rgb.width;
    front_mask.height = rgb.height;
    allocate_image(front_mask);
    
    build_mask(rgb, front_mask, profile.front_ranges);
    clean_mask(front_mask, morph_iters_open, morph_iters_close, morph_repeat);
    extract_blobs_filtered(front_mask, grey, front_blobs);
    
    if (debug_dump_masks) {
        std::cout << "Front (" << profile.name << "): " << front_blobs.size() << " blobs\n";
    }
    
    // Detect rear markers
    image rear_mask;
    rear_mask.type = GREY_IMAGE;
    rear_mask.width = rgb.width;
    rear_mask.height = rgb.height;
    allocate_image(rear_mask);
    
    build_mask(rgb, rear_mask, profile.rear_ranges);
    clean_mask(rear_mask, morph_iters_open, morph_iters_close, morph_repeat);
    extract_blobs_filtered(rear_mask, grey, rear_blobs);
    
    if (debug_dump_masks) {
        std::cout << "Rear (" << profile.name << "): " << rear_blobs.size() << " blobs\n";
    }
    
    // Cleanup
    free_image(grey);
    free_image(front_mask);
    free_image(rear_mask);
}

// ============================================================================
// NEW API: Multi-profile detection
// ============================================================================
std::map<std::string, std::pair<std::vector<Blob>, std::vector<Blob>>> 
MarkerDetector::detect_all_profiles(image& rgb)
{
    std::map<std::string, std::pair<std::vector<Blob>, std::vector<Blob>>> results;
    
    if (profiles_.empty()) {
        std::cerr << "MarkerDetector: No profiles configured!" << std::endl;
        return results;
    }
    
    // Allocate grey image once for all profiles
    image grey;
    grey.type = GREY_IMAGE;
    grey.width = rgb.width;
    grey.height = rgb.height;
    allocate_image(grey);
    copy(rgb, grey);
    
    // Process each profile
    for (const auto& profile : profiles_) {
        std::vector<Blob> front_blobs, rear_blobs;
        
        // Front markers
        image front_mask;
        front_mask.type = GREY_IMAGE;
        front_mask.width = rgb.width;
        front_mask.height = rgb.height;
        allocate_image(front_mask);
        
        build_mask(rgb, front_mask, profile.front_ranges);
        clean_mask(front_mask, morph_iters_open, morph_iters_close, morph_repeat);
        extract_blobs_filtered(front_mask, grey, front_blobs);
        free_image(front_mask);
        
        // Rear markers
        image rear_mask;
        rear_mask.type = GREY_IMAGE;
        rear_mask.width = rgb.width;
        rear_mask.height = rgb.height;
        allocate_image(rear_mask);
        
        build_mask(rgb, rear_mask, profile.rear_ranges);
        clean_mask(rear_mask, morph_iters_open, morph_iters_close, morph_repeat);
        extract_blobs_filtered(rear_mask, grey, rear_blobs);
        free_image(rear_mask);
        
        if (debug_dump_masks) {
            std::cout << "Profile " << profile.name << ": " 
                      << front_blobs.size() << " front, " 
                      << rear_blobs.size() << " rear\n";
        }
        
        results[profile.name] = {front_blobs, rear_blobs};
    }
    
    free_image(grey);
    return results;
}