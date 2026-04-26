#include "MarkerDetector.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <algorithm>

/*
 MarkerDetector.cpp - robust HSV segmentation and blob extraction.
 Keeps the original public API and restores compatibility with
 code that expects single-range fields (s_min, v_min).
*/

// Constructor: set reasonable defaults
MarkerDetector::MarkerDetector()
{
    // primary single-range defaults (for backward compatibility)
    blue_range.h_lo = 200.0;  blue_range.h_hi = 250.0;
    blue_range.s_min = 0.60; blue_range.v_min = 40;

    // red as single primary range (also provide wrap-around via blue/red_ranges)
    red_range.h_lo = 0.0;    red_range.h_hi = 15.0;
    red_range.s_min = 0.35;  red_range.v_min = 60;

    // populate vector ranges (useful for wrap-around red)
    blue_ranges = { blue_range };
    red_ranges = { {0.0, 10.0, red_range.s_min, red_range.v_min}, {335.0, 360.0, red_range.s_min, red_range.v_min} };

    // morphology defaults (already initialized in header, repeat here for clarity)
    morph_iters_open = 1;
    morph_iters_close = 2;
    morph_repeat = 1;

    // blob filters
    min_blob_area = 60;
    max_blob_area = 3000;
    min_area_ratio = 0.20;

    debug_dump_masks = true;
}

// convert RGB (bytes) to HSV (h deg, s 0..1, v 0..255)
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

    s = delta / (double)maxv;

    double H;
    if (maxv == R)      H = (double)(G - B) / delta;
    else if (maxv == G) H = (double)(B - R) / delta + 2;
    else                H = (double)(R - G) / delta + 4;

    h = 60.0 * H;
    if (h < 0.0) h += 360.0;
}

// check if hue (deg) falls inside any of the provided ranges (wrap-safe)
bool MarkerDetector::hue_in_ranges(double h, const std::vector<HSVRange>& ranges)
{
    for (const auto &r : ranges) {
        if (r.h_lo <= r.h_hi) {
            if (h >= r.h_lo && h <= r.h_hi) return true;
        } else {
            if (h >= r.h_lo || h <= r.h_hi) return true;
        }
    }
    return false;
}

// Build binary mask: pixels that match ANY hue_range AND s>=s_min AND v>=v_min
void MarkerDetector::build_mask(image& rgb, image& mask_out, const std::vector<HSVRange>& hue_ranges, double s_min, int v_min)
{
    int W = rgb.width;
    int H = rgb.height;
    std::memset(mask_out.pdata, 0, (size_t)W * H);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            int ridx = 3 * (j * W + i);
            ibyte B = rgb.pdata[ridx + 0];
            ibyte G = rgb.pdata[ridx + 1];
            ibyte R = rgb.pdata[ridx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            if (s >= s_min && v >= v_min && hue_in_ranges(h, hue_ranges)) {
                mask_out.pdata[j * W + i] = 255;
            } else {
                mask_out.pdata[j * W + i] = 0;
            }
        }
    }
}

// Morphological cleaning using existing erode/dialate primitives.
// Emulate larger kernels by repeating operations.
void MarkerDetector::clean_mask(image& mask, int open_iters, int close_iters, int repeat)
{
    if (repeat <= 0) repeat = 1;
    image tmp;
    tmp.type = GREY_IMAGE;
    tmp.width = mask.width;
    tmp.height = mask.height;
    allocate_image(tmp);

    for (int rep = 0; rep < repeat; ++rep) {
        // Opening: erode (open_iters times) -> dilate (open_iters times)
        for (int it = 0; it < open_iters; ++it) {
            erode(mask, tmp);
            copy(tmp, mask);
        }
        for (int it = 0; it < open_iters; ++it) {
            dialate(mask, tmp);
            copy(tmp, mask);
        }

        // Closing: dilate (close_iters times) -> erode (close_iters times)
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

// Extract connected components and apply simple shape filters.
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

    i2byte* pl = (i2byte*)label.pdata;
    int W = label.width;
    int H = label.height;

    for (int L = 1; L <= nlabels; ++L) {
        int area = 0;
        int minx = W, miny = H, maxx = 0, maxy = 0;
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

        if (area < min_blob_area) continue;
        if (max_blob_area > 0 && area > max_blob_area) continue;

        int bw = std::max(1, maxx - minx + 1);
        int bh = std::max(1, maxy - miny + 1);
        double area_ratio = (double)area / (double)(bw * bh);
        if (area_ratio < min_area_ratio) continue;

        double ic, jc;
        centroid(grey_for_centroid, label, L, ic, jc);

        Blob b;
        b.x = ic;
        b.y = jc;
        b.area = area;
        out_blobs.push_back(b);
    }

    free_image(label);
}

// Public API: detect_markers (keeps compatibility with existing callers)
void MarkerDetector::detect_markers(
    image& rgb,
    std::vector<Blob>& front_blobs,
    std::vector<Blob>& rear_blobs)
{
    front_blobs.clear();
    rear_blobs.clear();

    int W = rgb.width;
    int H = rgb.height;

    image grey, bin_blue, bin_red;
    grey.type = GREY_IMAGE;
    bin_blue.type = GREY_IMAGE;
    bin_red.type = GREY_IMAGE;
    grey.width = bin_blue.width = bin_red.width = W;
    grey.height = bin_blue.height = bin_red.height = H;

    allocate_image(grey);
    allocate_image(bin_blue);
    allocate_image(bin_red);

    // GREY used for centroid computations
    copy(rgb, grey);

    // Build masks:
    // Prefer vector ranges if provided; otherwise use single-range legacy members.
    const std::vector<HSVRange>* blue_src = &blue_ranges;
    const std::vector<HSVRange>* red_src  = &red_ranges;

    if (blue_src->empty()) {
        // fallback to single-range
        blue_ranges = { blue_range };
        blue_src = &blue_ranges;
    }
    if (red_src->empty()) {
        red_ranges = { red_range, red_range }; // ensure not empty; second entry may be adjusted by user
        red_src = &red_ranges;
    }

    // Use per-range s_min/v_min from the first range (legacy behavior)
    double blue_smin = (*blue_src)[0].s_min;
    int    blue_vmin = (int)(*blue_src)[0].v_min;
    double red_smin  = (*red_src)[0].s_min;
    int    red_vmin  = (int)(*red_src)[0].v_min;

    build_mask(rgb, bin_blue, *blue_src, blue_smin, blue_vmin);
    build_mask(rgb, bin_red,  *red_src,  red_smin,  red_vmin);

    // Morphology
    clean_mask(bin_blue, morph_iters_open, morph_iters_close, morph_repeat);
    clean_mask(bin_red, morph_iters_open, morph_iters_close, morph_repeat);

    // Extract blobs with filtering
    extract_blobs_filtered(bin_blue, grey, front_blobs);
    extract_blobs_filtered(bin_red, grey, rear_blobs);

    // Optional debug dump
    if (debug_dump_masks) {
        image dbg;
        dbg.type = RGB_IMAGE;
        dbg.width = W;
        dbg.height = H;
        allocate_image(dbg);
        // blue mask -> cyan-ish
        for (int j = 0; j < H; ++j) {
            for (int i = 0; i < W; ++i) {
                ibyte v = bin_blue.pdata[j * W + i];
                int idx = 3 * (j * W + i);
                dbg.pdata[idx] = v;
                dbg.pdata[idx+1] = v;
                dbg.pdata[idx+2] = 0;
            }
        }
        save_rgb_image("debug_bin_blue.bmp", dbg);
        // red mask -> red channel
        for (int j = 0; j < H; ++j) {
            for (int i = 0; i < W; ++i) {
                ibyte v = bin_red.pdata[j * W + i];
                int idx = 3 * (j * W + i);
                dbg.pdata[idx] = 0;
                dbg.pdata[idx+1] = 0;
                dbg.pdata[idx+2] = v;
            }
        }
        save_rgb_image("debug_bin_red.bmp", dbg);
        free_image(dbg);
    }

    free_image(grey);
    free_image(bin_blue);
    free_image(bin_red);
}