#include "Obstacles.h"
#include "MarkerDetector.h" // for HSVRange and Blob
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>

Obstacles::Obstacles() {}

void Obstacles::alloc_grey(image& img, int w, int h) {
    img.type = GREY_IMAGE;
    img.width = w;
    img.height = h;
    allocate_image(img);
}

void Obstacles::rgb_to_hsv(ibyte R, ibyte G, ibyte B,
    double& h, double& s, double& v)
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

    h = 60 * H;
    if (h < 0) h += 360;
}

void Obstacles::build_s_histogram(image& rgb, std::vector<int>& hist)
{
    int W = rgb.width;
    int H = rgb.height;
    hist.assign(256, 0);

    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {
            int idx = 3 * (j * W + i);
            ibyte B = rgb.pdata[idx];
            ibyte G = rgb.pdata[idx + 1];
            ibyte R = rgb.pdata[idx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            int bin = (int)(s * 255.0);
            if (bin < 0) bin = 0;
            if (bin > 255) bin = 255;

            hist[bin]++;
        }
    }
}

int Obstacles::find_s_threshold(const std::vector<int>& hist)
{
    // Find two largest peaks
    int peak1 = 0, peak2 = 0;
    for (int i = 0; i < 256; i++) {
        if (hist[i] > hist[peak1]) peak1 = i;
    }
    for (int i = 0; i < 256; i++) {
        if (i == peak1) continue;
        if (hist[i] > hist[peak2]) peak2 = i;
    }

    int lo = std::min(peak1, peak2);
    int hi = std::max(peak1, peak2);

    // Find valley between peaks
    int valley = lo;
    int min_val = hist[lo];
    for (int i = lo; i <= hi; i++) {
        if (hist[i] < min_val) {
            min_val = hist[i];
            valley = i;
        }
    }

    return valley;
}

std::vector<Obstacle> Obstacles::extract_obstacles(image& binary, image& grey)
{
    std::vector<Obstacle> obs;

    image label;
    label.type = LABEL_IMAGE;
    label.width = binary.width;
    label.height = binary.height;
    allocate_image(label);

    int nlabels = 0;
    label_image(binary, label, nlabels);

    i2byte* pl = (i2byte*)label.pdata;
    int W = label.width;
    int H = label.height;

    for (int L = 1; L <= nlabels; L++) {
        int minx = W, miny = H, maxx = 0, maxy = 0;
        int area = 0;

        for (int j = 0; j < H; j++) {
            for (int i = 0; i < W; i++) {
                if (pl[j * W + i] == L) {
                    area++;
                    if (i < minx) minx = i;
                    if (i > maxx) maxx = i;
                    if (j < miny) miny = j;
                    if (j > maxy) maxy = j;
                }
            }
        }

        int min_area = std::max(50, (W*H) / 50000); // tune divisor
        if (area < min_area) continue; // ignore tiny noise

        double ic, jc;
        centroid(grey, label, L, ic, jc);

        Obstacle o;
        o.x = minx;
        o.y = miny;
        o.w = maxx - minx + 1;
        o.h = maxy - miny + 1;
        o.cx = ic;
        o.cy = jc;
        o.area = area;

        obs.push_back(o);
    }

    free_image(label);
    return obs;
}

// helper to test HSVRange (wrap-around aware)
static bool in_range_local(double h, double s, double v, const HSVRange& r) {
    bool h_ok;
    if (r.h_lo <= r.h_hi) h_ok = (h >= r.h_lo && h <= r.h_hi);
    else h_ok = (h >= r.h_lo || h <= r.h_hi); // wrap around 360->0
    return h_ok && (s >= r.s_min) && (v >= r.v_min);
}

// New overload that uses provided HSV ranges and min_area
std::vector<Obstacle> Obstacles::detect(image& rgb, const HSVRange& blue_range, const HSVRange& red_range, int min_area_param)
{
    int W = rgb.width;
    int H = rgb.height;

    // 1. Convert RGB → GREY for centroid calculations
    image grey;
    alloc_grey(grey, W, H);
    copy(rgb, grey);

    // 2. Build saturation histogram
    std::vector<int> hist;
    build_s_histogram(rgb, hist);

    // 3. Automatic threshold
    int s_thresh = find_s_threshold(hist);

    // 4. Build binary mask from S > threshold
    image binary;
    alloc_grey(binary, W, H);
    memset(binary.pdata, 0, W * H);

    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {
            int idx = 3 * (j * W + i);
            ibyte B = rgb.pdata[idx];
            ibyte G = rgb.pdata[idx + 1];
            ibyte R = rgb.pdata[idx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            int s_bin = (int)(s * 255.0);
            int min_s_thresh = 30; // tune
            int thresh = std::max(s_thresh, min_s_thresh);

            // exclude marker colors using provided ranges
            bool is_marker = false;
            if (in_range_local(h, s, v, blue_range)) is_marker = true;
            if (in_range_local(h, s, v, red_range)) is_marker = true;

            // use normalized threshold and lower the floor so paler colors pass
            double thresh_s = std::max(s_thresh, 8) / 255.0; // try 8 or 10
            if (!is_marker && s > thresh_s) {
                binary.pdata[j * W + i] = 255;
            } else {
                binary.pdata[j * W + i] = 0;
            }
        }
    }

    // 5. Morphological cleanup: do opening (erode -> dilate) to remove specks, then a closing
    image tmp;
    alloc_grey(tmp, W, H);

    // opening
    erode(binary, tmp);
    erode(tmp, tmp);
    copy(tmp, binary);

    dialate(binary, tmp);
    dialate(tmp, binary);

    // closing (bridge small gaps)
    dialate(binary, tmp);
    copy(tmp, binary);
    erode(binary, tmp);
    copy(tmp, binary);

    // 6. Extract obstacles
    auto obs = extract_obstacles(binary, grey);

    // filter small components by provided min_area_param (and keep original heuristic)
    int computed_min = std::max(min_area_param, (W * H) / 50000);
    std::vector<Obstacle> filtered;
    for (auto &o : obs) {
        if (o.area >= computed_min) filtered.push_back(o);
    }

    free_image(tmp);
    free_image(binary);
    free_image(grey);

    return filtered;
}

// Add at top if not already included:
#include "MarkerDetector.h" // for HSVRange and Blob

// helper: draw filled circle into mask
static void draw_filled_circle_mask(image& mask, int cx, int cy, int r) {
    int W = mask.width, H = mask.height;
    for (int y = std::max(0, cy - r); y <= std::min(H-1, cy + r); ++y) {
        int dy = y - cy;
        int dxmax = (int)std::floor(std::sqrt((double)r*r - dy*dy));
        int x0 = std::max(0, cx - dxmax);
        int x1 = std::min(W-1, cx + dxmax);
        for (int x = x0; x <= x1; ++x)
            mask.pdata[y * W + x] = 255;
    }
}

std::vector<Obstacle> Obstacles::detect(image& rgb,
                                        const std::vector<Blob>& robot_blobs,
                                        const HSVRange& blue_range,
                                        const HSVRange& red_range,
                                        int min_area_param)
{
    int W = rgb.width;
    int H = rgb.height;

    // create robot mask
    image robot_mask;
    alloc_grey(robot_mask, W, H);
    std::memset(robot_mask.pdata, 0, W * H);

    // mark robot pixels: estimate radius from blob area (fallback to fixed radius)
    for (const auto &b : robot_blobs) {
        int r = 30; // fallback radius in px
        if (b.area > 0) {
            double est_r = std::sqrt((double)b.area / M_PI);
            r = (int)std::max(8.0, est_r * 1.2); // scale up slightly
        }
        draw_filled_circle_mask(robot_mask, (int)std::round(b.x), (int)std::round(b.y), r);
    }

    // copy of existing detect overload but treat robot_mask pixels as markers
    // (copy code from your detect(rgb, blue_range, red_range, min_area_param) implementation)
    // I'll show the key change: inside the pixel loop, set is_marker = true if robot_mask.pdata[...] != 0

    image grey;
    alloc_grey(grey, W, H);
    copy(rgb, grey);

    std::vector<int> hist;
    build_s_histogram(rgb, hist);
    int s_thresh = find_s_threshold(hist);

    image binary;
    alloc_grey(binary, W, H);
    memset(binary.pdata, 0, W * H);

    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {
            int idx = 3 * (j * W + i);
            ibyte B = rgb.pdata[idx];
            ibyte G = rgb.pdata[idx + 1];
            ibyte R = rgb.pdata[idx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            int s_bin = (int)(s * 255.0);
            int min_s_thresh = 30; // tune
            int thresh = std::max(s_thresh, min_s_thresh);

            bool is_marker = false;
            // robot mask override: if robot_mask pixel set, treat as marker
            if (robot_mask.pdata[j * W + i] != 0) is_marker = true;

            // also respect provided HSV ranges
            if (in_range_local(h, s, v, blue_range)) is_marker = true;
            if (in_range_local(h, s, v, red_range)) is_marker = true;

            if (!is_marker && s_bin > thresh) {
                binary.pdata[j * W + i] = 255;
            } else {
                binary.pdata[j * W + i] = 0;
            }
        }
    }

    // existing morphology / extract / filter (same as other overload)
    image tmp;
    alloc_grey(tmp, W, H);

    // opening
    erode(binary, tmp);
    erode(tmp, tmp);
    copy(tmp, binary);

    dialate(binary, tmp);
    dialate(tmp, binary);

    // closing (bridge small gaps)
    dialate(binary, tmp);
    copy(tmp, binary);
    erode(binary, tmp);
    copy(tmp, binary);

    auto obs = extract_obstacles(binary, grey);
    int computed_min = std::max(min_area_param, (W * H) / 50000);
    std::vector<Obstacle> filtered;
    for (auto &o : obs) if (o.area >= computed_min) filtered.push_back(o);

    // Debug: print threshold and robot mask coverage
    int robot_mask_count = 0;
    for (int i = 0; i < W * H; ++i) robot_mask_count += (robot_mask.pdata[i] != 0);
    int binary_count = 0;
    for (int i = 0; i < W * H; ++i) binary_count += (binary.pdata[i] != 0);

    std::cerr << "[Obstacles] s_thresh=" << s_thresh
              << "  robot_mask_pct=" << (100.0 * robot_mask_count) / (W * H)
              << "%  binary_px=" << binary_count << " / " << (W*H)
              << std::endl;

    // Optional: save binary mask to a simple RGB file so you can view it
    image dbg_rgb;
    dbg_rgb.type = RGB_IMAGE;
    dbg_rgb.width = W;
    dbg_rgb.height = H;
    allocate_image(dbg_rgb);
    // copy binary -> rgb (R=G=B=binary)
    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            unsigned char v = binary.pdata[j * W + i];
            int idx = 3 * (j * W + i);
            dbg_rgb.pdata[idx] = v;
            dbg_rgb.pdata[idx + 1] = v;
            dbg_rgb.pdata[idx + 2] = v;
        }
    }
    save_rgb_image("obstacles_binary_debug.bmp", dbg_rgb);
    free_image(dbg_rgb);

    free_image(tmp);
    free_image(binary);
    free_image(grey);
    free_image(robot_mask);

    return filtered;
}