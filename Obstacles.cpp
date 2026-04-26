#include "Obstacles.h"
#include "MarkerDetector.h"
#include "image_transfer.h"
#include "vision.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <vector>

Obstacles::Obstacles() {}

void Obstacles::alloc_grey(image& img, int w, int h) {
    img.type = GREY_IMAGE;
    img.width = w;
    img.height = h;
    allocate_image(img);
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

        int min_area = std::max(50, (W*H) / 50000);
        if (area < min_area) continue;

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

// ============================================================================
// RGB to CIE Lab conversion
// ============================================================================
static void rgb_to_lab(float R255, float G255, float B255, float &L, float &a, float &b)
{
    // Linearize sRGB
    auto toLin = [](float c) {
        float sr = c / 255.0f;
        if (sr <= 0.04045f) return sr / 12.92f;
        return std::pow((sr + 0.055f) / 1.055f, 2.4f);
    };
    float r = toLin(R255);
    float g = toLin(G255);
    float bl = toLin(B255);

    // Linear RGB -> XYZ (D65)
    float X = 0.4124564f * r + 0.3575761f * g + 0.1804375f * bl;
    float Y = 0.2126729f * r + 0.7151522f * g + 0.0721750f * bl;
    float Z = 0.0193339f * r + 0.1191920f * g + 0.9503041f * bl;

    // Reference white D65
    const float Xn = 0.95047f, Yn = 1.0f, Zn = 1.08883f;

    auto f = [](float t) {
        if (t > 0.008856f) return std::cbrt(t);
        return (7.787f * t) + (16.0f / 116.0f);
    };

    float fx = f(X / Xn);
    float fy = f(Y / Yn);
    float fz = f(Z / Zn);

    L = 116.0f * fy - 16.0f;
    a = 500.0f * (fx - fy);
    b = 200.0f * (fy - fz);
}

// Helper: draw filled circle into mask (add after rgb_to_lab function)
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

// ============================================================================
// FLOOR MODEL DETECTOR: Lab z-score based (lighting adaptive)
// ============================================================================
std::vector<Obstacle> Obstacles::detect_floor_model(image& rgb,
                                                    image* robot_mask,
                                                    float kL, float ka, float kb,
                                                    int min_area_param)
{
    int W = rgb.width;
    int H = rgb.height;

    image grey;
    alloc_grey(grey, W, H);
    copy(rgb, grey);

    double sumL=0.0, sumA=0.0, sumB=0.0;
    size_t count = 0;
    int border_margin = 0;
    
    for (int j = border_margin; j < H - border_margin; ++j) {
        for (int i = border_margin; i < W - border_margin; ++i) {
            int idx = j * W + i;
            if (robot_mask && robot_mask->pdata[idx]) continue;
            
            int ridx = 3 * idx;
            float R = (float)rgb.pdata[ridx + 2];
            float G = (float)rgb.pdata[ridx + 1];
            float B = (float)rgb.pdata[ridx + 0];
            
            float Lf, af, bf;
            rgb_to_lab(R, G, B, Lf, af, bf);
            sumL += Lf; sumA += af; sumB += bf;
            ++count;
        }
    }
    
    if (count < 16) {
        free_image(grey);
        return {};
    }
    
    double muL = sumL / (double)count;
    double muA = sumA / (double)count;
    double muB = sumB / (double)count;

    double sL=0.0, sA=0.0, sB=0.0;
    for (int j = border_margin; j < H - border_margin; ++j) {
        for (int i = border_margin; i < W - border_margin; ++i) {
            int idx = j * W + i;
            if (robot_mask && robot_mask->pdata[idx]) continue;
            
            int ridx = 3 * idx;
            float R = (float)rgb.pdata[ridx + 2];
            float G = (float)rgb.pdata[ridx + 1];
            float B = (float)rgb.pdata[ridx + 0];
            
            float Lf, af, bf;
            rgb_to_lab(R, G, B, Lf, af, bf);
            sL += (Lf - muL) * (Lf - muL);
            sA += (af - muA) * (af - muA);
            sB += (bf - muB) * (bf - muB);
        }
    }
    
    double denom = std::max<size_t>(1, count - 1);
    double sigL = std::sqrt(sL / denom) + 1e-6;
    double sigA = std::sqrt(sA / denom) + 1e-6;
    double sigB = std::sqrt(sB / denom) + 1e-6;

    image obs_mask;
    alloc_grey(obs_mask, W, H);
    std::memset(obs_mask.pdata, 0, W * H);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            int idx = j * W + i;
            
            if (i < border_margin || i >= W - border_margin ||
                j < border_margin || j >= H - border_margin) {
                continue;
            }
            if (robot_mask && robot_mask->pdata[idx]) continue;
            
            int ridx = 3 * idx;
            float R = (float)rgb.pdata[ridx + 2];
            float G = (float)rgb.pdata[ridx + 1];
            float B = (float)rgb.pdata[ridx + 0];
            
            float Lf, af, bf;
            rgb_to_lab(R, G, B, Lf, af, bf);
            
            double zL = std::abs((Lf - muL) / sigL);
            double zA = std::abs((af - muA) / sigA);
            double zB = std::abs((bf - muB) / sigB);

            if ((zL > kL) || (zA > ka) || (zB > kb)) {
                obs_mask.pdata[idx] = 255;
            }
        }
    }

    image tmp;
    alloc_grey(tmp, W, H);

    erode(obs_mask, tmp);
    copy(tmp, obs_mask);
    erode(obs_mask, tmp);
    copy(tmp, obs_mask);

    dialate(obs_mask, tmp);
    copy(tmp, obs_mask);
    dialate(obs_mask, tmp);
    copy(tmp, obs_mask);

    dialate(obs_mask, tmp);
    copy(tmp, obs_mask);
    erode(obs_mask, tmp);
    copy(tmp, obs_mask);

    auto obs = extract_obstacles(obs_mask, grey);

    int computed_min = std::max(min_area_param, (W * H) / 50000);
    std::vector<Obstacle> filtered;
    
    for (auto &o : obs) {
        if (o.area < computed_min) continue;
        
        double aspect = (double)o.w / (double)o.h;
        if (aspect > 6.0 || aspect < 0.16) continue;
        
        if (o.w > W/2 || o.h > H/2) continue;
        
        double fill_ratio = (double)o.area / (o.w * o.h);
        if (fill_ratio < 0.2) continue;
        
        filtered.push_back(o);
    }

    // Save debug images (keep these)
    image dbg_rgb;
    dbg_rgb.type = RGB_IMAGE;
    dbg_rgb.width = W;
    dbg_rgb.height = H;
    allocate_image(dbg_rgb);
    
    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            unsigned char v = obs_mask.pdata[j * W + i];
            int idx = 3 * (j * W + i);
            dbg_rgb.pdata[idx] = v;
            dbg_rgb.pdata[idx + 1] = v;
            dbg_rgb.pdata[idx + 2] = v;
        }
    }
    save_rgb_image("obstacles_floor_model_debug.bmp", dbg_rgb);
    free_image(dbg_rgb);

    image dbg_overlay;
    dbg_overlay.type = RGB_IMAGE;
    dbg_overlay.width = W;
    dbg_overlay.height = H;
    allocate_image(dbg_overlay);
    copy(rgb, dbg_overlay);

    for (int j = 0; j < H; ++j) {
        for (int i = 0; i < W; ++i) {
            int idx = j * W + i;
            if (obs_mask.pdata[idx]) {
                int ridx = 3 * idx;
                dbg_overlay.pdata[ridx]     = (ibyte)std::min(255, dbg_overlay.pdata[ridx] + 160);
                dbg_overlay.pdata[ridx + 1] = (ibyte)(dbg_overlay.pdata[ridx + 1] / 2);
                dbg_overlay.pdata[ridx + 2] = (ibyte)(dbg_overlay.pdata[ridx + 2] / 2);
            }
        }
    }
    save_rgb_image("obstacles_floor_model_overlay_debug.bmp", dbg_overlay);
    free_image(dbg_overlay);

    free_image(tmp);
    free_image(obs_mask);
    free_image(grey);

    return filtered;
}

// Wrapper: build robot_mask from robot_blobs and call detect_floor_model(...)
std::vector<Obstacle> Obstacles::detect_floor_model(image& rgb,
                                                    const std::vector<Blob>& robot_blobs,
                                                    float kL, float ka, float kb,
                                                    int min_area_param)
{
    int W = rgb.width;
    int H = rgb.height;

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

    auto res = detect_floor_model(rgb, &robot_mask, kL, ka, kb, min_area_param);

    free_image(robot_mask);
    return res;
}