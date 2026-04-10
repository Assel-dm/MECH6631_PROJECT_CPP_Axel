#include "MarkerDetector.h"
#include <cmath>
#include <cstring>
#include <iostream>

MarkerDetector::MarkerDetector()
{
    // BLUE marker range (tighter)
    blue_range.h_lo = 180;   // degrees
    blue_range.h_hi = 260;
    blue_range.s_min = 0.45; // 0–1
    blue_range.v_min = 80;   // 0–255

    // RED marker range (wraps around 350–360 OR 0–10)
    // Use wrap-around by setting h_lo > h_hi
    red_range.h_lo = 350;   // start of wrap
    red_range.h_hi = 10;    // end of wrap
    red_range.s_min = 0.45; // lower saturation threshold
    red_range.v_min = 80;   // lower value threshold
}

void MarkerDetector::rgb_to_hsv(ibyte R, ibyte G, ibyte B,
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

bool MarkerDetector::in_range(double h, double s, double v, const HSVRange& range)
{
    if (s < range.s_min) return false;
    if (v < range.v_min) return false;

    // Handle wrap-around (e.g., red)
    if (range.h_lo > range.h_hi)
        return (h >= range.h_lo || h <= range.h_hi);

    return (h >= range.h_lo && h <= range.h_hi);
}

void MarkerDetector::extract_blobs(
    image& binary,
    image& grey_for_centroid,
    std::vector<Blob>& out_blobs)
{
    image label;
    label.type = LABEL_IMAGE;
    label.width = binary.width;
    label.height = binary.height;
    allocate_image(label);

    int nlabels = 0;
    label_image(binary, label, nlabels);

    i2byte* pl = (i2byte*)label.pdata;

    for (int L = 1; L <= nlabels; L++) {

        // Count area
        int area = 0;
        for (int j = 0; j < label.height; j++)
            for (int i = 0; i < label.width; i++)
                if (pl[j * label.width + i] == L)
                    area++;

        // Size filtering (tuned)
        if (area < 60 || area > 3000)
            continue;

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
}

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

    // GREY for centroid
    copy(rgb, grey);

    // Init masks
    std::memset(bin_blue.pdata, 0, W * H);
    std::memset(bin_red.pdata, 0, W * H);

    // HSV thresholding
    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {

            int idx = 3 * (j * W + i);
            ibyte B = rgb.pdata[idx];
            ibyte G = rgb.pdata[idx + 1];
            ibyte R = rgb.pdata[idx + 2];

            double h, s, v;
            rgb_to_hsv(R, G, B, h, s, v);

            if (in_range(h, s, v, blue_range))
                bin_blue.pdata[j * W + i] = 255;

            if (in_range(h, s, v, red_range))
                bin_red.pdata[j * W + i] = 255;
        }
    }

    // Morphology
    image tmp;
    tmp.type = GREY_IMAGE;
    tmp.width = W;
    tmp.height = H;
    allocate_image(tmp);

    // BLUE: smooth (keep current)
    dialate(bin_blue, tmp);
    erode(tmp, bin_blue);

    // RED: use closing (dilate -> erode) to join/red regions and close small gaps
    dialate(bin_red, tmp);
    erode(tmp, bin_red);

    // Extract blobs
    extract_blobs(bin_blue, grey, front_blobs);
    extract_blobs(bin_red, grey, rear_blobs);

    free_image(tmp);
    free_image(grey);
    free_image(bin_blue);
    free_image(bin_red);
}