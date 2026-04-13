#include "ObstaclePipeline.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iostream>

// Helper: fill oriented rectangle into mask
static void draw_oriented_rect_mask(image& mask, 
                                     double cx, double cy, double theta,
                                     int length, int width)
{
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);
    
    double hl = length / 2.0;
    double hw = width / 2.0;
    
    double corners[4][2] = {
        {-hl, -hw}, {+hl, -hw}, {+hl, +hw}, {-hl, +hw}
    };
    
    int px[4], py[4];
    for (int i = 0; i < 4; ++i) {
        double rx = corners[i][0];
        double ry = corners[i][1];
        double wx = cx + rx * cos_th - ry * sin_th;
        double wy = cy + rx * sin_th + ry * cos_th;
        px[i] = (int)std::round(wx);
        py[i] = (int)std::round(wy);
    }
    
    int minx = px[0], maxx = px[0], miny = py[0], maxy = py[0];
    for (int i = 1; i < 4; ++i) {
        if (px[i] < minx) minx = px[i];
        if (px[i] > maxx) maxx = px[i];
        if (py[i] < miny) miny = py[i];
        if (py[i] > maxy) maxy = py[i];
    }
    
    int W = mask.width, H = mask.height;
    minx = std::max(0, minx);
    maxx = std::min(W - 1, maxx);
    miny = std::max(0, miny);
    maxy = std::min(H - 1, maxy);
    
    for (int y = miny; y <= maxy; ++y) {
        for (int x = minx; x <= maxx; ++x) {
            bool inside = true;
            for (int i = 0; i < 4; ++i) {
                int j = (i + 1) % 4;
                double dx = px[j] - px[i];
                double dy = py[j] - py[i];
                double dpx = x - px[i];
                double dpy = y - py[i];
                double cross = dx * dpy - dy * dpx;
                if (cross < 0) { inside = false; break; }
            }
            if (inside) {
                mask.pdata[y * W + x] = 255;
            }
        }
    }
}

ObstaclePipelineResult process_frame_obstacles(
    image& rgb,
    const std::vector<RobotDet>& robot_dets,
    Obstacles& obst,
    OccupancyGrid& grid_builder,
    float kL, float ka, float kb,
    int min_area,
    int cell_px,
    int inflate_px,
    int robot_length_px,
    int robot_width_px)
{
    ObstaclePipelineResult res;
    int W = rgb.width;
    int H = rgb.height;

    image robot_mask;
    robot_mask.type = GREY_IMAGE;
    robot_mask.width = W;
    robot_mask.height = H;
    allocate_image(robot_mask);
    std::memset(robot_mask.pdata, 0, (size_t)W * H);

    for (const auto &det : robot_dets) {
        draw_oriented_rect_mask(robot_mask, det.x, det.y, det.theta, 
                                robot_length_px, robot_width_px);
    }

    // DEBUG: Save robot_mask to verify dimensions
    image dbg_mask_rgb;
    dbg_mask_rgb.type = RGB_IMAGE;
    dbg_mask_rgb.width = W;
    dbg_mask_rgb.height = H;
    allocate_image(dbg_mask_rgb);
    for (int i = 0; i < W * H; ++i) {
        unsigned char v = robot_mask.pdata[i];
        dbg_mask_rgb.pdata[3*i] = v;
        dbg_mask_rgb.pdata[3*i+1] = v;
        dbg_mask_rgb.pdata[3*i+2] = v;
    }
    save_rgb_image("robot_mask_debug.bmp", dbg_mask_rgb);
    free_image(dbg_mask_rgb);

    std::vector<Obstacle> obstacles = obst.detect_floor_model(
        rgb, &robot_mask, kL, ka, kb, min_area
    );

    std::vector<Obstacle> filtered;
    for (const auto &o : obstacles) {
        int cx_px = (int)std::round(o.cx);
        int cy_px = (int)std::round(o.cy);
        
        if (cx_px >= 0 && cx_px < W && cy_px >= 0 && cy_px < H) {
            if (robot_mask.pdata[cy_px * W + cx_px] == 0) {
                filtered.push_back(o);
            }
        }
    }

    Grid occ = grid_builder.build(filtered, W, H, cell_px, inflate_px);

    free_image(robot_mask);

    res.obstacles = std::move(filtered);
    res.occ_grid = std::move(occ);
    return res;
}