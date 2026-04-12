#include "ObstaclePipeline.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iostream>

// draw filled circle into mask
static void draw_filled_circle_mask(image& mask, int cx, int cy, int r) {
    int W = mask.width, H = mask.height;
    if (r <= 0) return;
    int r2 = r * r;
    int y0 = std::max(0, cy - r);
    int y1 = std::min(H - 1, cy + r);
    for (int y = y0; y <= y1; ++y) {
        int dy = y - cy;
        int dxmax = (int)std::floor(std::sqrt((double)r2 - (double)dy*dy));
        int x0 = std::max(0, cx - dxmax);
        int x1 = std::min(W - 1, cx + dxmax);
        for (int x = x0; x <= x1; ++x) {
            mask.pdata[y * W + x] = 255;
        }
    }
}

// draw thick filled line between two points into mask
static void draw_filled_line_mask(image& mask, int x0, int y0, int x1, int y1, int thickness) {
    if (thickness <= 1) {
        // sample points along the segment and stamp circles
        int dx = x1 - x0, dy = y1 - y0;
        double L = std::hypot((double)dx, (double)dy);
        int steps = std::max(1, (int)std::ceil(L));
        for (int s = 0; s <= steps; ++s) {
            double t = (double)s / (double)steps;
            int x = (int)std::round(x0 + t * dx);
            int y = (int)std::round(y0 + t * dy);
            draw_filled_circle_mask(mask, x, y, thickness / 2);
        }
        return;
    }
    // for larger thickness reuse same stamping approach
    draw_filled_line_mask(mask, x0, y0, x1, y1, thickness);
}

ObstaclePipelineResult process_frame_obstacles(
    image& rgb,
    const std::vector<Blob>& robot_blobs,
    Obstacles& obst,
    OccupancyGrid& grid_builder,
    float kL, float ka, float kb,
    int min_area,
    int cell_px,
    int inflate_px,
    int robot_circle_radius_px,
    int robot_body_width_px)
{
    ObstaclePipelineResult res;

    int W = rgb.width;
    int H = rgb.height;

    // 1) build robot exclusion mask
    image robot_mask;
    robot_mask.type = GREY_IMAGE;
    robot_mask.width = W;
    robot_mask.height = H;
    allocate_image(robot_mask);
    std::memset(robot_mask.pdata, 0, (size_t)W * H);

    for (const auto &b : robot_blobs) {
        int cx = (int)std::round(b.x);
        int cy = (int)std::round(b.y);
        draw_filled_circle_mask(robot_mask, cx, cy, robot_circle_radius_px);
    }

    // debug: robot mask coverage
    int robot_mask_count = 0;
    for (int i = 0; i < W * H; ++i) robot_mask_count += (robot_mask.pdata[i] != 0);
    double robot_mask_pct = 100.0 * (double)robot_mask_count / (double)(W * H);
    std::cerr << "[ObstaclePipeline] robot_mask_count=" << robot_mask_count
              << " (" << robot_mask_pct << "%)" << std::endl;

    // 2) call existing floor-model detector with auto-relax loop when needed
    std::vector<Obstacle> obstacles;
    float kL_try = kL, ka_try = ka, kb_try = kb;
    const float kL_min = 0.8f, kAB_min = 0.5f;
    const int max_iters = 5;
    bool found = false;

    for (int iter = 0; iter < max_iters && !found; ++iter) {
        obstacles = obst.detect_floor_model(rgb, &robot_mask, kL_try, ka_try, kb_try, min_area);
        int total_px = 0;
        for (const auto &o : obstacles) total_px += (int)o.area;
        if (!obstacles.empty() && total_px > 0) {
            found = true;
            std::cerr << "[ObstaclePipeline] found obstacles with k=(" << kL_try << "," << ka_try << "," << kb_try << ") count=" << obstacles.size() << " total_area=" << total_px << std::endl;
            break;
        }
        // relax thresholds
        kL_try = std::max(kL_min, kL_try * 0.75f);
        ka_try = std::max(kAB_min, ka_try * 0.7f);
        kb_try = std::max(kAB_min, kb_try * 0.7f);
        std::cerr << "[ObstaclePipeline] relax attempt " << (iter+1) << " -> k=(" << kL_try << "," << ka_try << "," << kb_try << ")" << std::endl;
    }

    if (!found) {
        std::cerr << "[ObstaclePipeline] no obstacles after relaxation; returning empty list" << std::endl;
    }

    // 3) convert to occupancy grid
    Grid occ = grid_builder.build(obstacles, W, H, cell_px, inflate_px);

    free_image(robot_mask);

    res.obstacles = std::move(obstacles);
    res.occ_grid = std::move(occ);
    return res;
}