// Small test harness: run marker detection and pairing on video frames.
// Add this file to the project and run. Uses existing vision/image_transfer drawing functions.
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <Windows.h>
#include <iostream>
#include <vector>
#include <optional>

#include "image_transfer.h"
#include "vision.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "Types.h"
#include "Overlay.h"
#include "Obstacles.h"
#include "ObstaclePipeline.h"
#include "StrategyEngine.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

int main()
{
    std::cout << "Press SPACE to start..." << std::endl;
    pause();

    activate_vision();

    double t_duration;
    int width = 720, height = 1280;
    open_video_input("robot_test2.mp4", t_duration, width, height);

    std::cout << "width=" << width << " height=" << height << " duration(s)=" << t_duration << std::endl;

    image rgb;
    rgb.type = RGB_IMAGE;
    rgb.width = width;
    rgb.height = height;
    allocate_image(rgb);

    MarkerDetector detector;

    Obstacles obst;
    OccupancyGrid occBuilder;

    // Start with these values and adjust based on your lighting:
    float kL = 2.5f;  // Lightness sensitivity (2.0 - 3.5)
    float ka = 2.0f;  // Red-green sensitivity (1.5 - 2.5)
    float kb = 2.0f;  // Blue-yellow sensitivity (1.5 - 2.5)

    // Lower values = more sensitive (detect more obstacles)
    // Higher values = less sensitive (fewer false positives)

    while (true) {
        if (KEY('X')) break;

        double t_sample;
        read_video_input(rgb, t_sample);
        if (t_sample < 1e-6) break;

        std::vector<Blob> front_blobs, rear_blobs;
        detector.detect_markers(rgb, front_blobs, rear_blobs);

        // Estimate sep and pair
        auto est_sep = estimate_marker_sep_px(front_blobs, rear_blobs);
        std::vector<RobotDet> dets = Tracker().pairMarkers(front_blobs, rear_blobs, est_sep, 0.55, 1200.0);

        // Draw detections
        for (auto& b : front_blobs) draw_circle_rgb(rgb, (int)b.x, (int)b.y, 8, 0, 0, 255);
        for (auto& b : rear_blobs)  draw_circle_rgb(rgb, (int)b.x, (int)b.y, 8, 255, 0, 0);
        for (auto& d : dets) {
            draw_line_rgb(rgb, (int)d.front.x, (int)d.front.y, (int)d.rear.x, (int)d.rear.y, 0, 255, 0);
            char buf[64];
            sprintf(buf, "sep=%.1f", d.sep_px);
            draw_text_rgb(rgb, (int)d.x + 8, (int)d.y - 8, buf, 0, 255, 0);
        }

        // tuning
        int min_area = 1500;          // min obstacle area (px²)
        int cell_px = 10;
        int inflate_px = 20;

        // If you know marker separation and physical robot size:
        double marker_sep_in = 9.0;
        double robot_length_in = 15.0;  // Changed from 12.0
        double robot_width_in = 11.0;   // Changed from 8.0

        // Robot body dimensions (in pixels)
        int robot_length_px;   // front-to-rear length (along heading)
        int robot_width_px;    // side-to-side width (perpendicular to heading)


        // Compute scale if you have est_sep
        if (est_sep.has_value()) {
            double px_per_in = est_sep.value() / marker_sep_in;
            
            double safety_factor = 1.0;  // Add safety margin
            robot_length_px = (int)(robot_length_in * px_per_in * safety_factor);
            robot_width_px = (int)(robot_width_in * px_per_in * safety_factor);
        }

        // Run obstacle pipeline using paired detections (oriented rectangles)
        auto pipeline_res = process_frame_obstacles(
            rgb, dets, obst, occBuilder,
            kL, ka, kb,  // <-- RESTORED
            min_area, cell_px, inflate_px, 
            robot_length_px, robot_width_px
        );

        auto& obstacles = pipeline_res.obstacles;
        Grid occ_grid = pipeline_res.occ_grid;

        // Draw detected obstacles (bounding boxes + area labels)
        for (auto& obs : obstacles) {
            draw_obstacle_overlay(rgb, obs, 255, 128, 0); // orange
        }

        // Optional: draw occupancy grid cells
        int gh = occ_grid.size();
        if (gh > 0) {
            int gw = occ_grid[0].size();
            for (int gy = 0; gy < gh; ++gy) {
                for (int gx = 0; gx < gw; ++gx) {
                    if (occ_grid[gy][gx] == 1) {
                        int x0 = gx * cell_px;
                        int y0 = gy * cell_px;
                        int x1 = std::min(width - 1, (gx + 1) * cell_px - 1);
                        int y1 = std::min(height - 1, (gy + 1) * cell_px - 1);
                        // draw_rect_rgb(rgb, x0, y0, x1 - x0 + 1, y1 - y0 + 1, 200, 0, 0); // dark red outline
                    }
                }
            }
        }

        view_rgb_image(rgb);

        // small throttle to avoid burning CPU (leave as 1ms)
        Sleep(1);
    }

    free_image(rgb);
    close_video_input();
    deactivate_vision();
    std::cout << "Done." << std::endl;
    return 0;
}