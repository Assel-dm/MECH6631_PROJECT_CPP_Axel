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

        // tuning (try more permissive values)
        float kL = 1.2f, ka = 0.7f, kb = 0.7f; // looser z-score thresholds
        int min_area = 200;                    // much smaller to see any detections
        int cell_px = 10;
        int inflate_px = 20;                   // small inflation

        // build robot_blobs from detected marker blobs (front + rear)
        std::vector<Blob> robot_blobs = front_blobs;
        robot_blobs.insert(robot_blobs.end(), rear_blobs.begin(), rear_blobs.end());

        // run obstacle pipeline using the allocated `rgb` image
        auto pipeline_res = process_frame_obstacles(
            rgb, robot_blobs, obst, occBuilder,
            kL, ka, kb, min_area, cell_px, inflate_px, 30, 45
        );

        auto& obstacles = pipeline_res.obstacles;
        Grid occ_grid = pipeline_res.occ_grid;

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