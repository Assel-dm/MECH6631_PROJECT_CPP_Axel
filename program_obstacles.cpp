// Non-interactive marker detection and obstacle filtering
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <Windows.h>
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

/*int main()
{
    activate_vision();

    double t_duration;
    int width = 720, height = 1280;
    open_video_input("Physical_test.mp4", t_duration, width, height);

    image rgb;
    rgb.type = RGB_IMAGE;
    rgb.width = width;
    rgb.height = height;
    allocate_image(rgb);

    MarkerDetector detector;

    // ============================================================
    // TUNABLE PARAMETERS
    // ============================================================
    
    detector.blue_range.h_lo = 200.0;
    detector.blue_range.h_hi = 250.0;
    detector.blue_range.s_min = 0.60;
    detector.blue_range.v_min = 40;

    detector.red_range.h_lo = 0.0;
    detector.red_range.h_hi = 15.0;
    detector.red_range.s_min = 0.35;
    detector.red_range.v_min = 60;

    detector.min_blob_area = 2000;
    detector.max_blob_area = 10000;
    detector.min_area_ratio = 0.20;

    double pair_tolerance = 0.3;
    double pair_max_distance = 200.0;

    float kL = 2.5f;
    float ka = 2.0f;
    float kb = 2.0f;

    double safety_factor = 1.4;
    int min_area = 1500;
    int inflate_px = 20;
    int cell_px = 10;
    
    double marker_sep_in = 9.0;
    double robot_length_in = 15.0;
    double robot_width_in = 11.0;

    Obstacles obst;
    OccupancyGrid occBuilder;

    while (true) {
        double t_sample;
        read_video_input(rgb, t_sample);
        if (t_sample < 1e-6) {
            break;  // End of video
        }

        std::vector<Blob> front_blobs, rear_blobs;
        detector.detect_markers(rgb, front_blobs, rear_blobs);

        auto est_sep = estimate_marker_sep_px(front_blobs, rear_blobs);
        std::vector<RobotDet> dets = Tracker().pairMarkers(front_blobs, rear_blobs, est_sep, pair_tolerance, pair_max_distance);

        // Draw pairs
        for (auto& d : dets) {
            draw_line_rgb(rgb, (int)d.front.x, (int)d.front.y, (int)d.rear.x, (int)d.rear.y, 0, 255, 0);
            draw_circle_rgb(rgb, (int)d.front.x, (int)d.front.y, 12, 0, 255, 0);
            draw_circle_rgb(rgb, (int)d.rear.x, (int)d.rear.y, 12, 0, 255, 0);
            
            char buf[64];
            sprintf(buf, "sep=%.1f", d.sep_px);
            draw_text_rgb(rgb, (int)d.x + 8, (int)d.y - 20, buf, 0, 255, 0);
        }

        // Compute robot dimensions
        int robot_length_px = 60;
        int robot_width_px = 40;
        
        if (est_sep.has_value()) {
            double px_per_in = est_sep.value() / marker_sep_in;
            robot_length_px = (int)(robot_length_in * px_per_in * safety_factor);
            robot_width_px = (int)(robot_width_in * px_per_in * safety_factor);
        }

        // Process obstacles
        auto pipeline_res = process_frame_obstacles(
            rgb, dets, obst, occBuilder,
            kL, ka, kb,
            min_area, cell_px, inflate_px, 
            robot_length_px, robot_width_px
        );

        // Draw obstacles
        for (auto& obs : pipeline_res.obstacles) {
            draw_obstacle_overlay(rgb, obs, 255, 128, 0);
        }
        
        // 🆕 Draw robot masks (semi-transparent yellow/white outline)
        for (auto& d : dets) {
            draw_robot_mask_overlay(rgb, d, robot_length_px, robot_width_px, 255, 255, 100);
        }

        view_rgb_image(rgb);
    }

    free_image(rgb);
    close_video_input();
    deactivate_vision();
    
    return 0;
}*/