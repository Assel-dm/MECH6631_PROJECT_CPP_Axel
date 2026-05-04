// Fully Autonomous Physical Robot - Live Camera + Vision + Planning + Control + IDDance
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <Windows.h>
#include <iostream>
#include <vector>
#include <optional>
#include <cmath>
#include <deque>

#include "image_transfer.h"
#include "vision.h"
#include "timer.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "Types.h"
#include "Overlay.h"
#include "Obstacles.h"
#include "ObstaclePipeline.h"
#include "AStar.h"
#include "OccupancyGrid.h"
#include "Waypoint.h"
#include "Fuzzy.h"
#include "IDDance.h"

using namespace std;

// Keyboard macro
#define KEY(c) (GetAsyncKeyState((int)(c)) & 0x8000)

// ============================================================
// Bluetooth Serial Communication
// ============================================================
static HANDLE openSerial(const char* port, DWORD baud)
{
    HANDLE h = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        cerr << "ERROR: Could not open " << port << endl;
        return INVALID_HANDLE_VALUE;
    }
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(h, &dcb);

    COMMTIMEOUTS timeouts = {};
    timeouts.WriteTotalTimeoutConstant   = 20;
    timeouts.WriteTotalTimeoutMultiplier = 2;
    SetCommTimeouts(h, &timeouts);

    cout << "Bluetooth serial opened on " << port << " @ " << baud << " baud" << endl;
    return h;
}

static void sendCommand(HANDLE hSerial, const Command& cmd)
{
    if (hSerial == INVALID_HANDLE_VALUE) return;

    int pw_l = vel_to_pw_left(cmd.left);
    int pw_r = vel_to_pw_right(cmd.right);

    uint8_t buf[6] = {
        0xFF,
        (uint8_t)(pw_l >> 8), (uint8_t)(pw_l & 0xFF),
        (uint8_t)(pw_r >> 8), (uint8_t)(pw_r & 0xFF),
        (uint8_t)(cmd.laser ? 1 : 0)
    };
    DWORD written;
    WriteFile(hSerial, buf, sizeof(buf), &written, NULL);
}

// ============================================================
// Main
// ============================================================
int main()
{
    SetConsoleOutputCP(CP_UTF8);
    
    cout << "\n========================================" << endl;
    cout << "   AUTONOMOUS PHYSICAL ROBOT" << endl;
    cout << "   Live Camera + Strategy Engine" << endl;
    cout << "========================================" << endl;
    
    // ============================================================
    // Configuration
    // ============================================================
    const char*  BT_PORT   = "COM5";   // ← Your Bluetooth COM port
    const DWORD  BT_BAUD   = 9600;
    const int    CAM_INDEX = 0;        // Camera index (try 1 or 2 if 0 fails)
    const int    CAM_W     = 1920;
    const int    CAM_H     = 1080;
    const double TEST_DURATION = 120.0; // Auto-stop after seconds (0 = disabled)

    // Diagnostics logging
    const bool ENABLE_DIAGNOSTICS = true;
    const char* DIAG_CSV_PATH = "run_diagnostics.csv";
    
    // Mode selection
    cout << "\nSelect Mode:" << endl;
    cout << "  1 - Navigation Mode (move to opposite corner)" << endl;
    cout << "  2 - Offense Mode (track and shoot enemy robot)" << endl;
    cout << "\nEnter mode (1 or 2): ";
    int mode_choice;
    cin >> mode_choice;
    bool offense_mode = (mode_choice == 2);
    
    // ============================================================
    // Initialize Hardware
    // ============================================================
    HANDLE hSerial = openSerial(BT_PORT, BT_BAUD);
    if (hSerial == INVALID_HANDLE_VALUE) {
        cerr << "Failed to open Bluetooth. Exiting." << endl;
        return -1;
    }
    
    if (!activate_camera(CAM_INDEX, CAM_H, CAM_W)) {
        cerr << "ERROR: Could not open camera " << CAM_INDEX << endl;
        CloseHandle(hSerial);
        return -1;
    }
    cout << "Camera opened: " << CAM_W << "x" << CAM_H << endl;

    std::ofstream diag;
    if (ENABLE_DIAGNOSTICS) {
        diag.open(DIAG_CSV_PATH);
        if (diag.is_open()) {
            diag << "t,frame,mode,pwm_left,pwm_right,laser,"
                 << "my_x,my_y,enemy_x,enemy_y,enemy_dist_px,enemy_bearing_deg,"
                 << "nearest_obs_dist_px,num_obstacles,path_size,has_goal,path_planned,"
                 << "detect_ms,obstacles_ms,astar_ms,control_ms,total_ms,fps\n";
            diag << std::fixed << std::setprecision(6);
            cout << "Diagnostics logging enabled: " << DIAG_CSV_PATH << endl;
        } else {
            cerr << "WARNING: could not open diagnostics CSV file" << endl;
        }
    }
    
    // ============================================================
    // Vision & Obstacle Pipeline Setup
    // ============================================================
    image rgb;
    rgb.type = RGB_IMAGE;
    rgb.width = CAM_W;
    rgb.height = CAM_H;
    allocate_image(rgb);
    
    // HSV ranges (tuned for physical environment)
    MarkerDetector detector;
    detector.blue_range = {200.0, 240.0, 0.3, 50};   // Blue front marker
    detector.red_range = {0.0, 20.0, 0.3, 50};       // Red rear marker
    detector.min_blob_area = 2000;
    detector.max_blob_area = 10000;
    
    Tracker tracker;
    
    const double pair_tolerance = 0.5;
    const double pair_max_distance = 200.0;
    
    // Obstacle detection setup
    const double safety_factor = 1.4;
    const double marker_sep_in = 4.0;
    const double robot_length_in = 6.0;
    const double robot_width_in = 5.0;
    const double kL = 2.5;
    const double ka = 2.0;
    const double kb = 2.0;
    const int cell_px = 10;
    const int inflate_px = 20;
    const int min_obstacle_area = 1500;
    
    Obstacles obst;
    OccupancyGrid occBuilder;
    
    // Strategy components
    AStarPlanner planner;
    WaypointFollower follower;
    FuzzyLogic fuzzy;
    IDDance id_dance;
    
    // ============================================================
    // ID Dance Phase
    // ============================================================
    cout << "\n=== ID DANCE PHASE ===" << endl;
    cout << "Robot will perform identification dance..." << endl;
    
    bool id_assigned = false;
    int my_id = -1;
    double tc0 = high_resolution_time();
    double tc = 0.0;
    const double ID_DANCE_DURATION = 10.0;
    
    vector<RobotTrack> tracks;
    int frame_count = 0;
    
    cout << "Starting ID dance in 2 seconds..." << endl;
    Sleep(2000);
    
    while (tc < ID_DANCE_DURATION && !id_assigned) {
        tc = high_resolution_time() - tc0;
        
        // Grab frame from camera
        acquire_image(rgb, CAM_INDEX);
        
        // Detect markers
        double t_detect0 = high_resolution_time();
        std::vector<Blob> front_blobs, rear_blobs;
        detector.detect_markers(rgb, front_blobs, rear_blobs);
        
        auto est_sep = estimate_marker_sep_px(front_blobs, rear_blobs);
        std::vector<RobotDet> dets = tracker.pairMarkers(front_blobs, rear_blobs, est_sep, pair_tolerance, pair_max_distance);
        
        // Update tracking
        tracks = tracker.updateTracks(tracks, dets, tc, 80.0, 10);
        double detect_ms = (high_resolution_time() - t_detect0) * 1000.0;
        
        // Execute ID dance choreography (synchronous)
        int result_id = id_dance.run(tc, rgb, detector, tracker);
        Command dance_cmd = id_dance.currentCommand();
        sendCommand(hSerial, dance_cmd);
        
        // Check if dance is complete
        if (id_dance.done()) {
            my_id = id_dance.my_id();
            id_assigned = true;
            cout << "\n✓ ID Assigned: " << my_id << endl;
        }
        
        // Visualization
        for (auto& tr : tracks) {
            draw_circle_rgb(rgb, (int)tr.x, (int)tr.y, 15, 255, 255, 0);
            draw_arrow_rgb(rgb, (int)tr.x, (int)tr.y, tr.theta, 30, 255, 255, 0);
            
            char buf[32];
            sprintf_s(buf, "ID:%d", tr.id);
            draw_text_rgb(rgb, (int)tr.x + 20, (int)tr.y - 20, buf, 255, 255, 0);
        }
        
        draw_text_rgb(rgb, 10, 10, "ID DANCE", 255, 255, 0);
        char time_buf[32];
        sprintf_s(time_buf, "T:%.1fs", tc);
        draw_text_rgb(rgb, 10, 20, time_buf, 255, 255, 255);
        
        view_rgb_image(rgb);
        frame_count++;
    }
    
    // Stop after ID dance
    sendCommand(hSerial, {0.0, 0.0, false});
    Sleep(500);
    
    if (!id_assigned) {
        cout << "\n ID assignment failed! Defaulting to ID 0." << endl;
        my_id = 0;
    }
    
    // ============================================================
    // Autonomous Operation Phase
    // ============================================================
    cout << "\n=== AUTONOMOUS OPERATION ===" << endl;
    cout << "Starting autonomous navigation..." << endl;
    cout << "My Robot ID: " << my_id << endl;
    cout << "Mode: " << (offense_mode ? "OFFENSE" : "NAVIGATION") << endl;
    cout << "\nSystem will run autonomously. Press 'X' to stop.\n" << endl;
    
    Sleep(1000);
    
    // Path management (manual - WaypointFollower doesn't manage paths)
    vector<pair<double, double>> path_pixels;
    size_t current_waypoint_idx = 0;
    bool path_planned = false;
    bool goal_reached = false;
    int replan_counter = 0;
    const int REPLAN_INTERVAL = 30;
    const double WAYPOINT_REACHED_DIST = 60.0;
    
    tc0 = high_resolution_time();  // Reset time counter
    tc = 0.0;
    frame_count = 0;
    double last_loop_abs = high_resolution_time();
    
    while (true) {
        double loop_abs0 = high_resolution_time();
        double fps = 0.0;
        if (loop_abs0 > last_loop_abs) {
            fps = 1.0 / (loop_abs0 - last_loop_abs);
        }
        last_loop_abs = loop_abs0;
        if (KEY('X')) {
            cout << "\nStopped by user." << endl;
            break;
        }
        
        if (TEST_DURATION > 0.0 && tc > TEST_DURATION) {
            cout << "\nTest duration reached." << endl;
            break;
        }
        
        tc = high_resolution_time() - tc0;
        
        // Grab frame from live camera
        acquire_image(rgb, CAM_INDEX);
        
        // Detect markers
        double t_detect0 = high_resolution_time();
        std::vector<Blob> front_blobs, rear_blobs;
        detector.detect_markers(rgb, front_blobs, rear_blobs);
        
        auto est_sep = estimate_marker_sep_px(front_blobs, rear_blobs);
        std::vector<RobotDet> dets = tracker.pairMarkers(front_blobs, rear_blobs, est_sep, pair_tolerance, pair_max_distance);
        
        // Update tracking
        tracks = tracker.updateTracks(tracks, dets, tc, 80.0, 10);
        double detect_ms = (high_resolution_time() - t_detect0) * 1000.0;
        
        // Find my robot in tracking results
        RobotTrack* my_robot = nullptr;
        for (auto& tr : tracks) {
            if (tr.id == my_id && tr.misses < 3) {
                my_robot = &tr;
                break;
            }
        }
        
        if (!my_robot) {
            sendCommand(hSerial, {0.0, 0.0, false});
            draw_text_rgb(rgb, 10, 10, "ROBOT LOST", 255, 0, 0);
            view_rgb_image(rgb);
            continue;
        }
        
        // Compute robot dimensions from estimated marker separation
        int robot_length_px = 60;
        int robot_width_px = 40;
        if (est_sep.has_value()) {
            double px_per_in = est_sep.value() / marker_sep_in;
            robot_length_px = (int)(robot_length_in * px_per_in * safety_factor);
            robot_width_px = (int)(robot_width_in * px_per_in * safety_factor);
        }
        
        // Obstacle detection
        double t_obs0 = high_resolution_time();
        ObstaclePipelineResult pipeline_res = process_frame_obstacles(
            rgb, dets, obst, occBuilder,
            kL, ka, kb,
            min_obstacle_area, cell_px, inflate_px,
            robot_length_px, robot_width_px
        );
        double obstacles_ms = (high_resolution_time() - t_obs0) * 1000.0;
        
        // Goal determination
        bool has_goal = false;
        double goal_x = 0.0, goal_y = 0.0;
        
        if (offense_mode && tracks.size() > 1) {
            for (auto& tr : tracks) {
                if (tr.id != my_id && tr.misses < 5) {
                    goal_x = tr.x;
                    goal_y = tr.y;
                    has_goal = true;
                    break;
                }
            }
        } else {
            goal_x = CAM_W - my_robot->x;
            goal_y = CAM_H - my_robot->y;
            has_goal = true;
        }
        
        // Path planning
        double astar_ms = 0.0;
        replan_counter++;
        if (has_goal && (!path_planned || replan_counter > REPLAN_INTERVAL)) {
            replan_counter = 0;
            
            int start_gx = (int)(my_robot->x / cell_px);
            int start_gy = (int)(my_robot->y / cell_px);
            int goal_gx = (int)(goal_x / cell_px);
            int goal_gy = (int)(goal_y / cell_px);

            double t_astar0 = high_resolution_time();
            
            auto path_opt = planner.plan(pipeline_res.occ_grid, 
                                         {start_gx, start_gy}, 
                                         {goal_gx, goal_gy});
            astar_ms = (high_resolution_time() - t_astar0) * 1000.0;
            
            if (path_opt.has_value()) {
                path_pixels.clear();
                auto& path_grid = path_opt.value();
                for (size_t i = 0; i < path_grid.size(); ++i) {
                    int gx = path_grid[i].first;
                    int gy = path_grid[i].second;
                    path_pixels.push_back({(double)(gx * cell_px), (double)(gy * cell_px)});
                }
                
                current_waypoint_idx = 0;
                path_planned = true;
                cout << " Path planned with " << path_pixels.size() << " waypoints" << endl;
            } else {
                path_planned = false;
            }
        }
        
        // Control generation
        double t_control0 = high_resolution_time();
        Command cmd = {0.0, 0.0, false};
        
        // Offense mode: fire laser if aimed at enemy
        if (offense_mode && tracks.size() > 1) {
            RobotTrack* enemy = nullptr;
            for (auto& tr : tracks) {
                if (tr.id != my_id) {
                    enemy = &tr;
                    break;
                }
            }
            
            if (enemy) {
                double dx = enemy->x - my_robot->x;
                double dy = enemy->y - my_robot->y;
                double dist = std::hypot(dx, dy);
                double target_angle = std::atan2(dy, dx);
                double angle_diff = target_angle - my_robot->theta;
                while (angle_diff > M_PI) angle_diff -= 2*M_PI;
                while (angle_diff < -M_PI) angle_diff += 2*M_PI;
                
                if (dist < 350 && std::abs(angle_diff) < 0.2) {
                    cmd.laser = true;
                    cout << "FIRING!" << endl;
                }
            }
        }
        
        // Waypoint following
        if (path_planned && current_waypoint_idx < path_pixels.size()) {
            double wx = path_pixels[current_waypoint_idx].first;
            double wy = path_pixels[current_waypoint_idx].second;
            
            double dx = wx - my_robot->x;
            double dy = wy - my_robot->y;
            double dist_to_wp = std::hypot(dx, dy);
            
            // Check if reached current waypoint
            if (dist_to_wp < WAYPOINT_REACHED_DIST) {
                current_waypoint_idx++;
                if (current_waypoint_idx >= path_pixels.size()) {
                    cout << "Goal reached!" << endl;
                    goal_reached = true;
                    path_planned = false;
                }
            }
            
            // Use WaypointFollower.follow() for single waypoint control
            if (current_waypoint_idx < path_pixels.size()) {
                double current_wx = path_pixels[current_waypoint_idx].first;
                double current_wy = path_pixels[current_waypoint_idx].second;
                
                // Use fuzzy logic for tactical control
                TacticalFeatures features;
                features.enemy_dist = 9999.0;
                features.enemy_bearing_deg = 0.0;
                features.nearest_obs_dist = 9999.0;
                features.n_blocking = 0;
                features.has_cover = 0.0;
                features.close_danger = 0.0;
                features.obstacle_pressure = 0.0;
                features.surprise_desire = 0.0;
                
                // Find nearest obstacle
                for (const auto& obs : pipeline_res.obstacles) {
                    double obs_dist = std::hypot(obs.cx - my_robot->x, obs.cy - my_robot->y);
                    if (obs_dist < features.nearest_obs_dist) {
                        features.nearest_obs_dist = obs_dist;
                    }
                }
                
                FuzzyDecision decision = offense_mode ? fuzzy.offense(features) : fuzzy.defense(features);
                
                // Basic waypoint following control
                const double k_ang = 2.5;
                const double k_lin = 0.5;
                const double v_max = 80.0;
                
                cmd = follower.follow(my_robot->x, my_robot->y, my_robot->theta,
                                     {current_wx, current_wy},
                                     WAYPOINT_REACHED_DIST,
                                     k_ang * decision.lookahead_scale,
                                     k_lin * decision.speed_scale,
                                     v_max * decision.speed_scale);
            }
        }

        double control_ms = (high_resolution_time() - t_control0) * 1000.0;

        // Runtime diagnostics sample
        double enemy_x = std::numeric_limits<double>::quiet_NaN();
        double enemy_y = std::numeric_limits<double>::quiet_NaN();
        double enemy_dist_px = std::numeric_limits<double>::quiet_NaN();
        double enemy_bearing_deg = std::numeric_limits<double>::quiet_NaN();
        for (auto& tr : tracks) {
            if (tr.id != my_id && tr.misses < 5) {
                enemy_x = tr.x;
                enemy_y = tr.y;
                double dx_e = enemy_x - my_robot->x;
                double dy_e = enemy_y - my_robot->y;
                enemy_dist_px = std::hypot(dx_e, dy_e);
                double desired_e = std::atan2(dy_e, dx_e);
                double err_e = desired_e - my_robot->theta;
                while (err_e > M_PI) err_e -= 2 * M_PI;
                while (err_e < -M_PI) err_e += 2 * M_PI;
                enemy_bearing_deg = err_e * 180.0 / M_PI;
                break;
            }
        }

        double nearest_obs_dist_px = std::numeric_limits<double>::quiet_NaN();
        for (const auto& obs : pipeline_res.obstacles) {
            double d_obs = std::hypot(obs.cx - my_robot->x, obs.cy - my_robot->y);
            if (!(nearest_obs_dist_px == nearest_obs_dist_px) || d_obs < nearest_obs_dist_px) {
                nearest_obs_dist_px = d_obs;
            }
        }

        double total_ms = (high_resolution_time() - loop_abs0) * 1000.0;

        if (diag.is_open()) {
            diag << tc << ',' << frame_count << ',' << (offense_mode ? 1 : 0) << ','
                 << cmd.left << ',' << cmd.right << ',' << (cmd.laser ? 1 : 0) << ','
                 << my_robot->x << ',' << my_robot->y << ','
                 << enemy_x << ',' << enemy_y << ',' << enemy_dist_px << ',' << enemy_bearing_deg << ','
                 << nearest_obs_dist_px << ',' << pipeline_res.obstacles.size() << ','
                 << path_pixels.size() << ',' << (has_goal ? 1 : 0) << ',' << (path_planned ? 1 : 0) << ','
                 << detect_ms << ',' << obstacles_ms << ',' << astar_ms << ',' << control_ms << ',' << total_ms << ',' << fps << '\n';
        }
        
        sendCommand(hSerial, cmd);
        
        // Visualization
        for (auto& tr : tracks) {
            int R = (tr.id == my_robot->id) ? 0 : 255;
            int G = (tr.id == my_robot->id) ? 255 : 0;
            int B = 0;
            
            draw_circle_rgb(rgb, (int)tr.x, (int)tr.y, 15, R, G, B);
            draw_arrow_rgb(rgb, (int)tr.x, (int)tr.y, tr.theta, 30, R, G, B);
            
            char buf[64];
            sprintf_s(buf, (tr.id == my_robot->id) ? "YOU(ID:%d)" : "TARGET(ID:%d)", tr.id);
            draw_text_rgb(rgb, (int)tr.x + 20, (int)tr.y - 20, buf, R, G, B);
        }
        
        // Draw current waypoint
        if (path_planned && current_waypoint_idx < path_pixels.size()) {
            double wx = path_pixels[current_waypoint_idx].first;
            double wy = path_pixels[current_waypoint_idx].second;
            draw_circle_rgb(rgb, (int)wx, (int)wy, 15, 0, 255, 255);
            draw_line_rgb(rgb, (int)my_robot->x, (int)my_robot->y, (int)wx, (int)wy, 0, 255, 255);
        }
        
        if (has_goal) {
            draw_circle_rgb(rgb, (int)goal_x, (int)goal_y, 25, 255, 0, 255);
            draw_text_rgb(rgb, (int)goal_x + 30, (int)goal_y - 10, "GOAL", 255, 0, 255);
        }
        
        for (const auto& obs : pipeline_res.obstacles) {
            draw_obstacle_overlay(rgb, obs, 255, 128, 0);
        }
        
        for (const auto& d : dets) {
            draw_robot_mask_overlay(rgb, d, robot_length_px, robot_width_px, 255, 255, 100);
        }
        
        draw_text_rgb(rgb, 10, 10, offense_mode ? "OFFENSE" : "NAVIGATE", 255, 255, 0);
        draw_text_rgb(rgb, 10, 20, goal_reached ? "COMPLETE" : "RUNNING", 0, 255, 0);
        
        char time_buf[32];
        sprintf_s(time_buf, "T:%.1fs", tc);
        draw_text_rgb(rgb, 10, 30, time_buf, 255, 255, 255);
        
        view_rgb_image(rgb);
        frame_count++;
    }
    
    // ============================================================
    // Cleanup
    // ============================================================
    cout << "\nShutting down..." << endl;
    sendCommand(hSerial, {0.0, 0.0, false});
    
    if (hSerial != INVALID_HANDLE_VALUE) CloseHandle(hSerial);
    if (diag.is_open()) {
        diag.close();
        cout << "Diagnostics saved to " << DIAG_CSV_PATH << endl;
        cout << "Generate plots with: python plot_diagnostics.py " << DIAG_CSV_PATH << endl;
    }
    free_image(rgb);
    
    cout << "Complete. Processed " << frame_count << " frames." << endl;
    return 0;
}
