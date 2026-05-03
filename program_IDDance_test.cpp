// Test program for IDDance using vision_simulator
// Based on professor's simulation example
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Windows.h>

using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "IDDance.h"
#include "Types.h"
#include "Overlay.h"  // ⭐ Use your existing drawing functions!

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

/*int main()
{
    double x0, y0, theta0, max_speed, opponent_max_speed;
    int pw_l, pw_r, pw_laser, laser;
    double width1, height1;
    int N_obs, n_robot;
    double x_obs[50], y_obs[50], size_obs[50];
    double D, Lx, Ly, Ax, Ay, alpha_max;
    double tc, tc0; // clock time
    int mode, level;

    cout << "\n=== ID Dance Simulation Test (TWO ROBOTS) ===" << endl;
    cout << "This will test the robot performing an ID dance." << endl;
    cout << "Robot 1 will spin, Robot 2 will stay still." << endl;
    cout << "\nPress SPACE to begin..." << endl;
    pause();

    // ============ Setup Simulation Parameters ============

    // Image size (vision simulation assumes 640x480)
    width1 = 640;
    height1 = 480;

    // No obstacles for this test
    N_obs = 0;

    // Robot model parameters (pixels)
    D = 121.0;         // distance between wheels
    Lx = 31.0;         // laser x position
    Ly = 0.0;          // laser y position
    Ax = 37.0;         // axis of rotation x
    Ay = 0.0;          // axis of rotation y
    alpha_max = 3.14159 / 2;  // max laser angle (rad)

    // ✅ TWO ROBOTS for this test
    n_robot = 2;

    // ============ Activate Libraries ============

    // Activate regular vision library FIRST
    activate_vision();

    // Fixed: Correct activate_simulation signature
    char obstacle_files[N_MAX][S_MAX];
    for (int i = 0; i < N_MAX; i++)
        strncpy_s(obstacle_files[i], "obstacle.bmp", S_MAX);

    int err = activate_simulation(
        width1, height1,
        x_obs, y_obs, N_obs,
        "robot_A.bmp", "robot_B.bmp", "background.bmp",
        obstacle_files,
        D, Lx, Ly, Ax, Ay, alpha_max, n_robot
    );

    if (err != 0) {
        cout << "\nERROR: Failed to activate simulation!" << endl;
        cout << "Make sure these files exist in your project directory:" << endl;
        cout << "  - robot_BR.bmp" << endl;
        cout << "  - background.bmp" << endl;
        cout << "  - obstacle.bmp" << endl;
        pause();
        return 1;
    }

    cout << "\nSimulation activated successfully!" << endl;

    // Set simulation mode
    mode = 0;
    set_simulation_mode(mode);

    // ============ Set Initial Robot Positions ============

    // Robot 1 (YOUR robot) - Left side
    x0 = 200;
    y0 = 240;
    theta0 = 0.0;
    set_robot_position(x0, y0, theta0);
    cout << "Robot 1 (YOU) positioned at (" << x0 << ", " << y0 << "), angle = " << theta0 << endl;

    // Robot 2 (OPPONENT) - Right side, well separated
    double x_opponent = 480;
    double y_opponent = 240;
    double theta_opponent = M_PI;
    set_opponent_position(x_opponent, y_opponent, theta_opponent);
    cout << "Robot 2 (OPPONENT) positioned at (" << x_opponent << ", " << y_opponent << "), angle = " << theta_opponent << endl;

    cout << "\nRobots separated by " << (x_opponent - x0) << " pixels" << endl;

    // ============ Initialize Vision Components ============

    image rgb;
    rgb.type = RGB_IMAGE;
    rgb.width = 640;
    rgb.height = 480;
    allocate_image(rgb);

    MarkerDetector detector;
    Tracker tracker;
    IDDance id_dance;

    // ============ Initial Parameters ============

    pw_l = 1500;
    pw_r = 1500;
    pw_laser = 1500;
    laser = 0;

    max_speed = 100;
    opponent_max_speed = 100;

    set_inputs(pw_l, pw_r, pw_laser, laser, max_speed);
    set_opponent_inputs(1500, 1500, 1500, 0, opponent_max_speed);

    // ============ Main Loop ============

    tc0 = high_resolution_time();
    int frame_count = 0;
    bool dance_complete = false;
    int my_id = -1;

    std::vector<RobotTrack> tracks;

    cout << "\n=== Starting ID Dance ===" << endl;
    cout << "Press 'X' to exit early" << endl << endl;

    while (!dance_complete) {

        if (KEY('X')) {
            cout << "\nAborted by user." << endl;
            break;
        }

        tc = high_resolution_time() - tc0;

        // ============ Acquire Simulated Image ============
        acquire_image_sim(rgb);

        // ============ Detect Markers ============
        std::vector<Blob> front_blobs, rear_blobs;

        // ⭐ Robot A: Green front, Red rear
        // ⭐ Robot B: Orange front, Blue rear
        detector.detect_two_profiles(
            rgb,
            ColorProfile::GR,     // Robot A (left): Green front, Red rear
            ColorProfile::OB,     // Robot B (right): Orange front, Blue rear
            front_blobs,          // Combined: Green + Orange
            rear_blobs            // Combined: Red + Blue
        );
        
        // ⭐ PAIR markers into robot detections (this was missing!)
        std::optional<double> expected_sep;
        auto detections = tracker.pairMarkers(front_blobs, rear_blobs, expected_sep, 0.55, 1200.0);

        // Update tracks with paired detections
        tracks = tracker.updateTracks(tracks, detections, tc, 80.0, 10);

        // ============ 🎨 DEBUG VISUALIZATION (Using Overlay.h) ============
        
        // Draw detected FRONT markers (Green + Orange → show as CYAN for visibility)
        for (const auto& blob : front_blobs) {
            draw_circle_rgb(rgb, (int)blob.x, (int)blob.y, 10, 0, 255, 255);      // Cyan outline
            draw_circle_rgb(rgb, (int)blob.x, (int)blob.y, 8, 100, 255, 255);     // Lighter cyan
        }
        
        // Draw detected REAR markers (Red + Blue → show as MAGENTA for visibility)
        for (const auto& blob : rear_blobs) {
            draw_circle_rgb(rgb, (int)blob.x, (int)blob.y, 10, 255, 0, 255);      // Magenta outline
            draw_circle_rgb(rgb, (int)blob.x, (int)blob.y, 8, 255, 100, 255);     // Lighter magenta
        }
        
        // Draw robot tracks with IDs
        for (const auto& track : tracks) {
            int cx = (int)track.x;
            int cy = (int)track.y;
            
            // Draw centroid as yellow cross
            draw_line_rgb(rgb, cx - 15, cy, cx + 15, cy, 255, 255, 0);  // Horizontal
            draw_line_rgb(rgb, cx, cy - 15, cx, cy + 15, 255, 255, 0);  // Vertical
            
            // Draw heading arrow (green) using your arrow function!
            draw_arrow_rgb(rgb, cx, cy, track.theta, 40, 0, 255, 0);
            
            // Draw ID label using your text function!
            draw_id_label(rgb, cx - 30, cy - 30, track.id);
        }

        // ============ Run ID Dance ============
        my_id = id_dance.run(tc, rgb, detector, tracker);

        Command cmd = id_dance.currentCommand();

        pw_l = vel_to_pw(cmd.left);
        pw_r = vel_to_pw(cmd.right);
        laser = cmd.laser ? 1 : 0;

        set_inputs(pw_l, pw_r, pw_laser, laser, max_speed);

        // ============ 📊 DEBUG OUTPUT ============

        if (frame_count % 20 == 0) {
            cout << "\n--- Frame " << frame_count << " (t=" << tc << "s) ---" << endl;
            cout << "Detected: " << front_blobs.size() << " front, " 
                 << rear_blobs.size() << " rear" << endl;
            cout << "Paired: " << detections.size() << " robots" << endl;
            cout << "Tracks: " << tracks.size() << " active" << endl;
            
            for (const auto& track : tracks) {
                cout << "  Track[" << track.id << "]: "
                     << "pos=(" << (int)track.x << "," << (int)track.y << ") "
                     << "theta=" << (int)(track.theta * 180.0 / M_PI) << "° "
                     << "sep=" << (int)track.sep_px << "px" << endl;
            }
            
            cout << "Command: L=" << cmd.left << " R=" << cmd.right 
                 << " | PWM: L=" << pw_l << " R=" << pw_r << endl;
            
            double wheel_base = D;
            double v_left = cmd.left * max_speed;
            double v_right = cmd.right * max_speed;
            double omega = (v_right - v_left) / wheel_base;
            cout << "Angular vel: " << omega << " rad/s = " 
                 << (omega * 180.0 / M_PI) << " deg/s" << endl;
        }

        if (frame_count % 30 == 0) {
            cout << "\nTime: " << tc << "s | Frame: " << frame_count << " | ";
            if (id_dance.done()) {
                cout << "ID: " << my_id << " (COMPLETE)";
            } else {
                cout << "Status: DANCING";
            }
            cout << endl;
        }

        if (id_dance.done() && !dance_complete) {
            dance_complete = true;
            cout << "\n=== ID DANCE COMPLETE ===" << endl;
            cout << "My assigned ID: " << my_id << endl;
            cout << "Total time: " << tc << " seconds" << endl;
            cout << "Total frames: " << frame_count << endl;
        }

        view_rgb_image(rgb);
        frame_count++;

        if (tc > 10.0 && !dance_complete) {
            cout << "\n=== TIMEOUT ===" << endl;
            cout << "Dance did not complete within 10 seconds." << endl;
            break;
        }

        Sleep(10);
    }

    // ============ Cleanup ============

    free_image(rgb);
    deactivate_vision();
    deactivate_simulation();

    cout << "\n=== Test Complete ===" << endl;
    cout << "Press any key to exit..." << endl;
    pause();

    return 0;
}*/
