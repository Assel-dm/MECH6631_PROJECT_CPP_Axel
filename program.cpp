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

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

// Convert Command velocity (-1 to 1) to pulse width (1000-2000 us)
int vel_to_pw(double vel) {
    int pw_center = 1500;
    int pw_range = 500;
    // Clamp velocity to [-1, 1]
    if (vel < -1.0) vel = -1.0;
    if (vel > 1.0) vel = 1.0;
    return pw_center + (int)(vel * pw_range);
}

int main()
{
    double x0, y0, theta0, max_speed, opponent_max_speed;
    int pw_l, pw_r, pw_laser, laser;
    double light, light_gradient, light_dir, image_noise;
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

    // Then activate simulation
    // NOTE: Make sure these BMP files exist in your project directory!
    int err = activate_simulation(
        width1, height1,
        x_obs, y_obs, size_obs, N_obs,
        "robot_BR.bmp", "robot_BR.bmp", "background.bmp", "obstacle.bmp",
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

    // Set simulation mode (single player, manual opponent)
    mode = 0;
    level = 1;
    set_simulation_mode(mode, level);

    // ============ Set Initial Robot Positions ============

    // Robot 1 (YOUR robot) - Left side
    x0 = 200;      // ✅ Left of center (640/2 = 320)
    y0 = 240;      // Center vertically
    theta0 = 0.0;  // Facing right
    set_robot_position(x0, y0, theta0);
    cout << "Robot 1 (YOU) positioned at (" << x0 << ", " << y0 << "), angle = " << theta0 << endl;

    // Robot 2 (OPPONENT) - Right side, well separated
    double x_opponent = 480;  // ✅ Right of center (280 pixels away)
    double y_opponent = 240;  // Center vertically
    double theta_opponent = M_PI;  // ✅ Facing left (toward Robot 1)
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

    // Start stationary
    pw_l = 1500;
    pw_r = 1500;
    pw_laser = 1500;
    laser = 0;

    max_speed = 100;              // pixels/s
    opponent_max_speed = 100;

    light = 1.0;
    light_gradient = 0.0;
    light_dir = 0.0;
    image_noise = 0.0;

    set_inputs(pw_l, pw_r, pw_laser, laser,
        light, light_gradient, light_dir, image_noise,
        max_speed, opponent_max_speed);

    // Keep opponent stationary during dance
    set_opponent_inputs(1500, 1500, 1500, 0, opponent_max_speed);

    // ============ Main Loop ============

    tc0 = high_resolution_time();
    int frame_count = 0;
    bool dance_complete = false;
    int my_id = -1;

    cout << "\n=== Starting ID Dance ===" << endl;
    cout << "Press 'X' to exit early" << endl << endl;

    while (!dance_complete) {

        // Check for exit key
        if (KEY('X')) {
            cout << "\nAborted by user." << endl;
            break;
        }

        // Get current time
        tc = high_resolution_time() - tc0;

        // ============ Acquire Simulated Image ============
        acquire_image_sim(rgb);

        // ============ Run ID Dance ============
        my_id = id_dance.run(tc, rgb, detector, tracker);

        // Get current dance command
        Command cmd = id_dance.currentCommand();

        // Convert to pulse widths
        pw_l = vel_to_pw(cmd.left);
        pw_r = vel_to_pw(cmd.right);
        laser = cmd.laser ? 1 : 0;

        // Update simulation inputs
        set_inputs(pw_l, pw_r, pw_laser, laser,
            light, light_gradient, light_dir, image_noise,
            max_speed, opponent_max_speed);

        // ============ Status Display ============

        if (frame_count % 30 == 0) {
            cout << "Time: " << tc << "s | ";
            cout << "Frame: " << frame_count << " | ";

            if (id_dance.done()) {
                cout << "ID: " << my_id << " (COMPLETE)";
            }
            else {
                cout << "Status: DANCING";
            }
            cout << endl;
        }

        // Check if dance is complete
        if (id_dance.done() && !dance_complete) {
            dance_complete = true;
            cout << "\n=== ID DANCE COMPLETE ===" << endl;
            cout << "My assigned ID: " << my_id << endl;
            cout << "Total time: " << tc << " seconds" << endl;
            cout << "Total frames: " << frame_count << endl;
        }

        // Display the simulated camera view
        view_rgb_image(rgb);

        frame_count++;

        // Safety timeout (10 seconds)
        if (tc > 10.0 && !dance_complete) {
            cout << "\n=== TIMEOUT ===" << endl;
            cout << "Dance did not complete within 10 seconds." << endl;
            cout << "This may indicate detection issues." << endl;
            cout << "Check that:" << endl;
            cout << "  - Robot markers are visible in the image" << endl;
            cout << "  - MarkerDetector settings match your marker colors" << endl;
            break;
        }

        // Frame rate control (don't simulate too fast)
        Sleep(10); // ~100 fps max
    }

    // ============ Cleanup ============

    free_image(rgb);
    deactivate_vision();
    deactivate_simulation();

    cout << "\n=== Test Complete ===" << endl;
    cout << "Press any key to exit..." << endl;
    pause();

    return 0;
}