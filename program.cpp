// TestArena.cpp - Comprehensive testing environment for robot strategies
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <memory>
#include <cmath>

using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "IDDance.h"
#include "OpponentBehaviour.h"
#include "Types.h"
#include "DebugVisualizer.h"
#include "ArenaSetup.h"

// Strategy files
#include "Offense.h"
#include "Defense.h"
#include "AStar.h"
#include "OccupancyGrid.h"
#include "Waypoint.h"
#include "Fuzzy.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )


int main()
{
    SetConsoleOutputCP(CP_UTF8);
    // ============ Configuration ============
    
    // SELECT TEST MODE HERE:
    TestMode test_mode = TEST_OFFENSE;  // ⭐ CHANGE THIS TO TEST DIFFERENT MODES
    
    // Test duration (seconds)
    double test_duration = 30.0;
    
    // ============ Print Test Info ============
    
    cout << "\n========================================" << endl;
    cout << "    ROBOT STRATEGY TEST ARENA" << endl;
    cout << "========================================" << endl;
    
    switch (test_mode) {
        case TEST_ID_DANCE:
            cout << "Mode: ID DANCE TEST" << endl;
            cout << "Your robot will perform ID dance" << endl;
            cout << "Opponent will move randomly" << endl;
            test_duration = 15.0;
            break;
        case TEST_OFFENSE:
            cout << "Mode: OFFENSE TEST" << endl;
            cout << "Your robot will attack" << endl;
            cout << "Opponent will defend its zone" << endl;
            break;
        case TEST_DEFENSE:
            cout << "Mode: DEFENSE TEST" << endl;
            cout << "Your robot will defend" << endl;
            cout << "Opponent will attack" << endl;
            break;
        case TEST_NAVIGATION:
            cout << "Mode: NAVIGATION TEST" << endl;
            cout << "Your robot will navigate obstacles" << endl;
            cout << "Opponent will patrol waypoints" << endl;
            break;
        case TEST_FULL_GAME:
            cout << "Mode: FULL GAME SIMULATION" << endl;
            cout << "Complete strategy test" << endl;
            break;
    }
    
    cout << "\nPress SPACE to begin..." << endl;
    pause();
    
    // ============ Setup Variables ============
    
    double x0, y0, theta0, max_speed, opponent_max_speed;
    int pw_l, pw_r, pw_laser, laser;
    double light, light_gradient, light_dir, image_noise;
    double width1, height1;
    int N_obs, n_robot;
    double x_obs[50], y_obs[50], size_obs[50];
    double D, Lx, Ly, Ax, Ay, alpha_max;
    double tc, tc0;
    int mode, level;
    
    // Image size
    const double ARENA_SCALE = 2.0;  // Change this to scale entire arena
    width1 = 1280;
    height1 = 960;
    
    // Setup obstacles based on test mode
    setupObstacles(test_mode, N_obs, x_obs, y_obs, size_obs, ARENA_SCALE);  // ← ADD ARENA_SCALE
    
    // Add boundary walls to keep robots in arena
    addBoundaryObstacles(N_obs, x_obs, y_obs, size_obs, width1, height1, 60.0);  // 60px margin from edges

    cout << "\nArena setup: " << N_obs << " obstacles" << endl;
    
    // Robot model parameters
    D = 121.0;
    Lx = 31.0;
    Ly = 0.0;
    Ax = 37.0;
    Ay = 0.0;
    alpha_max = M_PI / 2;
    
    // Two robots
    n_robot = 2;
    
    // ============ Activate Libraries ============
    
    activate_vision();
    
    int err = activate_simulation(
        width1, height1,
        x_obs, y_obs, size_obs, N_obs,
        "robot_BR.bmp", "robot_BR.bmp", "background.bmp", "obstacle.bmp",
        D, Lx, Ly, Ax, Ay, alpha_max, n_robot
    );
    
    if (err != 0) {
        cout << "\nERROR: Failed to activate simulation!" << endl;
        pause();
        return 1;
    }
    
    cout << "Simulation activated successfully!" << endl;
    
    mode = 0;
    level = 1;
    set_simulation_mode(mode, level);
    
    // ============ Set Initial Robot Positions ============
    
    if (test_mode == TEST_ID_DANCE) {
        // ID Dance positions (separated)
        x0 = 200 * ARENA_SCALE;
        y0 = 240 * ARENA_SCALE;
        theta0 = 0.0;
        
        double x_opp = 480 * ARENA_SCALE;
        double y_opp = 240 * ARENA_SCALE    ;
        double theta_opp = M_PI;
        
        set_robot_position(x0, y0, theta0);
        set_opponent_position(x_opp, y_opp, theta_opp);
        
    } else if (test_mode == TEST_OFFENSE) {
        // Offense test: you start left, opponent defends right
        x0 = 150 * ARENA_SCALE;
        y0 = 240 * ARENA_SCALE;
        theta0 = 0.0;
        
        double x_opp = 500 * ARENA_SCALE;
        double y_opp = 240 * ARENA_SCALE;
        double theta_opp = M_PI;
        
        set_robot_position(x0, y0, theta0);
        set_opponent_position(x_opp, y_opp, theta_opp);
        
    } else if (test_mode == TEST_DEFENSE) {
        // Defense test: you defend left, opponent attacks from right
        x0 = 150 * ARENA_SCALE;
        y0 = 240 * ARENA_SCALE;
        theta0 = 0.0;
        
        double x_opp = 500 * ARENA_SCALE;
        double y_opp = 240 * ARENA_SCALE;
        double theta_opp = M_PI;
        
        set_robot_position(x0, y0, theta0);
        set_opponent_position(x_opp, y_opp, theta_opp);
        
    } else {
        // Navigation/Full game: start in opposite corners
        x0 = 210 * ARENA_SCALE;
        y0 = 90 * ARENA_SCALE;
        theta0 = M_PI / 4;
        
        double x_opp = 420 * ARENA_SCALE;
        double y_opp = 380 * ARENA_SCALE;
        double theta_opp = -3 * M_PI / 4;
        
        set_robot_position(x0, y0, theta0);
        set_opponent_position(x_opp, y_opp, theta_opp);
    }
    
    cout << "Robot 0 (YOU) at (" << x0 << ", " << y0 << ")" << endl;
    
    // ============ Initialize Vision & Strategies ============
    
    image rgb;
    rgb.type = RGB_IMAGE;
    rgb.width = 1280;
    rgb.height = 960;
    allocate_image(rgb);
    
    MarkerDetector detector;
    Tracker tracker;
    
    // Initialize ID dance
    IDDance id_dance;
    
    // ============ Initialize REAL Strategy Components ============
    
    // Create pathfinding and control components
    AStarPlanner planner;
    WaypointFollower follower;
    FuzzyLogic fuzzy;
    OccupancyGrid occupancy_grid;
    
    // Create strategies with dependencies
    OffenseStrategy offense_strategy(&planner, &follower, &fuzzy);
    DefenseStrategy defense_strategy(&planner, &follower, &fuzzy);
    
    cout << "Strategies initialized with A*, Waypoint Follower, and Fuzzy Logic" << endl;
    
    // Create opponent behavior
    std::unique_ptr<OpponentBehavior> opponent;
    
    switch (test_mode) {
        case TEST_ID_DANCE:
            opponent = std::make_unique<RandomMovement>();
            break;
        case TEST_OFFENSE:
            opponent = std::make_unique<ScriptedDefender>(500 * ARENA_SCALE, 240 * ARENA_SCALE, 100);
            break;
        case TEST_DEFENSE:
            opponent = std::make_unique<ScriptedAttacker>(150 * ARENA_SCALE, 240 * ARENA_SCALE);
            break;
        case TEST_NAVIGATION:
            opponent = std::make_unique<Patroller>(
                std::vector<std::pair<double, double>>{
                    {150 * ARENA_SCALE, 150 * ARENA_SCALE}, 
                    {490 * ARENA_SCALE, 150 * ARENA_SCALE}, 
                    {490 * ARENA_SCALE, 330 * ARENA_SCALE}, 
                    {150 * ARENA_SCALE, 330 * ARENA_SCALE}
                });
            break;
        case TEST_FULL_GAME:
            opponent = std::make_unique<RandomMovement>();
            break;
    }
    
    // ============ Initial Parameters ============
    
    pw_l = 1500;
    pw_r = 1500;
    pw_laser = 1500;
    laser = 0;
    
    max_speed = 100;
    opponent_max_speed = 100;
    
    light = 1.0;
    light_gradient = 0.0;
    light_dir = 0.0;
    image_noise = 0.0;
    
    set_inputs(pw_l, pw_r, pw_laser, laser,
        light, light_gradient, light_dir, image_noise,
        max_speed, opponent_max_speed);
    
    set_opponent_inputs(1500, 1500, 1500, 0, opponent_max_speed);
    
    // ============ Collision Detection Configuration ============
    const double ROBOT_COLLISION_WIDTH = 90.0;   // pixels (perpendicular to heading)
    const double ROBOT_COLLISION_LENGTH = 140.0;  // pixels (along heading direction)
    // ===========================================================

    // ============ Strategy Parameters ============
    
    const int CELL_PX = 20;              // Grid cell size in pixels
    const double ROBOT_RADIUS = ROBOT_COLLISION_WIDTH / 2.0;  // 45px for 90px width
    const int INFLATE_PX = static_cast<int>(ROBOT_RADIUS * 1.5);  // was 1.0 — increase clearance
 //   const double LASER_CLOSE_PX = 50.0;  // Distance to fire laser
 //   const double LASER_ALIGN_DEG = 15.0; // Angle tolerance for laser
    const double V_MAX = 1.0;            // Max velocity
    const int LOOKAHEAD_CELLS = 1;        // Defense hiding point samples
    const int HIDING_SAMPLES = 8;        // Defense hiding point samples

    // ============ Laser Parameters ============
    const double LASER_MAX_RANGE_PX    = 900.0;  // Max range to attempt shot (px)
    const int    LASER_LOS_FRAMES_REQ  = 6;   // ⭐ ~0.75s at 8fps — tune this
                                       // your actual fps ≈ frame_count / tc
    
    // ============ Main Loop ============
    
    tc0 = high_resolution_time();
    int frame_count = 0;
    bool test_complete = false;
    int my_id = -1;
    
    // Tracking state
    std::vector<RobotTrack> tracks;
    double my_x = x0, my_y = 0, my_theta = theta0;
    double opp_x = 0, opp_y = 0, opp_theta = 0;
    
    // Visualization flags
    static bool show_collision_box = true;
    static bool show_grid = false;
    
    cout << "\n=== TEST STARTED ===" << endl;
    cout << "Press 'X' to exit early" << endl << endl;
    
    while (!test_complete) {
        
        if (KEY('X')) {
            cout << "\nTest aborted by user." << endl;
            break;
        }
        
        tc = high_resolution_time() - tc0;
        
        // ============ Acquire Image & Detect ============
        acquire_image_sim(rgb);
        
        std::vector<Blob> front, rear;
        detector.detect_markers(rgb, front, rear);
        
        std::optional<double> expected_sep;
        auto dets = tracker.pairMarkers(front, rear, expected_sep, 0.55, 1200.0);
        
        tracks = tracker.updateTracks(tracks, dets, tc, 80.0, 10);
        
        // ============ Process Based on Test Mode ============
        
        Command my_cmd{ 0.0, 0.0, false };
        bool freeze_on_laser = false;
        
        if (test_mode == TEST_ID_DANCE) {
            // Run ID dance
            my_id = id_dance.run(tc, rgb, detector, tracker);
            my_cmd = id_dance.currentCommand();
            
            if (id_dance.done() && !test_complete) {
                test_complete = true;
                cout << "\n=== ID DANCE COMPLETE ===" << endl;
                cout << "Identified as Robot ID: " << my_id << endl;
            }
            
        } else if (test_mode == TEST_OFFENSE) {
            // ============ Run REAL Offense Strategy with Collision ============
            
            if (my_id < 0) {
                my_id = id_dance.run(tc, rgb, detector, tracker);
                my_cmd = id_dance.currentCommand();
                
                if (id_dance.done()) {
                    cout << "ID identified as: " << my_id << " - Starting offense with collision detection!" << endl;
                }

            } else {
                // Build obstacle list
                std::vector<Obstacle> obstacle_list;
                for (int i = 1; i <= N_obs; i++) {
                    int w = (int)size_obs[i];
                    int h = (int)size_obs[i];
                    if (i > N_obs - 4) {
                        if (i == N_obs - 3 || i == N_obs - 2) h = 40;
                        else                                   w = 40;
                    }
                    obstacle_list.push_back({
                        static_cast<int>(x_obs[i] - w / 2.0),
                        static_cast<int>(y_obs[i] - h / 2.0),
                        w, h,
                        x_obs[i], y_obs[i],
                        (double)(w * h)
                    });
                }

                Grid grid_visual   = occupancy_grid.build(obstacle_list, (int)width1, (int)height1, CELL_PX, 0);
                Grid grid_planning = occupancy_grid.build(obstacle_list, (int)width1, (int)height1, CELL_PX, INFLATE_PX);

                double my_x, my_y, my_theta;
                if (getTrackedRobotState(tracks, my_id, my_x, my_y, my_theta)) {

                    // ⭐ Compute front-axle position first — used for planning AND control
                    double wheel_x, wheel_y;
                    getWheelCenterPosition(my_x, my_y, my_theta, wheel_x, wheel_y, Lx);

                    // Find opponent position — including heading and marker separation
                    double opp_x = 500 * ARENA_SCALE, opp_y = 240 * ARENA_SCALE;
                    double opp_theta_track = 0.0;
                    double opp_sep = 0.0;
                    for (const auto& t : tracks) {
                        if (t.id != my_id) {
                            opp_x     = t.x;
                            opp_y     = t.y;
                            opp_theta_track = t.theta;
                            opp_sep   = t.sep_px;
                            break;
                        }
                    }

                    // ⭐ Aim at the opponent's RED (rear) marker — half sep behind centroid
                    double red_x = opp_x - (opp_sep / 2.0) * cos(opp_theta_track);
                    double red_y = opp_y - (opp_sep / 2.0) * sin(opp_theta_track);

                    // Snap start cell to nearest free cell if inside inflated zone
                    auto snap_to_free = [&](const Grid& grid, std::pair<int, int> cell) -> std::pair<int, int> {
                        if (!grid[cell.first][cell.second]) return cell;
                        for (int r = 1; r < 6; r++) {
                            for (int dy = -r; dy <= r; dy++) {
                                for (int dx = -r; dx <= r; dx++) {
                                    int ny = cell.first + dy;
                                    int nx = cell.second + dx;
                                    if (ny >= 0 && ny < (int)grid.size() &&
                                        nx >= 0 && nx < (int)grid[0].size() &&
                                        !grid[ny][nx]) {
                                        return { ny, nx };
                                    }
                                }
                            }
                        }
                        return cell;
                    };

                    // ⭐ Plan from front-axle position, not centroid
                    auto raw_start = std::make_pair((int)(wheel_y / CELL_PX), (int)(wheel_x / CELL_PX));
                    auto start_cell = snap_to_free(grid_planning, raw_start);
                    auto raw_goal = std::make_pair((int)(red_y / CELL_PX), (int)(red_x / CELL_PX));
                    auto goal_cell = snap_to_free(grid_planning, raw_goal);

                    auto path = planner.plan(grid_planning, start_cell, goal_cell);

                    // Visualization
                    DebugVisualizer::drawOccupancyGrid(rgb, grid_visual, CELL_PX, 60);
                    if (path.has_value())
                        DebugVisualizer::drawPath(rgb, *path, CELL_PX, 0, 255, 255);

                    DebugVisualizer::drawRobotCollisionBox(rgb, my_x, my_y, my_theta,
                        ROBOT_COLLISION_WIDTH, ROBOT_COLLISION_LENGTH,
                        &grid_visual, CELL_PX, 0, 255, 255, 255, 0, 255);
                    DebugVisualizer::drawRobot(rgb, my_x, my_y, my_theta, 15, 255, 255, 0);
                    // ⭐ Show front axle as orange dot (same as navigation mode)
                    DebugVisualizer::drawTarget(rgb, wheel_x, wheel_y, 15, 255, 128, 0);
                    DebugVisualizer::drawTarget(rgb, opp_x, opp_y, 20, 255, 0, 0);
                    DebugVisualizer::drawTarget(rgb, red_x, red_y, 20, 255, 0, 0);

                    // Follow path
                    if (path.has_value() && path->size() > 1) {
                        // Heading error from front-axle to next waypoint
                        double dx = path->at(1).second * CELL_PX + CELL_PX/2.0 - wheel_x;
                        double dy = path->at(1).first  * CELL_PX + CELL_PX/2.0 - wheel_y;
                        double heading_error = fmod(atan2(dy, dx) - my_theta + M_PI, 2*M_PI) - M_PI;

                        int lookahead_idx;
                        if      (std::fabs(heading_error) > M_PI / 2.5) lookahead_idx = std::min(3, (int)path->size() - 1);
                        else if (std::fabs(heading_error) > M_PI / 4)   lookahead_idx = std::min(5, (int)path->size() - 1);
                        else                                             lookahead_idx = std::min(8, (int)path->size() - 1);

                        auto [wy, wx] = path->at(lookahead_idx);
                        double waypoint_x = wx * CELL_PX + CELL_PX / 2.0;
                        double waypoint_y = wy * CELL_PX + CELL_PX / 2.0;

                        DebugVisualizer::drawTarget(rgb, waypoint_x, waypoint_y, 25, 255, 255, 255);

                        // Recalculate heading error for selected waypoint
                        dx = waypoint_x - wheel_x;
                        dy = waypoint_y - wheel_y;
                        heading_error = fmod(atan2(dy, dx) - my_theta + M_PI, 2*M_PI) - M_PI;

                        // ⭐ follower.follow uses front-axle position
                        Command cmd = follower.follow(
                            wheel_x, wheel_y, my_theta,
                            std::make_pair(waypoint_x, waypoint_y),
                            5.0, 2.5, 0.6, V_MAX
                        );
                        my_cmd = cmd;

                    } else {
                        my_cmd = Command{ 0.0, 0.0, false };
                        if (frame_count % 60 == 0)
                            cout << "  NO PATH TO OPPONENT" << endl;
                    }

                    // ============ Laser: LOS-based, one shot per round ============
                    // Runs regardless of path status — fires whenever LOS is stable
                    auto hasLineOfSight = [&](double x1, double y1, double x2, double y2) -> bool {
                        int r1 = (int)(y1 / CELL_PX), c1 = (int)(x1 / CELL_PX);
                        int r2 = (int)(y2 / CELL_PX), c2 = (int)(x2 / CELL_PX);
                        int dr = std::abs(r2 - r1), dc = std::abs(c2 - c1);
                        int sr = (r1 < r2) ? 1 : -1, sc = (c1 < c2) ? 1 : -1;
                        int err = dr - dc, r = r1, c = c1;
                        int rows = (int)grid_visual.size(), cols = (int)grid_visual[0].size();
                        while (true) {
                            if (r < 0 || r >= rows || c < 0 || c >= cols) return false;
                            if (grid_visual[r][c]) return false;
                            if (r == r2 && c == c2) return true;
                            int e2 = 2 * err;
                            if (e2 > -dc) { err -= dc; r += sr; }
                            if (e2 <  dr) { err += dr; c += sc; }
                        }
                    };

                    static int  los_clear_frames = 0;
                    static bool laser_fired      = false;

                    // Laser servo — track the RED marker, not centroid
                    double bearing_to_opp = atan2(red_y - wheel_y, red_x - wheel_x);
                    double laser_rel_angle = fmod(bearing_to_opp - my_theta + M_PI, 2.0 * M_PI) - M_PI;
                    laser_rel_angle = std::max(-alpha_max, std::min(alpha_max, laser_rel_angle));
                    pw_laser = 1500 + (int)(laser_rel_angle / alpha_max * 500.0);

                    double dist_to_opp = std::hypot(wheel_x - red_x, wheel_y - red_y);
                    bool in_range  = dist_to_opp < LASER_MAX_RANGE_PX;
                    bool los_clear = in_range && hasLineOfSight(wheel_x, wheel_y, red_x, red_y);

                    if (los_clear && !laser_fired)
                        los_clear_frames++;
                    else
                        los_clear_frames = 0;

                    my_cmd.laser = (!laser_fired && los_clear_frames >= LASER_LOS_FRAMES_REQ);
                    if (my_cmd.laser) {
                        laser_fired = true;
                        freeze_on_laser = true;   // ← freeze after set_inputs
                        cout << "\n*** LASER FIRED at RED MARKER! dist=" << (int)dist_to_opp
                             << "px | stable_frames=" << los_clear_frames << " ***" << endl;

                        cout << "\n╔══════════════════════════════╗" << endl;
                        cout <<   "║      GAME OVER — YOU WIN!    ║" << endl;
                        cout <<   "╚══════════════════════════════╝\n" << endl;
                    }

                    if (frame_count % 20 == 0) {
                        cout << "  LOS: " << (los_clear ? "CLEAR" : "BLOCKED")
                             << " | stable=" << los_clear_frames << "/" << LASER_LOS_FRAMES_REQ
                             << " | fired=" << laser_fired
                             << " | dist=" << (int)dist_to_opp << endl;
                    }

                    if (frame_count % 20 == 0)
                        cout << "  SERVO: laser_angle=" << (int)(laser_rel_angle * 180.0 / M_PI)
                        << "° pw_laser=" << pw_laser << endl;
                }
            }
            
        } else if (test_mode == TEST_DEFENSE) {
            // ============ Run REAL Defense Strategy ============
            
            if (my_id < 0) {
                // Before ID is known, just spin to identify
                my_id = id_dance.run(tc, rgb, detector, tracker);
                my_cmd = id_dance.currentCommand();

                if (id_dance.done()) {
                    cout << "ID identified as: " << my_id << " - Starting defense!" << endl;
                }
            } else {
                // Build obstacle list from simulation
                std::vector<Obstacle> obstacle_list;
                for (int i = 1; i <= N_obs; i++) {
                    obstacle_list.push_back({
                        static_cast<int>(x_obs[i] - size_obs[i]/2.0),
                        static_cast<int>(y_obs[i] - size_obs[i]/2.0),
                        static_cast<int>(size_obs[i]),
                        static_cast<int>(size_obs[i]),
                        x_obs[i],
                        y_obs[i],
                        size_obs[i] * size_obs[i]
                    });
                }
                
                // Build occupancy grid
                Grid grid = occupancy_grid.build(obstacle_list, (int)width1, (int)height1, CELL_PX, INFLATE_PX);
                
                // Run defense strategy
                auto result = defense_strategy.compute(
                    tracks,
                    my_id,
                    grid,
                    CELL_PX,
                    LOOKAHEAD_CELLS,
                    HIDING_SAMPLES,
                    V_MAX,
                    obstacle_list
                );
                
                my_cmd = result.cmd;
                
                // Debug output
                if (frame_count % 60 == 0) {
                    cout << "Defense: vL=" << result.cmd.left << " vR=" << result.cmd.right 
                         << " tactic=" << result.decision.tactic;
                    if (result.hiding_point.has_value()) {
                        auto [hx, hy] = *result.hiding_point;
                        cout << " hiding=(" << hx << "," << hy << ")";
                    }
                    cout << endl;
                }
            }
            
        } else {
            // ============ Navigation Test - A* Pathfinding Demo ============
            
            if (my_id < 0) {
                // Run ID dance first
                my_id = id_dance.run(tc, rgb, detector, tracker);
                my_cmd = id_dance.currentCommand();
                
                if (id_dance.done()) {
                    cout << "ID identified as: " << my_id << " - Starting navigation test!" << endl;
                }
            } else {
                // Build obstacle list from simulation
                std::vector<Obstacle> obstacle_list;
                for (int i = 1; i <= N_obs; i++) {
                    int w = (int)size_obs[i];
                    int h = (int)size_obs[i];
                    
                    // Special handling for boundary walls (last 4 obstacles if addBoundaryObstacles was called)
                    if (i > N_obs - 4) {  // Boundary walls
                        if (i == N_obs - 3 || i == N_obs - 2) {  // Top/bottom walls
                            h = 40;  // Thin height
                        } else {  // Left/right walls
                            w = 40;  // Thin width
                        }
                    }
                    
                    obstacle_list.push_back({
                        static_cast<int>(x_obs[i] - w/2.0),  // x (top-left corner)
                        static_cast<int>(y_obs[i] - h/2.0),  // y (top-left corner)
                        w,                                    // width
                        h,                                    // height
                        x_obs[i],                             // cx (center x)
                        y_obs[i],                             // cy (center y)
                        (double)(w * h)                       // area
                    });
                }
                
                // Build occupancy grid
                Grid grid_planning = occupancy_grid.build(obstacle_list, (int)width1, (int)height1, CELL_PX, INFLATE_PX);
                Grid grid_visual = occupancy_grid.build(obstacle_list, (int)width1, (int)height1, CELL_PX, 0);
                
                static double goal_x = 1100;   // Near bottom-right
                static double goal_y = 850;
                static bool reached_goal = false;
                static int goals_reached = 0;
                
                // Get robot state
                double my_x, my_y, my_theta;
                if (getTrackedRobotState(tracks, my_id, my_x, my_y, my_theta)) {
                    
                    // Compute front-axle position (kinematic control point)
                    double wheel_x, wheel_y;
                    getWheelCenterPosition(my_x, my_y, my_theta, wheel_x, wheel_y, Lx);

                    // Check if goal reached — use wheel center for accuracy
                    double dist_to_goal = sqrt(pow(wheel_x - goal_x, 2) + pow(wheel_y - goal_y, 2));
                    if (dist_to_goal < 50.0 && !reached_goal) {
                        reached_goal = true;
                        goals_reached++;
                        cout << "\n*** GOAL REACHED! (" << goals_reached << " total) ***" << endl;
                        goal_x = 150 + rand() % 980;
                        goal_y = 150 + rand() % 660;
                        cout << "New goal: (" << goal_x << ", " << goal_y << ")" << endl;
                        Sleep(500);
                        reached_goal = false;
                    }

                    auto snap_to_free = [&](const Grid& grid, std::pair<int, int> cell) -> std::pair<int, int> {
                        if (!grid[cell.first][cell.second]) return cell;
                        for (int r = 1; r < 6; r++) {
                            for (int dy = -r; dy <= r; dy++) {
                                for (int dx = -r; dx <= r; dx++) {
                                    int ny = cell.first + dy;
                                    int nx = cell.second + dx;
                                    if (ny >= 0 && ny < (int)grid.size() &&
                                        nx >= 0 && nx < (int)grid[0].size() &&
                                        !grid[ny][nx]) {
                                        return { ny, nx };
                                    }
                                }
                            }
                        }
                        return cell;
                    };

                    // ⭐ Plan from front-axle position, not centroid
                    auto raw_start = std::make_pair((int)(wheel_y / CELL_PX), (int)(wheel_x / CELL_PX));
                    auto start_cell = snap_to_free(grid_planning, raw_start);
                    auto raw_goal = std::make_pair((int)(goal_y / CELL_PX), (int)(goal_x / CELL_PX));
                    auto goal_cell = snap_to_free(grid_planning, raw_goal);

                    auto path = planner.plan(grid_planning, start_cell, goal_cell);

                    DebugVisualizer::drawOccupancyGrid(rgb, grid_visual, CELL_PX, 60);
                    if (path.has_value())
                        DebugVisualizer::drawPath(rgb, *path, CELL_PX, 0, 255, 255);

                    DebugVisualizer::drawRobotCollisionBox(rgb, my_x, my_y, my_theta,
                        ROBOT_COLLISION_WIDTH, ROBOT_COLLISION_LENGTH,
                        &grid_visual, CELL_PX, 0, 255, 255, 255, 0, 255);
                    DebugVisualizer::drawRobot(rgb, my_x, my_y, my_theta, 15, 255, 255, 0);
                    // ⭐ Show front axle as orange dot (same as navigation mode)
                    DebugVisualizer::drawTarget(rgb, wheel_x, wheel_y, 15, 255, 128, 0);
                    DebugVisualizer::drawTarget(rgb, goal_x, goal_y, 30, 0, 255, 0);

                    if (path.has_value() && path->size() > 1) {
                        // Heading error from front-axle to next waypoint
                        double dx = path->at(1).second * CELL_PX + CELL_PX/2.0 - wheel_x;
                        double dy = path->at(1).first  * CELL_PX + CELL_PX/2.0 - wheel_y;
                        double heading_error = fmod(atan2(dy, dx) - my_theta + M_PI, 2*M_PI) - M_PI;

                        int lookahead_idx;
                        if      (std::fabs(heading_error) > M_PI / 2.5) lookahead_idx = std::min(3, (int)path->size() - 1);
                        else if (std::fabs(heading_error) > M_PI / 4)   lookahead_idx = std::min(5, (int)path->size() - 1);
                        else                                             lookahead_idx = std::min(8, (int)path->size() - 1);

                        auto [gy, gx] = path->at(lookahead_idx);
                        double waypoint_x = gx * CELL_PX + CELL_PX / 2.0;
                        double waypoint_y = gy * CELL_PX + CELL_PX / 2.0;

                        DebugVisualizer::drawTarget(rgb, waypoint_x, waypoint_y, 25, 255, 255, 255);

                        // Recalculate for selected waypoint
                        dx = waypoint_x - wheel_x;
                        dy = waypoint_y - wheel_y;
                        heading_error = fmod(atan2(dy, dx) - my_theta + M_PI, 2*M_PI) - M_PI;

                        // ⭐ follower.follow receives the front-axle position
                        Command cmd = follower.follow(
                            wheel_x, wheel_y, my_theta,
                            std::make_pair(waypoint_x, waypoint_y),
                            5.0, 2.5, 0.6, V_MAX
                        );

                        my_cmd.left  = cmd.left;
                        my_cmd.right = cmd.right;
                        my_cmd.laser = false;

                        if (frame_count % 20 == 0) {
                            double dist_to_wp = std::hypot(wheel_x - waypoint_x, wheel_y - waypoint_y);
                            cout << "  WP[" << lookahead_idx << "]: (" << (int)waypoint_x << "," << (int)waypoint_y << ")"
                                 << " dist=" << (int)dist_to_wp
                                 << " | AXLE: (" << (int)wheel_x << "," << (int)wheel_y << ")"
                                 << " | HEAD_ERR=" << (int)(heading_error * 180.0 / M_PI) << "°"
                                 << " | CMD: L=" << cmd.left << " R=" << cmd.right << endl;
                        }

                    } else if (path.has_value() && path->size() == 1) {
                        my_cmd = Command{ 0.0, 0.0, false };
                        if (frame_count % 60 == 0) cout << "  ARRIVED" << endl;
                    } else {
                        my_cmd = Command{ 0.0, 0.0, false };
                        if (frame_count % 60 == 0) cout << "  NO PATH FOUND" << endl;
                    }

                    if (frame_count % 60 == 0) {
                        cout << "Nav: axle=(" << (int)wheel_x << "," << (int)wheel_y << ") "
                             << "goal=(" << (int)goal_x << "," << (int)goal_y << ") "
                             << "dist=" << (int)dist_to_goal;
                        if (path.has_value()) cout << " path_len=" << path->size();
                        else cout << " NO_PATH";
                        cout << endl;
                    }
                }
            }
        }
        
        // ============ Update Your Robot ============
        pw_l = vel_to_pw(my_cmd.left);
        pw_r = vel_to_pw(my_cmd.right);
        laser = my_cmd.laser ? 1 : 0;
        
        set_inputs(pw_l, pw_r, pw_laser, laser,
            light, light_gradient, light_dir, image_noise,
            max_speed, opponent_max_speed);

        // ⭐ Freeze on the laser frame — acquire one more image so sim renders the beam
        if (freeze_on_laser) {
            acquire_image_sim(rgb);   // this frame has laser=1 → sim draws green beam
            view_rgb_image(rgb);
            Sleep(10);
            pause();
        }
        
        // ============ Update Opponent ============
        
        // Get opponent state from tracks
        int opp_id = (my_id == 0) ? 1 : 0;  // Assume opponent is other ID
        bool opp_found = getTrackedRobotState(tracks, opp_id, opp_x, opp_y, opp_theta);
        
        if (!opp_found && tracks.size() > 1) {
            // If we can't find by ID, use the other tracked robot
            for (const auto& t : tracks) {
                if (t.id != my_id) {
                    opp_x = t.x;
                    opp_y = t.y;
                    opp_theta = t.theta;
                    break;
                }
            }
        }
        
        // Get opponent command
        Command opp_cmd = opponent->getCommand(tc, opp_x, opp_y, opp_theta);
        int opp_pw_l = vel_to_pw(opp_cmd.left);
        int opp_pw_r = vel_to_pw(opp_cmd.right);
        
        set_opponent_inputs(opp_pw_l, opp_pw_r, 1500, opp_cmd.laser ? 1 : 0, opponent_max_speed);
        
        // ============ Display ============
        
        if (frame_count % 30 == 0) {
            cout << "Time: " << tc << "s | Frame: " << frame_count 
                 << " | Tracks: " << tracks.size();
            
            if (my_id >= 0) {
                cout << " | My ID: " << my_id;
            } else {
                cout << " | My ID: ?";
            }
            
            cout << endl;
        }
        
        view_rgb_image(rgb);
        
        frame_count++;
        
        // Check timeout
        if (tc > test_duration) {
            test_complete = true;
            cout << "\n=== TEST DURATION COMPLETE ===" << endl;
        }
        
        Sleep(10);
    }
    
    // ============ Test Results ============
    
    cout << "\n========================================" << endl;
    cout << "    TEST COMPLETE" << endl;
    cout << "========================================" << endl;
    cout << "Duration: " << tc << " seconds" << endl;
    cout << "Frames processed: " << frame_count << endl;
    
    if (my_id >= 0) {
        cout << "My Robot ID: " << my_id << endl;
    }
    
    // ============ Cleanup ============
    
    free_image(rgb);
    deactivate_vision();
    deactivate_simulation();
    
    cout << "\nPress any key to exit..." << endl;
    pause();
    
    return 0;
}