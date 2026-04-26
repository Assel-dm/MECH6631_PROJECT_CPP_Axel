#pragma once
#include "Types.h"
#include "image_transfer.h"
#include <vector>
#include <utility>

class DebugVisualizer {
public:
    // Draw occupancy grid overlay on RGB image
    // - Occupied cells = RED, Free cells = GREEN (semi-transparent)
    static void drawOccupancyGrid(
        image& rgb,
        const Grid& grid,
        int cell_px,
        int alpha = 80  // Transparency: 0 (transparent) to 255 (opaque)
    );

    // Draw A* path on RGB image
    static void drawPath(
        image& rgb,
        const std::vector<std::pair<int, int>>& path,
        int cell_px,
        int R = 0, int G = 255, int B = 255  // Cyan by default
    );

    // Draw robot position with heading indicator
    static void drawRobot(
        image& rgb,
        double x, double y, double theta,
        int size = 20,
        int R = 255, int G = 255, int B = 0  // Yellow by default
    );

    // Draw a waypoint target
    static void drawTarget(
        image& rgb,
        double x, double y,
        int size = 15,
        int R = 255, int G = 0, int B = 0  // Red by default
    );

    // Draw robot collision box (oriented rectangle showing physical footprint)
    // - Optional collision checking: if grid provided, colors box RED if in collision
    static void drawRobotCollisionBox(
        image& rgb,
        double x, double y, double theta,
        double width, double length,  // Robot dimensions in pixels
        const Grid* grid = nullptr,   // Optional: provide grid to check collision
        int cell_px = 10,              // Grid cell size (needed if checking collision)
        int R = 0, int G = 255, int B = 255,  // Color when NOT in collision
        int R_collision = 255, int G_collision = 0, int B_collision = 0  // Color when in collision
    );

private:
    // Helper: Set pixel with bounds check
    static void setPixel(image& rgb, int x, int y, int R, int G, int B);
    
    // Helper: Blend pixel with existing color (for transparency)
    static void blendPixel(image& rgb, int x, int y, int R, int G, int B, int alpha);
    
    // Helper: Draw a line
    static void drawLine(image& rgb, int x0, int y0, int x1, int y1, int R, int G, int B);
};
