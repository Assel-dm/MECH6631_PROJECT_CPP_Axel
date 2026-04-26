#include "DebugVisualizer.h"
#include <algorithm>
#include <cmath>

void DebugVisualizer::setPixel(image& rgb, int x, int y, int R, int G, int B) {
    if (x < 0 || x >= rgb.width || y < 0 || y >= rgb.height) return;
    
    ibyte* p = rgb.pdata + 3 * (y * rgb.width + x);
    p[0] = B;  // BGR order
    p[1] = G;
    p[2] = R;
}

void DebugVisualizer::blendPixel(image& rgb, int x, int y, int R, int G, int B, int alpha) {
    if (x < 0 || x >= rgb.width || y < 0 || y >= rgb.height) return;
    
    ibyte* p = rgb.pdata + 3 * (y * rgb.width + x);
    
    // Alpha blend: new = alpha * overlay + (1 - alpha) * existing
    double a = alpha / 255.0;
    p[0] = (ibyte)(a * B + (1.0 - a) * p[0]);  // B
    p[1] = (ibyte)(a * G + (1.0 - a) * p[1]);  // G
    p[2] = (ibyte)(a * R + (1.0 - a) * p[2]);  // R
}

void DebugVisualizer::drawLine(image& rgb, int x0, int y0, int x1, int y1, int R, int G, int B) {
    // Bresenham's line algorithm
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        setPixel(rgb, x0, y0, R, G, B);
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void DebugVisualizer::drawOccupancyGrid(image& rgb, const Grid& grid, int cell_px, int alpha) {
    int gh = grid.size();
    if (gh == 0) return;
    int gw = grid[0].size();

    // Draw each grid cell
    for (int gy = 0; gy < gh; gy++) {
        for (int gx = 0; gx < gw; gx++) {
            bool occupied = (grid[gy][gx] > 0);

            // Calculate pixel coordinates (top-left corner of cell)
            int px = gx * cell_px;
            int py = gy * cell_px;

            // Choose color based on occupancy
            int R, G, B;
            if (occupied) {
                R = 255; G = 0; B = 0;  // Red for obstacles
            } else {
                R = 0; G = 150; B = 0;  // Dark green for free space
            }

            // Draw a small indicator in the center of the cell
            int cx = px + cell_px / 2;
            int cy = py + cell_px / 2;
            int marker_size = std::max(1, cell_px / 4);

            for (int dy = -marker_size; dy <= marker_size; dy++) {
                for (int dx = -marker_size; dx <= marker_size; dx++) {
                    blendPixel(rgb, cx + dx, cy + dy, R, G, B, alpha);
                }
            }
        }
    }
}

void DebugVisualizer::drawPath(image& rgb, const std::vector<std::pair<int, int>>& path, int cell_px, int R, int G, int B) {
    if (path.empty()) return;

    // Draw lines connecting path waypoints
    for (size_t i = 0; i < path.size(); i++) {
        int gy = path[i].first;
        int gx = path[i].second;

        // Cell center in pixels
        int cx = gx * cell_px + cell_px / 2;
        int cy = gy * cell_px + cell_px / 2;

        // Draw circle at waypoint
        int radius = cell_px / 3;
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                if (dx * dx + dy * dy <= radius * radius) {
                    setPixel(rgb, cx + dx, cy + dy, R, G, B);
                }
            }
        }

        // Draw line to next waypoint
        if (i + 1 < path.size()) {
            int next_gy = path[i + 1].first;
            int next_gx = path[i + 1].second;
            int next_cx = next_gx * cell_px + cell_px / 2;
            int next_cy = next_gy * cell_px + cell_px / 2;

            drawLine(rgb, cx, cy, next_cx, next_cy, R, G, B);
        }
    }
}

void DebugVisualizer::drawRobot(image& rgb, double x, double y, double theta, int size, int R, int G, int B) {
    int ix = (int)x;
    int iy = (int)y;

    // Draw circle for robot body
    for (int dy = -size; dy <= size; dy++) {
        for (int dx = -size; dx <= size; dx++) {
            if (dx * dx + dy * dy <= size * size) {
                setPixel(rgb, ix + dx, iy + dy, R, G, B);
            }
        }
    }

    // Draw heading line
    int line_len = (int)(size * 1.5);
    int hx = ix + (int)(line_len * cos(theta));
    int hy = iy + (int)(line_len * sin(theta));
    drawLine(rgb, ix, iy, hx, hy, 255, 255, 255);  // White line for heading
}

void DebugVisualizer::drawTarget(image& rgb, double x, double y, int size, int R, int G, int B) {
    int ix = (int)x;
    int iy = (int)y;

    // Draw crosshair
    drawLine(rgb, ix - size, iy, ix + size, iy, R, G, B);
    drawLine(rgb, ix, iy - size, ix, iy + size, R, G, B);

    // Draw circle around target
    int radius = size / 2;
    for (int angle = 0; angle < 360; angle += 10) {
        double rad = angle * M_PI / 180.0;
        int x1 = ix + (int)(radius * cos(rad));
        int y1 = iy + (int)(radius * sin(rad));
        setPixel(rgb, x1, y1, R, G, B);
    }
}

void DebugVisualizer::drawRobotCollisionBox(
    image& rgb,
    double x, double y, double theta,
    double width, double length,
    const Grid* grid,
    int cell_px,
    int R, int G, int B,
    int R_collision, int G_collision, int B_collision)
{
    // Calculate the 4 corners of the robot's bounding box
    // Robot coordinate system: length along heading, width perpendicular
    double half_length = length / 2.0;
    double half_width = width / 2.0;
    
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    
    // Corners in robot frame (relative to center)
    double corners_local[4][2] = {
        { half_length,  half_width},   // Front-right
        { half_length, -half_width},   // Front-left
        {-half_length, -half_width},   // Rear-left
        {-half_length,  half_width}    // Rear-right
    };
    
    // Transform to world coordinates
    int corners_world[4][2];
    for (int i = 0; i < 4; i++) {
        double wx = x + corners_local[i][0] * cos_th - corners_local[i][1] * sin_th;
        double wy = y + corners_local[i][0] * sin_th + corners_local[i][1] * cos_th;
        corners_world[i][0] = (int)wx;
        corners_world[i][1] = (int)wy;
    }
    
    // Check for collision if grid is provided
    bool in_collision = false;
    if (grid != nullptr) {
        int gh = grid->size();
        int gw = (gh > 0) ? (*grid)[0].size() : 0;
        
        // Check if any corner is in an occupied cell
        for (int i = 0; i < 4; i++) {
            int gx = corners_world[i][0] / cell_px;
            int gy = corners_world[i][1] / cell_px;
            
            if (gx >= 0 && gx < gw && gy >= 0 && gy < gh) {
                if ((*grid)[gy][gx] > 0) {
                    in_collision = true;
                    break;
                }
            }
        }
        
        // Also check center point
        int cx = (int)x / cell_px;
        int cy = (int)y / cell_px;
        if (cx >= 0 && cx < gw && cy >= 0 && cy < gh) {
            if ((*grid)[cy][cx] > 0) {
                in_collision = true;
            }
        }
    }
    
    // Choose color based on collision state
    int draw_R = in_collision ? R_collision : R;
    int draw_G = in_collision ? G_collision : G;
    int draw_B = in_collision ? B_collision : B;
    
    // Draw the 4 edges of the box
    for (int i = 0; i < 4; i++) {
        int next = (i + 1) % 4;
        drawLine(rgb, 
            corners_world[i][0], corners_world[i][1],
            corners_world[next][0], corners_world[next][1],
            draw_R, draw_G, draw_B);
    }
    
    // Draw thicker lines by offsetting
    for (int i = 0; i < 4; i++) {
        int next = (i + 1) % 4;
        drawLine(rgb, 
            corners_world[i][0] + 1, corners_world[i][1],
            corners_world[next][0] + 1, corners_world[next][1],
            draw_R, draw_G, draw_B);
        drawLine(rgb, 
            corners_world[i][0], corners_world[i][1] + 1,
            corners_world[next][0], corners_world[next][1] + 1,
            draw_R, draw_G, draw_B);
    }
    
    // Draw heading indicator (line from center to front)
    int front_x = (int)(x + half_length * cos_th);
    int front_y = (int)(y + half_length * sin_th);
    drawLine(rgb, (int)x, (int)y, front_x, front_y, 255, 255, 255);  // White heading line
}