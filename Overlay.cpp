#include "Overlay.h"
#include <cstring>

void draw_circle_rgb(image& img, int cx, int cy, int r, int R, int G, int B)
{
    for (int angle = 0; angle < 360; angle++)
    {
        double rad = angle * M_PI / 180.0;
        int x = cx + (int)(r * cos(rad));
        int y = cy + (int)(r * sin(rad));

        if (x >= 0 && x < img.width && y >= 0 && y < img.height)
            draw_point_rgb(img, x, y, R, G, B);
    }
}

// 5x7 bitmap font for ASCII 32–127 (digits only + a few letters used in labels)
static const unsigned char font5x7[][5] = {
    // digits 0–9
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
};

// Additional characters used in labels
static const unsigned char font_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00}; // 'I'
static const unsigned char font_D[5] = {0x7F, 0x41, 0x41, 0x41, 0x3E}; // 'D'
static const unsigned char font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00}; // ' '

void draw_char(image& img, int x, int y, char c, int R, int G, int B)
{
    const unsigned char* bitmap = nullptr;

    if (c >= '0' && c <= '9') {
        bitmap = font5x7[c - '0'];
    } else if (c == 'I') {
        bitmap = font_I;
    } else if (c == 'D') {
        bitmap = font_D;
    } else if (c == ' ') {
        bitmap = font_space;
    } else {
        // unsupported character -> draw as space
        bitmap = font_space;
    }

    for (int col = 0; col < 5; col++)
    {
        unsigned char bits = bitmap[col];
        for (int row = 0; row < 7; row++)
        {
            if (bits & (1 << row))
            {
                int px = x + col;
                int py = y + row;
                if (px >= 0 && px < img.width && py >= 0 && py < img.height)
                    draw_point_rgb(img, px, py, R, G, B);
            }
        }
    }
}

void draw_text_rgb(image& img, int x, int y, const char* text, int R, int G, int B)
{
    int offset = 0;
    while (*text)
    {
        draw_char(img, x + offset, y, *text, R, G, B);
        offset += 6; // 5 pixels + 1 spacing
        text++;
    }
}

void draw_line_rgb(image& img, int x1, int y1, int x2, int y2, int R, int G, int B)
{
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;

    while (true)
    {
        draw_point_rgb(img, x1, y1, R, G, B);
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

void draw_rect_rgb(image& img, int x, int y, int w, int h, int R, int G, int B)
{
    draw_line_rgb(img, x, y, x + w - 1, y, R, G, B);
    draw_line_rgb(img, x + w - 1, y, x + w - 1, y + h - 1, R, G, B);
    draw_line_rgb(img, x + w - 1, y + h - 1, x, y + h - 1, R, G, B);
    draw_line_rgb(img, x, y + h - 1, x, y, R, G, B);
}

void draw_filled_rect_rgb(image& img, int x, int y, int w, int h, int R, int G, int B)
{
    for (int yy = y; yy < y + h; yy++)
    {
        if (yy < 0 || yy >= img.height) continue;
        draw_line_rgb(img, x, yy, x + w - 1, yy, R, G, B);
    }
}

void draw_arrow_rgb(image& img, int cx, int cy, double theta, int len, int R, int G, int B)
{
    // main shaft
    int x2 = cx + (int)(len * cos(theta));
    int y2 = cy + (int)(len * sin(theta));
    draw_line_rgb(img, cx, cy, x2, y2, R, G, B);

    // arrowhead
    double ang1 = theta + M_PI * 3.0 / 4.0;
    double ang2 = theta - M_PI * 3.0 / 4.0;
    int hx1 = x2 + (int)(10 * cos(ang1));
    int hy1 = y2 + (int)(10 * sin(ang1));
    int hx2 = x2 + (int)(10 * cos(ang2));
    int hy2 = y2 + (int)(10 * sin(ang2));
    draw_line_rgb(img, x2, y2, hx1, hy1, R, G, B);
    draw_line_rgb(img, x2, y2, hx2, hy2, R, G, B);
}

void draw_id_label(image& img, int x, int y, int id)
{
    char buf[32];
    sprintf(buf, "ID %d", id);
    draw_text_rgb(img, x, y, buf, 255, 255, 0);
}

void draw_robot_overlay(image& img, const RobotTrack& tr, int R, int G, int B)
{
    // center
    draw_circle_rgb(img, (int)tr.x, (int)tr.y, 10, R, G, B);

    // heading arrow
    draw_arrow_rgb(img, (int)tr.x, (int)tr.y, tr.theta, 30, R, G, B);

    // ID label
    draw_id_label(img, (int)tr.x + 12, (int)tr.y - 12, tr.id);
}

void draw_obstacle_overlay(image& img, const Obstacle& obs, int R, int G, int B)
{
    int x = obs.x;
    int y = obs.y;
    int w = obs.w;
    int h = obs.h;

    // draw bounding box (same as before)
    draw_rect_rgb(img, x, y, w, h, R, G, B);

    // draw centroid marker similar to the blue/red blobs so obstacles are clearly visible
    int cx = (int)std::round(obs.cx);
    int cy = (int)std::round(obs.cy);
    draw_circle_rgb(img, cx, cy, 8, R, G, B);

    // area label (rounded) with a small background for readability
    char buf[32];
    int area_i = (int)std::round(obs.area);
    sprintf(buf, "A:%d", area_i);

    int tx = x;
    int ty = y - 10; // place text above bbox if possible
    int tw = (int)std::strlen(buf) * 6;
    int th = 9;

    // background (semi-opaque effect by drawing a black rectangle behind the text)
    draw_filled_rect_rgb(img, tx, ty, tw, th, 0, 0, 0);

    // text (slightly offset for padding)
    draw_text_rgb(img, tx + 1, ty + 1, buf, R, G, B);
}

// Draw robot safety mask as a rotated rectangle
void draw_robot_mask_overlay(image& img, const RobotDet& det, int length_px, int width_px, int R, int G, int B)
{
    double cx = det.x;
    double cy = det.y;
    double theta = det.theta;
    
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);
    
    // ⭐ Match ObstaclePipeline coordinate system
    double hl = length_px / 2.0;  // half-length along robot's forward axis
    double hw = width_px / 2.0;   // half-width perpendicular to forward
    
    // Corners in local frame (length=X, width=Y)
    double corners[4][2] = {
        {-hl, -hw},  // rear-left
        {+hl, -hw},  // front-left
        {+hl, +hw},  // front-right
        {-hl, +hw}   // rear-right
    };
    
    int px[4], py[4];
    for (int i = 0; i < 4; ++i) {
        double rx = corners[i][0];
        double ry = corners[i][1];
        // Rotate by theta
        double wx = cx + rx * cos_t - ry * sin_t;
        double wy = cy + rx * sin_t + ry * cos_t;
        px[i] = (int)std::round(wx);
        py[i] = (int)std::round(wy);
    }
    
    // Draw rectangle
    for (int i = 0; i < 4; ++i) {
        int next = (i + 1) % 4;
        draw_line_rgb(img, px[i], py[i], px[next], py[next], R, G, B);
    }
}