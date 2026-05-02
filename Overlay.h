#pragma once
#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include <cmath>
#include <cstdio>


// Draw a colored circle on an RGB image
void draw_circle_rgb(image& img, int cx, int cy, int r, int R, int G, int B);

// Draw a colored line on an RGB image
void draw_line_rgb(image& img, int x1, int y1, int x2, int y2, int R, int G, int B);

// Draw robot ID text (uses draw_text_rgb)
void draw_id_label(image& img, int x, int y, int id);

// Draw a single character at (x,y)
void draw_char(image& img, int x, int y, char c, int R, int G, int B);

// Draw a string at (x,y)
void draw_text_rgb(image& img, int x, int y, const char* text, int R, int G, int B);

// Draw rectangle (outline)
void draw_rect_rgb(image& img, int x, int y, int w, int h, int R, int G, int B);

// Draw filled rectangle
void draw_filled_rect_rgb(image& img, int x, int y, int w, int h, int R, int G, int B);

// Draw an arrow at (cx,cy) with angle theta and length len
void draw_arrow_rgb(image& img, int cx, int cy, double theta, int len, int R, int G, int B);

// Draw a robot visualization (center circle, heading arrow, ID label)
void draw_robot_overlay(image& img, const RobotTrack& tr, int R, int G, int B);

// Draw an obstacle visualization (bbox + area label)
void draw_obstacle_overlay(image& img, const Obstacle& obs, int R, int G, int B);

