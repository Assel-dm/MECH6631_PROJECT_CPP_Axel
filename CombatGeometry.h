#pragma once

#include "Types.h"
#include <optional>
#include <vector>
#include <utility>

namespace CombatGeometry {

// Distance from point p to segment [a,b].
double pointLineDistance(double px, double py,
                         double ax, double ay,
                         double bx, double by);

// Approximate whether an obstacle blocks the line between two robots.
// The obstacle is approximated by a disc centered at (cx,cy) with radius
// derived from its bounding box plus a safety margin.
bool obstacleBlocksLine(double my_x, double my_y,
                        double enemy_x, double enemy_y,
                        const Obstacle& obs,
                        double margin_px = 10.0);

// Return true if no obstacle blocks the current line of sight.
bool lineOfSightClear(double my_x, double my_y,
                      double enemy_x, double enemy_y,
                      const std::vector<Obstacle>& obstacles,
                      double margin_px = 10.0);

// Count how many obstacles approximately block the line of sight.
int countBlockingObstacles(double my_x, double my_y,
                           double enemy_x, double enemy_y,
                           const std::vector<Obstacle>& obstacles,
                           double margin_px = 10.0);

// Distance from the robot to the closest obstacle center.
double nearestObstacleDistance(double my_x, double my_y,
                               const std::vector<Obstacle>& obstacles);

// Select the most promising obstacle to hide behind.
std::optional<Obstacle> bestHidingObstacle(double my_x, double my_y,
                                           double enemy_x, double enemy_y,
                                           const std::vector<Obstacle>& obstacles);

// Compute a target point on the far side of an obstacle from the enemy viewpoint.
std::pair<double,double> hidingPointBehindObstacle(double my_x, double my_y,
                                                   double enemy_x, double enemy_y,
                                                   const Obstacle& obs,
                                                   double stand_off_px = 35.0);

} // namespace CombatGeometry
