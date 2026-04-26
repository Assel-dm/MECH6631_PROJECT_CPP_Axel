#pragma once

#include "Types.h"
#include <optional>
#include <vector>
#include <utility>

namespace GridUtils {

// Convert pixel coordinates to occupancy-grid cell indices (row, col).
std::pair<int,int> pixToCell(double x, double y, int cell_px);

// Convert a grid cell index back to the pixel coordinates of its center.
std::pair<double,double> cellToPix(int row, int col, int cell_px);

// Check whether a grid cell index is inside the grid.
bool inBounds(const Grid& grid, int row, int col);

// Return true if the cell is occupied or invalid.
bool isBlocked(const Grid& grid, int row, int col);

// If the requested cell is blocked, find the nearest free cell using BFS.
// Returns std::nullopt if no free cell is reachable.
std::optional<std::pair<int,int>> snapToNearestFree(
    const Grid& grid,
    std::pair<int,int> start_cell,
    int max_radius_cells = 25);

// Convenience helper:
// - converts a pixel position to a cell
// - snaps it to the nearest free cell if needed
std::optional<std::pair<int,int>> safePixToFreeCell(
    const Grid& grid,
    double x,
    double y,
    int cell_px,
    int max_radius_cells = 25);

// Convert a grid path to pixel waypoints (cell centers).
std::vector<std::pair<double,double>> pathCellsToPixels(
    const std::vector<std::pair<int,int>>& path,
    int cell_px);

} // namespace GridUtils
