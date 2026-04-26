#pragma once
#include "Types.h"
#include <vector>
#include <optional>
#include <utility>

class AStarPlanner {
public:
    AStarPlanner() = default;

    std::optional<std::vector<std::pair<int, int>>> plan(
        const Grid& grid,
        std::pair<int, int> start,
        std::pair<int, int> goal);
};
