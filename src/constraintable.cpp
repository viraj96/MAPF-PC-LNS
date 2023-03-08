#include "constrainttable.hpp"

int
ConstraintTable::getHoldingTime()
{
    int holding_time = length_min;
    auto it = constraint_table.find(goal_location);
    if (it != constraint_table.end()) {
        for (pair<int, int> time_range : it->second)
            holding_time = max(holding_time, time_range.second);
    }
    for (pair<size_t, size_t> landmark : landmarks)
        if (landmark.second != goal_location)
            holding_time = max(holding_time, (int)landmark.first + 1);
    if (g_goal_time.size() != 0)
        holding_time = max(holding_time, g_goal_time.back() + 1);
    return holding_time;
}
