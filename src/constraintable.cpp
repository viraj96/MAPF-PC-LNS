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
        if ((int)landmark.second != goal_location)
            holding_time = max(holding_time, (int)landmark.first + 1);
    if (g_goal_time.size() != 0)
        holding_time = max(holding_time, g_goal_time.back() + 1);
    return holding_time;
}

bool
ConstraintTable::constrained(size_t location, int timestep) const
{
    assert(location >= 0);
    if (location < map_size) {
        const auto& it = landmarks.find(timestep);
        if (it != landmarks.end() && it->second != location)
            return true; // violate the positive vertex constraint
    }

    const auto& it = constraint_table.find(location);
    if (it == constraint_table.end())
        return false;
    for (const auto& constraint : it->second)
        if (constraint.first <= timestep && timestep < constraint.second)
            return true;
    return false;
}

bool
ConstraintTable::constrained(size_t current_location, size_t next_location, int next_timestep) const
{
    return constrained(getEdgeIndex(current_location, next_location), next_timestep);
}
