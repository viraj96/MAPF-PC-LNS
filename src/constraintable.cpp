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

void
ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max)
{
    insert2CT(getEdgeIndex(from, to), t_min, t_max);
}

void
ConstraintTable::insert2CT(size_t location, int t_min, int t_max)
{
    assert(location >= 0);
    constraint_table[location].emplace_back(t_min, t_max);
    if (t_max < MAX_TIMESTEP && t_max > latest_timestep)
        latest_timestep = t_max;
    else if (t_max == MAX_TIMESTEP && t_min > latest_timestep)
        latest_timestep = t_min;
}

void
ConstraintTable::addPath(const Path& path, bool wait_at_goal)
{
    int offset = path.begin_time;
    for (int i = 0; i < (int)path.size() - 1; i++) {
        int timestep = i + offset;
        insert2CT(path[i].location, timestep, timestep + 1);
        insert2CT(path[i + 1].location, path[i].location, timestep + 1, timestep + 2);
    }
    int i = (int)path.size() - 1;
    int timestep = i + offset;
    if (wait_at_goal)
        insert2CT(path[i].location, timestep, MAX_TIMESTEP);
    else
        insert2CT(path[i].location, timestep, timestep + 1);
}
