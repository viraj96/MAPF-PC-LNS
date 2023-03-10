#pragma once

#include "common.hpp"
#include <plog/Log.h>

class ConstraintTable
{

  protected:
    unordered_map<size_t, size_t> landmarks;
    unordered_map<size_t, list<pair<int, int>>> constraint_table;

    void insertLandmark(size_t location, int timestep);
    inline size_t getEdgeIndex(size_t from, size_t to) const { return (1 + from) * map_size + to; }

  public:
    size_t num_col, map_size;
    int size = 0, length_min = 0, length_max = MAX_TIMESTEP, goal_location = -1,
        latest_timestep = 0;

    vector<int> leq_goal_time, g_goal_time; // what do these do?

    ConstraintTable() = default;
    ConstraintTable(size_t num_col, size_t map_size);
    ConstraintTable(const ConstraintTable& old) { copy(old); }

    int getHoldingTime();
    bool constrained(size_t location, int time) const;
    bool constrained(size_t current_location, size_t next_location, int next_timestep) const;

    void copy(const ConstraintTable& old);
    void insert2CT(size_t loc, int t_min, int t_max);
    void insert2CT(size_t from, size_t to, int t_min, int t_max);

    unordered_map<size_t, size_t> getLandmarks() const { return landmarks; }
};
