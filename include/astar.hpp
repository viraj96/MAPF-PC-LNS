#pragma once

#include "common.hpp"
#include "constrainttable.hpp"
#include "instance.hpp"
#include <plog/Log.h>

class LLNode
{
  public:
    vector<int> timestamps;
    vector<int> secondary_keys;

    LLNode* parent = nullptr;
    int location, g_val, h_val = 0, timestep = 0, num_of_conflicts = 0;
    bool in_openlist = false, wait_at_goal = false;
    unsigned int stage = 0, distance_to_next = 0;

    struct open_compare_node
    {
        bool operator()(const LLNode* lhs, const LLNode* rhs) const
        {
            if (lhs->g_val + lhs->h_val == rhs->g_val + rhs->h_val) {
                if (lhs->h_val == rhs->h_val)
                    return rand() % 2 == 0;
                return lhs->h_val >= rhs->h_val;
            }
            return lhs->g_val + lhs->h_val >= rhs->g_val + rhs->h_val;
        }
    };

    struct compare_timestamps
    {
        int operator()(const LLNode* lhs, const LLNode* rhs) const
        {
            for (int i = 0; i < min((int)lhs->timestamps.size(), (int)rhs->timestamps.size()); i++)
                if (lhs->timestamps[i] != rhs->timestamps[i])
                    return lhs->timestamps[i] > rhs->timestamps[i] ? 1 : -1;

            int lhs_last_val, rhs_last_val;
            if (lhs->timestamps.size() > rhs->timestamps.size()) {
                lhs_last_val = lhs->timestamps[rhs->timestamps.size()];
                rhs_last_val = rhs->g_val + rhs->distance_to_next;
            } else if (lhs->timestamps.size() < rhs->timestamps.size()) {
                lhs_last_val = lhs->g_val + lhs->distance_to_next;
                rhs_last_val = rhs->timestamps[lhs->timestamps.size()];
            } else {
                lhs_last_val = lhs->g_val + lhs->distance_to_next;
                rhs_last_val = rhs->g_val + rhs->distance_to_next;
            }

            if (lhs_last_val != rhs_last_val)
                return lhs_last_val > rhs_last_val ? 1 : -1;

            return 0;
        }
    };

    struct focal_compare_node
    {
        bool operator()(const LLNode* lhs, const LLNode* rhs) const
        {
            for (int i = 0;
                 i < min((int)lhs->secondary_keys.size(), (int)rhs->secondary_keys.size());
                 i++)
                if (lhs->secondary_keys[i] != rhs->secondary_keys[i])
                    return lhs->secondary_keys[i] > rhs->secondary_keys[i];
            int timestamps_comparison = compare_timestamps()(lhs, rhs);
            if (timestamps_comparison != 0)
                return timestamps_comparison == 1 ? true : false;
            if (lhs->num_of_conflicts == rhs->num_of_conflicts) {
                if (lhs->g_val + lhs->h_val == rhs->g_val + rhs->h_val) {
                    if (lhs->h_val == rhs->h_val)
                        return rand() % 2 == 0;
                    return lhs->h_val >= rhs->h_val;
                }
                return lhs->g_val + lhs->h_val >= rhs->g_val + rhs->h_val;
            }
            return lhs->num_of_conflicts >= rhs->num_of_conflicts;
        }
    };

    LLNode() {}
    LLNode(LLNode* parent,
           int location,
           int g_val,
           int h_val,
           int timestep,
           int num_of_conflicts,
           unsigned int stage)
      : parent(parent)
      , location(location)
      , g_val(g_val)
      , h_val(h_val)
      , timestep(timestep)
      , num_of_conflicts(num_of_conflicts)
      , in_openlist(false)
      , wait_at_goal(false)
      , stage(stage)
    {}
    LLNode(const LLNode& old) { copy(old); }

    inline double getFVal() const { return g_val + h_val; }

    void copy(const LLNode& old)
    {
        location = old.location;
        g_val = old.g_val;
        h_val = old.h_val;
        parent = old.parent;
        timestep = old.timestep;
        num_of_conflicts = old.num_of_conflicts;
        wait_at_goal = old.wait_at_goal;
    }
};

class SingleAgentSolver
{
  protected:
    void compute_heuristics();

  public:
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;

    bool use_timestamps = true;

    const Instance& instance;

    int start_location;
    vector<int> goal_locations;
    vector<int> heuristic_landmarks;

    vector<vector<int>> heuristic;
    int get_heuristic(int stage, int location) const
    {
        return heuristic[stage][location] + heuristic_landmarks[stage];
    }
    int compute_heuristic(int from, int to) const
    {
        return instance.getManhattanDistance(from, to);
    }
    inline void setGoalLocations(vector<int> goals) { goal_locations = goals; }

    virtual string getName() const = 0;
    virtual Path findPathSegment(ConstraintTable& constraint_table,
                                 int start_time,
                                 int stage,
                                 int lower_bound) = 0;
    /* virtual int getTravelTime(int start, */
    /*                           int end, */
    /*                           const ConstraintTable& constraint_table, */
    /*                           int upper_bound) = 0; */

    list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }

    SingleAgentSolver(const Instance& instance, int agent)
      : instance(instance)
      , start_location(instance.start_locations[agent])
      , goal_locations(instance.task_locations)
    {
        compute_heuristics();
    }
    virtual ~SingleAgentSolver() = default;
};
