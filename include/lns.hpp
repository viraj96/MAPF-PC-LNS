#pragma once

#include "common.hpp"
#include "constrainttable.hpp"
#include "mlastar.hpp"
#include <plog/Log.h>

struct Agent
{
    int id;
    Path path;
    vector<Path*> task_paths;
    SingleAgentSolver* path_planner = nullptr;

    Agent(const Instance& instance, int id)
      : id(id)
    {
        path_planner = new MultiLabelSpaceTimeAStar(instance, id);
    }
    ~Agent() { delete path_planner; }
};

struct Potential
{
    vector<int> id_base;
    vector<vector<int>> task_assignments;
    vector<pair<int, int>> precedence_constraints, id_to_agent_task;

    int agent_task_to_id(pair<int, int> agent_task) const
    {
        int agent = agent_task.first, task = agent_task.second;
        return id_base[agent] + task;
    }

    vector<int> tasks;
    set<pair<int, int>> conflicting_pairs_of_tasks;
};

class LNS
{
  private:
    int num_of_iterations;

  protected:
    bool ALNS = false;
    Neighbor neighbor;
    Instance& instance;
    vector<Agent> agents;
    vector<double> destroy_weights;
    int neighbor_size, selected_neighbor;
    high_resolution_clock::time_point start_time;
    double time_limit, replan_time_limit, decay_factor = -1, reaction_factor = -1,
                                          preprocessing_time = 0, initial_solution_runtime = 0;

  public:
    vector<Path> initial_paths, paths;
    list<IterationStats> iteration_stats;
    int num_of_failures = 0, sum_of_costs = 0;
    double runtime = 0, average_group_size = -1;

    LNS(int num_of_iterations, Instance& instance, int neighbor_size, double time_limit);
    inline Instance getInstance() { return instance; }
    bool run();
    void joinPaths();
    void printPaths() const;
    bool validateSolution(set<int>* conflicted_tasks = nullptr);
    void build_constraint_table(ConstraintTable& constraint_table, int agent, int task);
};
