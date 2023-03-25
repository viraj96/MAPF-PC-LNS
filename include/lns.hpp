#pragma once

#include "common.hpp"
#include "constrainttable.hpp"
#include "mlastar.hpp"
#include <numeric>
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

struct Utility
{
    int agent, task_position;
    double value;
    Utility(int agent, int task_position, double value)
      : agent(agent)
      , task_position(task_position)
      , value(value)
    {}

    struct compare_node
    {
        bool operator()(const Utility& lhs, const Utility& rhs) const
        {
            return lhs.value >= rhs.value;
        }
    };
};

struct Regret
{
    int task, agent, task_position;
    double value;
    Regret(int task, int agent, int task_position, double value)
      : task(task)
      , agent(agent)
      , task_position(task_position)
      , value(value)
    {}

    struct compare_node
    {
        bool operator()(const Regret& lhs, const Regret& rhs) const
        {
            return lhs.value <= rhs.value;
        }
    };
};

struct Neighbor
{
    set<int> conflicted_tasks;
    pairing_heap<Regret, compare<Regret::compare_node>> regret_max_heap;
};

class Solution
{
  public:
    Neighbor neighbor;
    vector<Path> paths;
    int num_of_agents, num_of_tasks;
    vector<vector<int>> task_assignments;
    vector<pair<int, int>> precedence_constraints;

    Solution(const Instance& instance)
    {
        num_of_tasks = instance.getTasksNum();
        num_of_agents = instance.getAgentNum();
        task_assignments.resize(num_of_agents);
    }

    int getAgentWithTask(int global_task) const
    {
        for (int i = 0; i < num_of_agents; i++)
            for (int j = 0; j < (int)task_assignments[i].size(); j++)
                if (task_assignments[i][j] == global_task)
                    return i;
        return -1;
    }

    inline void assignTaskToAgent(int agent, int task) { task_assignments[agent].push_back(task); }
    inline vector<int> getAgentGlobalTasks(int agent) const { return task_assignments[agent]; }
    void clearInterAgentPrecedenceConstraint(int task)
    {
        int agent = getAgentWithTask(task), task_position = getLocalTaskIndex(agent, task);
        int previous_task = -1, next_task = -1;
        if (task_position != 0)
            previous_task = task_assignments[agent][task_position - 1];
        if (task_position != (int)task_assignments[agent].size() - 1)
            next_task = task_assignments[agent][task_position + 1];

        precedence_constraints.erase(
          std::remove_if(precedence_constraints.begin(),
                         precedence_constraints.end(),
                         [task, previous_task, next_task](pair<int, int> x) {
                             return ((x.first == previous_task && x.second == task) ||
                                     (x.first == task && x.second == next_task));
                         }),
          precedence_constraints.end());

        if (previous_task != -1 && next_task != -1)
            insertPrecedenceConstraint(previous_task, next_task);
    }
    inline void insertPrecedenceConstraint(int task_a, int task_b)
    {
        precedence_constraints.push_back(make_pair(task_a, task_b));
    }
    int getLocalTaskIndex(int agent, int global_task) const
    {
        for (int i = 0; i < (int)task_assignments[agent].size(); i++) {
            if (task_assignments[agent][i] == global_task)
                return i;
        }
        return -1;
    }
};

class LNS
{
  private:
    int num_of_iterations;

  protected:
    bool ALNS = false;
    const Instance& instance;
    vector<Agent> agents;
    vector<double> destroy_weights;
    int neighbor_size, selected_neighbor;
    Solution solution, previous_solution;
    high_resolution_clock::time_point planner_start_time;
    double time_limit, replan_time_limit, decay_factor = -1, reaction_factor = -1,
                                          preprocessing_time = 0, initial_solution_runtime = 0;

  public:
    vector<Path> initial_paths, paths;
    list<IterationStats> iteration_stats;
    int num_of_failures = 0, sum_of_costs = 0;
    double runtime = 0, average_group_size = -1;

    LNS(int num_of_iterations, const Instance& instance, int neighbor_size, double time_limit);
    inline Instance getInstance() { return instance; }
    bool run();
    void printPaths() const;
    void prepareNextIteration();
    bool validateSolution(bool extract = false);
    void joinPaths(vector<int> agents_to_compute = vector<int>());
    void build_constraint_table(ConstraintTable& constraint_table, int agent, int task);

    void build_constraint_table(ConstraintTable& constraint_table,
                                int task,
                                int task_location,
                                vector<Path>* paths,
                                vector<vector<int>>* task_assignments,
                                vector<pair<int, int>>* precedence_constraints);

    void printAgents() const
    {
        for (int i = 0; i < instance.getAgentNum(); i++) {
            pair<int, int> start_loc = instance.getCoordinate(instance.getStartLocations()[i]);
            PLOGI << "Agent " << i << " : S = (" << start_loc.first << ", " << start_loc.second
                  << ") ;\nGoals : \n";
            for (int j = 0; j < (int)solution.task_assignments[i].size(); j++) {
                pair<int, int> goal_loc = instance.getCoordinate(solution.task_assignments[i][j]);
                PLOGI << "\t" << j << " : (" << goal_loc.first << " , " << goal_loc.second << ")\n";
            }
        }
    }
};
