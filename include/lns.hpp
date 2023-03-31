#pragma once

#include "common.hpp"
#include "constrainttable.hpp"
#include "mlastar.hpp"
#include <limits>
#include <numeric>
#include <plog/Log.h>

struct Agent
{
    int id;
    Path path;
    vector<Path> task_paths;
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

    Utility()
    {
        agent = -1;
        task_position = -1;
        value = std::numeric_limits<double>::max();
    }

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
    map<int, int> conflicted_tasks_path_size;
    pairing_heap<Regret, compare<Regret::compare_node>> regret_max_heap;
};

class Solution
{
  public:
    int sum_of_costs;
    Neighbor neighbor;
    vector<Path> paths;
    vector<Agent> agents;
    int num_of_agents, num_of_tasks;
    vector<vector<int>> task_assignments;
    vector<pair<int, int>> precedence_constraints;

    Solution(const Instance& instance)
    {
        num_of_tasks = instance.getTasksNum();
        num_of_agents = instance.getAgentNum();
        task_assignments.resize(num_of_agents);

        agents.reserve(num_of_agents);
        for (int i = 0; i < num_of_agents; i++)
            agents.emplace_back(instance, i);
    }

    int getAgentWithTask(int global_task) const
    {
        for (int i = 0; i < num_of_agents; i++)
            for (int j = 0; j < (int)task_assignments[i].size(); j++)
                if (task_assignments[i][j] == global_task)
                    return i;
        assert(false);
    }

    int getLocalTaskIndex(int agent, int global_task) const
    {
        for (int i = 0; i < (int)task_assignments[agent].size(); i++) {
            if (task_assignments[agent][i] == global_task)
                return i;
        }
        assert(false);
    }

    inline vector<int> getAgentGlobalTasks(int agent) const { return task_assignments[agent]; }
    inline int getAgentGlobalTasks(int agent, int task_index) const
    {
        return task_assignments[agent][task_index];
    }

    inline int getAssignedTaskSize(int agent) const { return (int)task_assignments[agent].size(); }
    inline void assignTaskToAgent(int agent, int task) { task_assignments[agent].push_back(task); }

    inline void insertPrecedenceConstraint(int task_a, int task_b)
    {
        precedence_constraints.push_back(make_pair(task_a, task_b));
    }

    void clearIntraAgentPrecedenceConstraint(int task)
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

    void joinPaths(vector<int> agents_to_compute)
    {
        for (int agent : agents_to_compute) {

            assert(getAssignedTaskSize(agent) ==
                   (int)agents[agent].path_planner->goal_locations.size());
            assert(getAssignedTaskSize(agent) == (int)agents[agent].task_paths.size());

            for (int i = 0; i < getAssignedTaskSize(agent); i++) {
                if (i == 0)
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].front());
                assert((int)agents[agent].path.size() - 1 ==
                       agents[agent].task_paths[i].begin_time);
                for (int j = 1; j < (int)agents[agent].task_paths[i].size(); j++)
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].at(j));
                agents[agent].path.timestamps.push_back(agents[agent].path.size() - 1);
            }
        }
    }
};

class LNS
{
  private:
    int num_of_iterations;

  protected:
    int neighbor_size;
    const Instance& instance;
    Solution solution, previous_solution;
    double time_limit, initial_solution_runtime = 0;
    high_resolution_clock::time_point planner_start_time;

  public:
    double runtime = 0;
    vector<Path> initial_paths;
    list<IterationStats> iteration_stats;
    int num_of_failures = 0, sum_of_costs = 0;

    LNS(int num_of_iterations, const Instance& instance, int neighbor_size, double time_limit);

    inline Instance getInstance() { return instance; }

    bool run();
    void prepareNextIteration();
    void printPaths() const;
    bool validateSolution(set<int>* conflicted_tasks = nullptr);
    void build_constraint_table(ConstraintTable& constraint_table, int task);

    void build_constraint_table(ConstraintTable& constraint_table,
                                int task,
                                int task_location,
                                vector<Path>* paths,
                                vector<vector<int>>* task_assignments,
                                vector<pair<int, int>>* precedence_constraints);

    void computeRegret();
    void regretBasedReinsertion();
    void computeRegretForTask(int task);
    void commitBestRegretTask(Regret best_regret);
    void computeRegretForTaskWithAgent(
      int task,
      int agent,
      unordered_set<int>* previous_tasks,
      vector<pair<int, int>>* precedence_constraints,
      pairing_heap<Utility, compare<Utility::compare_node>>* service_times);
    Utility insertTask(int task,
                       int agent,
                       int task_position,
                       vector<Path>* task_paths,
                       vector<vector<int>>* task_assignments,
                       vector<pair<int, int>>* precedence_constraints,
                       bool commit = false);

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
