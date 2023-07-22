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
    int agent, task_position, task; // adding task capability since not using Regret
    double value;

    Utility()
    {
        agent = -1;
        task_position = -1;
        value = std::numeric_limits<double>::max();
        task = -1;
    }

    Utility(int agent, int task_position, int task, double value) // adding task capability
      : agent(agent)
      , task_position(task_position)
      , task(task) // appending constructor for task
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

struct Combination
{
    vector<Utility> combo_bucket; // storing all the utility from each task
    int rank_sum = 0;

    inline bool operator< (const Combination &ob) const // functor to arrange combinations
    {
        return (rank_sum <= ob.rank_sum);
    }
};
struct Matching
{
    vector<Combination> combination_list; // store all the combinations
    vector<pair<int, vector<Utility>>> all_services; // store all the service times
    vector<int> task_order; // to cross check task order

    void clearall() // before starting clear all vectors
    {
        task_order.clear();
        combination_list.clear();
        all_services.clear();
    }
};


class Solution
{
  public:
    int sum_of_costs;
    Neighbor neighbor;
    Matching combo_prog; // adding matching to be a part of solution class
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

    void computeRegret(Solution* solution);
    void regretBasedReinsertion();
    void computeRegretForMetaTask(deque<int> meta_task);
    void computeRegretForTask(int task);
    void commitBestRegretTask(Regret best_regret);
    void computeRegretForTaskWithAgent(
      int task,
      int agent,
      int earliest_timestep,
      int latest_timestep,
      vector<pair<int, int>>* precedence_constraints,
    //   pairing_heap<Utility, compare<Utility::compare_node>>* service_times);
      vector<Utility>* service_times); // since we changed the need for checking all service times
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

    // add the recursive combo builder function here
    vector<Combination> recursive_combo_builder(int index, vector<pair<int, vector<Utility>>> tasks_agents, Solution* solution)
    {
        int vec_size = tasks_agents.size();
        if ((index+1) == (vec_size-1)) // compare both index and task
        {
            assert(solution->combo_prog.task_order[index+1] == tasks_agents[index+1].first); // for debugging
            vector<Combination> combo_list;
            vector<Utility> service_time;
            service_time = tasks_agents[index+1].second; // the vector
            for(Utility elem : service_time)
            {
                Combination combo;
                combo.combo_bucket.push_back(elem);
                combo.rank_sum += elem.value;
                combo_list.push_back(combo);
            }
            return combo_list;
        }
        else
        {
            assert((index+1) < vec_size); // just ensuring that logic is not wrong
            assert(solution->combo_prog.task_order[index+1] == tasks_agents[index+1].first); // for debugging
            vector<Combination> combo_list;
            vector<Utility> service_time;
            service_time = tasks_agents[index+1].second;
            // loop over service times for consecutive conflicted tasks while recursively calling the function
            for(Utility elem : service_time)
            {
                vector<Combination> child_combo_list;
                child_combo_list = recursive_combo_builder(index+1, tasks_agents, solution);
                for(Combination c : child_combo_list)
                {
                    c.combo_bucket.push_back(elem);
                    c.rank_sum +=  elem.value;
                    combo_list.push_back(c);
                }
            }
            return combo_list;
        }
    }

    // adding the program function
    void ComboRunProgram(Solution* solution)
    {
        vector<Utility> first_task = solution->combo_prog.all_services[0].second;
        if(solution->combo_prog.all_services.size() > 1)
        {
            for( Utility s : first_task)
            {
                // start the recursion loop
                int index = 0; // will this help or should I do something else?
                vector<Combination> combo_list;
                combo_list = recursive_combo_builder(index, solution->combo_prog.all_services, solution);
                // loop over all service times for first task, order doesn't matter
                for(Combination c: combo_list)
                {
                    c.combo_bucket.push_back(s);;
                    c.rank_sum += s.value;
                    solution->combo_prog.combination_list.push_back(c); // storing the combination in big combination list
                }
            }
        }
        else // when there is only one conflicted task
        {
            for( Utility s : first_task)
            {
                Combination c;
                c.combo_bucket.push_back(s);
                c.rank_sum = s.value;
                solution->combo_prog.combination_list.push_back(c); 
            }
        }
        // ordering combinations based on the total value of the sum of each service time in the combination for all conflicted tasks
        std::sort(solution->combo_prog.combination_list.begin(), solution->combo_prog.combination_list.end());
    }

    bool ComboTemporalChecker(const Instance* instance, Solution* solution)
    {
        unordered_map<int, vector<int>> tasks_depen;
        tasks_depen = instance->getTaskDependencies(); // this is giving wrong tasks dependencies
        set<int> local_conflict_check = solution->neighbor.conflicted_tasks;
        for(pair<int, vector<int>> dependency : instance->getTaskDependencies())
        {
            // int child_task = distance(tasks_depen.begin(), it);
            int child_task = dependency.first;
            if (local_conflict_check.find(child_task) == local_conflict_check.end())
            {
                int child_agent = solution->getAgentWithTask(child_task);
                int child_index = solution->getLocalTaskIndex(child_agent, child_task);
                int child_timestamp = solution->agents[child_agent].task_paths[child_index].end_time();
                vector<int> child_ancestors = dependency.second;
                for(int ancestor_task: child_ancestors)
                {
                    if (local_conflict_check.find(ancestor_task) == local_conflict_check.end())
                    {
                        int ancestor_agent = solution->getAgentWithTask(ancestor_task);
                        int ancestor_index = solution->getLocalTaskIndex(ancestor_agent, ancestor_task);
                        int ancestor_timestamp = solution->agents[ancestor_agent].task_paths[ancestor_index].end_time();
                        if (child_timestamp <= ancestor_timestamp)
                        {
                            PLOGI <<" Child task = " << child_task <<" Child end timestamp = "<< child_timestamp;
                            PLOGI <<" Ancestor task = "<< ancestor_task <<" Ancestor end timestamp = "<< ancestor_timestamp;
                            // PLOGI <<" Regret for Task=" << task << " in Agent = "<< agent << " by pushing local task= " << next_task << " out";
                            PLOGI <<" Found a TEMPORAL VIOLATION commit phase, child task = " << child_task << " ancestor task = " << ancestor_task;
                            // value = INT_MIN;
                            return true; // temporal issue found so move on to next task
                        }
                    }
                }
            }
        }
        return false; // means everything is okay and time for cost check
    }


    bool ComboCostChecker(const Instance* instance, Solution* solution, Solution* prev_solution)
    {
        int Tcosts = 0;
        for (int taskz = 0; taskz < instance->getTasksNum(); taskz++) {
            Tcosts += (solution->paths[taskz].end_time() - solution->paths[taskz].begin_time);
        }
        PLOGI << " Combination total cost = " << Tcosts;
        if (prev_solution->sum_of_costs < Tcosts)
        {
            PLOGI << "Combination cost is higher, cannot use!" << endl;
            return true;
        }
        else
        {
            return false; // means we have a lower cost solution
        }
    }    
};
