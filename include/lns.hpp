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
    // ~Agent() { delete path_planner; } // Delete this since it was originally throwing an error
};

struct Utility
{
    int agent, task_position, task, endtime; // Adding 1. task 2. end time as additional variables
    double value;
    Path saved_path_task; // adding 3. task Path

    Utility()
    {
        agent = -1;
        task_position = -1;
        value = std::numeric_limits<double>::max();
        task = -1;
        endtime = -1;
        saved_path_task = Path();
    }

    Utility(int agent, int task_position, int task,int time, double value, Path temp) // adding task capability
      : agent(agent)
      , task_position(task_position)
      , task(task) // appending constructor for task
      , endtime(time) // appending constructor for endtime
      , value(value)
      , saved_path_task(temp) // appending constructor for Path
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
    // adding 1. end time (will come from utility)
    // 2. task Path (will come from utility)
    int task, agent, task_position, endtime;
    double value;
    Path saved_path_task;

    Regret(int task, int agent, int task_position, int time, double value, Path temp)
      : task(task)
      , agent(agent)
      , task_position(task_position)
      , endtime(time) // appending constructor with end time
      , value(value)
      , saved_path_task(temp) // appending constructor with task Path
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

struct Combination // Currently not being used, but can be useful for combinations of regrets
{
    vector<Utility> combo_bucket; // storing all the utility from each task
    int rank_sum = 0;

    inline bool operator< (const Combination &ob) const // functor to arrange combinations
    {
        // return (rank_sum <= ob.rank_sum); // this might be causing issues with sort - segfault (larger terms)
        return (rank_sum < ob.rank_sum);
    }
};
struct Matching // currently not being used but can be useful for combination of regrets
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

struct TemporalOrder // struct to save information about a task in an island
{
    int task;
    set<int> predecessors;
    set<int> successors;
    vector<pair<int,int>> predecessor_time;
    int task_time;
};

struct CopySolution // struct to save copy of current solution class methods for use within an island
{
    vector<Path> task_paths_refs;
    vector<Agent> agent_refs;
    vector<vector<int>> task_assign_refs;
    vector<pair<int,int>> precedence_refs;
    map<int, int> conflicted_pathsize_ref;
    unordered_map<int,int> new_distances;

    CopySolution(vector<Path> path_refs, vector<Agent> a_refs, vector<vector<int>> assign_refs, vector<pair<int,int>> p_refs, map<int, int> ct_path_ref)
        : task_paths_refs(path_refs)
        , agent_refs(a_refs)
        , task_assign_refs(assign_refs)
        , precedence_refs(p_refs)
        , conflicted_pathsize_ref(ct_path_ref) 
    {}

    void clear_all()
    {
        task_paths_refs.clear();
        agent_refs.clear();
        task_assign_refs.clear();
        precedence_refs.clear();
        conflicted_pathsize_ref.clear();
        new_distances.clear();
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
    bool combination_conflict_flag; // this is to allow the program to end for exhausted combination list

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
        // PLOGI << "global task inside task assign " << global_task; // delete 
        // PLOGI << "inside the getnagent task func, task assign [9] [4] == " << task_assignments[9][4];// DELET AFTER DEBUG
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
        for (int agent : agents_to_compute) { // for prepnext goes over conflicts, otherwise all agents

            assert(getAssignedTaskSize(agent) ==
                   (int)agents[agent].path_planner->goal_locations.size());
            assert(getAssignedTaskSize(agent) == (int)agents[agent].task_paths.size());

            for (int i = 0; i < getAssignedTaskSize(agent); i++) { // going over each task in sequence
                if (i == 0) // first task
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].front());
                // int task = getAgentGlobalTasks(agent, i);
                // PLOGI <<"Agent path size current " << (int)agents[agent].path.size() - 1 << " Begin time of next task " << agents[agent].task_paths[i].begin_time << " for task " << task; // FIXME: delete
                assert((int)agents[agent].path.size() - 1 ==
                       agents[agent].task_paths[i].begin_time); // assert that each consecutive task has its previous task end time as begin time
                for (int j = 1; j < (int)agents[agent].task_paths[i].size(); j++) // not sure
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].at(j));
                agents[agent].path.timestamps.push_back(agents[agent].path.size() - 1);
            }
        }
    }
    // new online join path function that uses the input as set instead of vector
    void OnlinejoinPaths(set<int> agents_to_compute)
    {
        for (int agent : agents_to_compute) { // going over each agent

            assert(getAssignedTaskSize(agent) ==
                   (int)agents[agent].path_planner->goal_locations.size());
            assert(getAssignedTaskSize(agent) == (int)agents[agent].task_paths.size());

            for (int i = 0; i < getAssignedTaskSize(agent); i++) { // going over each task in sequence
                if (i == 0) // first task
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].front());
                assert((int)agents[agent].path.size() - 1 ==
                       agents[agent].task_paths[i].begin_time);
                for (int j = 1; j < (int)agents[agent].task_paths[i].size(); j++) // joining all the paths for the task
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].at(j));
                agents[agent].path.timestamps.push_back(agents[agent].path.size() - 1);
            }
        }
    }

    vector<set<int>> getConflictIslands(const Instance* instance)
    {
        /*
            Inputs:
                1. const instance class object
            Outputs:
                1. vector of set of integers

            Description: The function assists in creation of task islands, where each task within an island
                is temporally related to another task. It does so by looping over the current conflict
                set, and tries to find if any other task is a successor or a predecessor of it. 
                E.g. 3 -> 41 -> 15 -> 12 is a task order. If 3 and 12 are in conflict set, then they will
                make an island together.
        */


        /*
        A. Setup for the for loop
        */
        // get successor to ancestor list
        unordered_map<int, vector<int>> task_depen = instance->getTaskDependencies();
        // get ancestor to successor list
        unordered_map<int, vector<int>> ancestor_depen = instance->getAncestorToSuccessorDependencies();
        set<int> local_ct_copy = neighbor.conflicted_tasks;
        // new vector to save islands
        vector<set<int>> islands; // can also call this from main loop so we can save prev iteration islands

        /*
        B. For loop for looping over conflict set
        */
        for(auto it: neighbor.conflicted_tasks)
        {   
            /*
            B1. If islands is not initiated, then this task will make a "new" island
            */
            if (islands.empty())
            {
                // B1.1 Add the task to a new island
                set<int> local_set;
                local_set.insert(it);

                // B1.2 remove the task from local ct copy, since we already made a new island no need to keep it active for others
                local_ct_copy.erase(it);

                // B1.3 find any other tasks in the conflict set that are temporally related to it
                vector<int> found;
                for (auto in: local_ct_copy)
                {
                    // find in its ancestors
                    
                    if (task_depen.find(it) != task_depen.end())
                    {
                        vector<int> ancestors = task_depen[it];
                        for (int a: ancestors)
                        {
                            if (a == in)
                            {
                                found.push_back(in);
                            }
                        }
                    }
                    // find in its successors
                    if (ancestor_depen.find(it) != ancestor_depen.end())
                    {
                        vector<int> successors = ancestor_depen[it];
                        for(int s: successors)
                        {
                            if (s == in)
                            {
                                found.push_back(in);
                            }
                        }
                    }
                }

                // B1.4 for each dependency found above remove from local_ct
                
                for(auto f: found)
                {
                    local_set.insert(f);
                    local_ct_copy.erase(f);
                }
                // B1.5 push local set to islands
                islands.push_back(local_set);
            }
            else
            {
                /*
                B2. means islands already exist at the turn of this task, so check if it already exists
                    in an island or if it belongs to some island
                */

                // B2.1 if yes means its already a part of island
                if (local_ct_copy.find(it) == local_ct_copy.end())
                {
                    continue;
                }
                else
                {
                    // B2.2 if not then needs to form a new island or join an existing island!
                    bool exist_flag = false;

                    // B2.3 first check existing islands to see if it can be added to an island
                    for(auto ild = islands.begin(); ild != islands.end(); ild++) // isld  = each island
                    {
                        set<int> isld = islands[distance(islands.begin(), ild)]; // C++ dont allow to update sets
                        islands.erase(ild); // removing the current set from the vector
                        set<int> local_set = isld; // creating new local copy of set
                        for(int t : isld) // t = each existing task
                        {
                            if (task_depen.find(it) != task_depen.end())
                            {
                                vector<int> ancestors = task_depen[it];
                                for (int a: ancestors)
                                {
                                    if (a == t)
                                    {
                                        // isld.insert(it);  // this doesn't work for sets
                                        local_set.insert(local_set.end(), it);
                                        local_ct_copy.erase(it);
                                        exist_flag = true;
                                    }
                                }
                            }
                            // find task in ancestors
                            if (!exist_flag){
                                if (ancestor_depen.find(it) != ancestor_depen.end())
                                {
                                    vector<int> successors = ancestor_depen[it];
                                    for(int s: successors)
                                    {
                                        if (s == t)
                                        {
                                            local_set.insert(local_set.end(), it);
                                            local_ct_copy.erase(it);
                                            exist_flag = true;
                                        }
                                    }
                                }
                            }
                        }
                        islands.insert(ild, local_set); // pushing the local copy of the set back to the vector
                    }

                    // B2.4 If not means we have to make a new island for the task
                    // operations similar to B1
                    if (!exist_flag)
                    {
                        // make a new island
                        set<int> local_set;
                        local_set.insert(it);
                        local_ct_copy.erase(it);
                        //for loop over local ct copy and same steps as above
                        vector<int> found;
                        for (auto in: local_ct_copy)
                        {
                            // find task in successors
                            
                            if (task_depen.find(it) != task_depen.end())
                            {
                                vector<int> ancestors = task_depen[it];
                                for (int a: ancestors)
                                {
                                    if (a == in)
                                    {
                                        found.push_back(in);
                                    }
                                }
                            }
                            // find task in ancestors
                            if (ancestor_depen.find(it) != ancestor_depen.end())
                            {
                                vector<int> successors = ancestor_depen[it];
                                for(int s: successors)
                                {
                                    if (s == in)
                                    {
                                        found.push_back(in);
                                    }
                                }
                            }
                        }
                        // for each dependency found above remove from local_ct
                        // push local set to islands
                        for(auto f: found)
                        {
                            local_set.insert(f);
                            local_ct_copy.erase(f);
                        }
                        islands.push_back(local_set); 
                    }// end
                }
            }

        }

        // ordering can be done HERE, skip for now

        return islands;
    }

    pair<vector<int>, vector<TemporalOrder>> getIslandOrder(set<int> island, const Instance* instance)
    {
        /*
            Inputs: 
                1. island set
                2. const instance class object
            Outputs:
                2. A pair of two vectors
                    vector 1 - ordered list of tasks
                    vector 2 - unordered list of tasks with important information
            
            Description: This function assists in determining any temporal order between the tasks within an island.
                It also saves some important information about the tasks which is used in LNS loop
        */

        /* 
        A. Setup the nodes for the tasks
        */
        vector<TemporalOrder> local;
        // get list of ancestors for each task
        unordered_map<int, vector<int>> successor_depen = instance->getTaskDependencies();
        // get list of successors for each task
        unordered_map<int, vector<int>> ancestor_depen = instance->getAncestorToSuccessorDependencies();

        /*
        B. For loop to go over each task in the island
        */
        for(int curr : island)
        {
            /*
            B1. Make its temporal struct
            */
            TemporalOrder o;
            o.task = curr;

            /*
            B2. We only care about tasks that have a temporal relation
            */

            // B2.1 check if it has any ancestors
            if(successor_depen.find(o.task) != successor_depen.end())
            {
                vector<int> ancestors = successor_depen[o.task];
                // For each of its ancestors check if they are part of the conflict set
                for(int anc: ancestors)
                {
                    if(neighbor.conflicted_tasks.find(anc) != neighbor.conflicted_tasks.end())
                    {
                        o.predecessors.insert(anc); // append the ancestor to its temporal order predecessor set
                    }
                }
            }

            // B2.2 check if it has any successors
            if(ancestor_depen.find(o.task) != ancestor_depen.end())
            {
                vector<int> childs = ancestor_depen[o.task];
                // For each of its successors check if they are part of the conflict set
                for(int succ : childs)
                {
                    if(neighbor.conflicted_tasks.find(succ) != neighbor.conflicted_tasks.end())
                    {
                        o.successors.insert(succ); // append the successor to its temporal order successors set
                    }
                }
            }

            /*
            B3. Finally push this temporal order task to a vector
            */
            local.push_back(o);
        }

        /*
        C. Temporal sort or topological sort section
        */

        /*
        C1. Setup
        */
        vector<int> closed;
        set<int> expanded;
        std::queue<int> open;

        /*
        C2. Find the children and add them to queue first
        */ 
        for(auto o : local)
        {
            if (o.successors.size() == 0)
            {
                open.push(o.task);
                expanded.insert(o.task);
            }
        }
        
        /*
        C3. Queue loop for temporal sort
        */
        while(!open.empty())
        {

            // pop the element from queue
            int curr = open.front();
            
            // get its parents and children
            set<int> predecessors; // vector<int> predecessors
            set<int> successors; // vector<int> successors
            for(auto o : local)
            {
                if (curr == o.task)
                {
                    predecessors = o.predecessors;
                    successors = o.successors;
                    break;
                }
            }

            // are the parents in expanded list?
            for(int p: predecessors)
            {
                // if not add them to expaneded list and them to the queue
                if(expanded.find(p) == expanded.end())
                {
                    expanded.insert(p);
                    open.push(p);
                }

            }
            // move onto children
            bool add_child = false;
            for(int c: successors)
            {
                bool close = false;    
                // if yes, are they all in closed?
                for(int l : closed)
                {
                    if (l == c)
                    {
                        close = true;
                        break;
                    }
                }
                if (close)
                    continue;
                
                // if not in closed
                // are they in expanded list?
                bool expand = false;
                // if not - add them to expanded list and to the queue
                if(expand || (!close))
                {
                    open.push(c);
                    expanded.insert(c); // will this ever happen?
                    add_child = true;
                }
                // pop the task and add it back to queue
                // if all in closed then
            }
            // move task to closed and pop
            if (add_child)
            {
                open.pop();
                open.push(curr);
            }
            else{
                open.pop();
                closed.push_back(curr);
            }

            // move to next
        }

        /*
        D. Reverse the obtained temporal order to: predecessors to successors
        */
        std::reverse(closed.begin(), closed.end());

        return make_pair(closed, local);
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
    const string& combo_flag; // adding combo flag as part of protected class

  public:
    double runtime = 0;
    vector<Path> initial_paths;
    list<IterationStats> iteration_stats;
    int num_of_failures = 0, sum_of_costs = 0;

    LNS(int num_of_iterations, const Instance& instance, int neighbor_size, double time_limit, const string& combo_flag);

    inline Instance getInstance() { return instance; }

    bool run();
    void prepareNextIteration();
    void OnlineprepareNextIteration(set<int> new_conflict_tasks, set<int> conflicted_tasks);
    void IslandprepareNextIteration();
    void printPaths() const;
    bool validateSolution(set<int>* conflicted_tasks = nullptr);
    bool OnlinevalidateSolution(set<int>* conflicted_tasks = nullptr);
    void build_constraint_table(ConstraintTable& constraint_table, int task);

    void build_constraint_table(ConstraintTable& constraint_table,
                                int task,
                                int task_location,
                                vector<Path>* paths,
                                vector<vector<int>>* task_assignments,
                                vector<pair<int, int>>* precedence_constraints);
    void Onlinebuild_constraint_table(ConstraintTable& constraint_table,
                                int task,
                                int task_location,
                                vector<Path>* paths,
                                vector<vector<int>>* task_assignments,
                                vector<pair<int, int>>* precedence_constraints);
    void Online_prep_build_constraint_table(ConstraintTable& constraint_table, int task, set<int> new_conflict_tasks, set<int> conflicted_tasks);
    void computeRegret();
    void OnlinecomputeRegret(int task, int earlyT, CopySolution* cp_soln, unordered_map<int, vector<pair<int,int>>> cancelled_positions);
    void regretBasedReinsertion();
    void computeRegretForMetaTask(deque<int> meta_task);
    void computeRegretForTask(int task);
    void OnlinecomputeRegretForTask(int task, int earlyT, CopySolution* cp_soln, unordered_map<int, vector<pair<int,int>>> cancelled_positions);
    void commitBestRegretTask(Regret best_regret);
    void computeRegretForTaskWithAgent(
      int task,
      int agent,
      int earliest_timestep,
      int latest_timestep,
      vector<pair<int, int>>* precedence_constraints,
      pairing_heap<Utility, compare<Utility::compare_node>>* service_times);
    void OnlinecomputeRegretForTaskWithAgent(
      int task,
      int agent,
      int earliest_timestep,
      int latest_timestep,
      vector<pair<int, int>>* precedence_constraints,
      pairing_heap<Utility, compare<Utility::compare_node>>* service_times,
      CopySolution* cp_soln,
      unordered_map<int, vector<pair<int,int>>> cancelled_positions,
      vector<pair<pair<int,int>,int>>* distances);
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
