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
    // ~Agent() { delete path_planner; }
};

struct Utility
{
    int agent, task_position, task, endtime; // adding task capability since not using Regret
    double value;
    Path saved_path_task;

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
      , endtime(time)
      , value(value)
      , saved_path_task(temp)
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
    int task, agent, task_position, endtime;
    double value;
    Path saved_path_task;

    Regret(int task, int agent, int task_position, int time, double value, Path temp)
      : task(task)
      , agent(agent)
      , task_position(task_position)
      , endtime(time)
      , value(value)
      , saved_path_task(temp)
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
        // return (rank_sum <= ob.rank_sum); // this might be causing issues with sort - segfault (larger terms)
        return (rank_sum < ob.rank_sum);
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

struct TemporalOrder
{
    int task;
    set<int> predecessors;
    set<int> successors;
    vector<pair<int,int>> predecessor_time;
    int task_time;
};

struct CopySolution
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
                int task = getAgentGlobalTasks(agent, i);
                // PLOGI <<"Agent path size current " << (int)agents[agent].path.size() - 1 << " Begin time of next task " << agents[agent].task_paths[i].begin_time << " for task " << task; // FIXME: delete
                assert((int)agents[agent].path.size() - 1 ==
                       agents[agent].task_paths[i].begin_time); // assert that each consecutive task has its previous task end time as begin time
                for (int j = 1; j < (int)agents[agent].task_paths[i].size(); j++) // not sure
                    agents[agent].path.path.push_back(agents[agent].task_paths[i].at(j));
                agents[agent].path.timestamps.push_back(agents[agent].path.size() - 1);
            }
        }
    }
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
        // how do you find out relations, and then order them within?
        // choose something - go over both task dependencies and task ancestors
        // if you find something add it to its list. then look at the ones in it and do the same
        // start from beginning for the left over conflict tasks
        unordered_map<int, vector<int>> task_depen = instance->getTaskDependencies();
        unordered_map<int, vector<int>> ancestor_depen = instance->getAncestorToSuccessorDependencies();
        set<int> local_ct_copy = neighbor.conflicted_tasks; // will need to remove any that exist from prev iteration?
        vector<set<int>> islands; // can also call this from main loop so we can save prev iteration islands
        for(auto it: neighbor.conflicted_tasks)
        {   
            if (islands.empty())
            {
                // first iteration
                set<int> local_set;
                local_set.insert(it);
                // remove from local ct copy
                local_ct_copy.erase(it);
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
                continue; // don't need it
            }
            else
            {
                // is this not in local_ct_copy? if yes means its already a part of island
                if (local_ct_copy.find(it) == local_ct_copy.end())
                {
                    continue;
                }
                else
                {
                    // if not then needs to form a new island or join an existing island!
                    bool exist_flag = false;
                    // first check existing islands
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

        // for each set find the correct order, SKIP FOR NOW

        return islands;
    }

    pair<vector<int>, vector<TemporalOrder>> getIslandOrder(set<int> island, const Instance* instance)
    {
        // setup the nodes for the tasks
        vector<TemporalOrder> local; // do I want this available outside? (use a pointer?)

        // for(auto t: island)
        // {
        //     TemporalOrder o;
        //     o.task = t;
        //     // for(pair<int,int> pc: precedence_constraints) // TODO: issue with pc, had to use set, can go back to vector with task depen
        //     // {
        //     //     if (pc.first == t)
        //     //     {
        //     //         if(island.find(pc.second) != island.end())
        //     //             o.successors.insert(pc.second);// o.successors.push_back(pc.second);
        //     //     }
        //     //     if (pc.second == t)
        //     //     {
        //     //         if(island.find(pc.first) != island.end())
        //     //             o.predecessors.insert(pc.first);// o.predecessors.push_back(pc.first);
        //     //     }
        //     // }
        //     local.push_back(o);
        // }
        unordered_map<int, vector<int>> successor_depen = instance->getTaskDependencies();
        unordered_map<int, vector<int>> ancestor_depen = instance->getAncestorToSuccessorDependencies();
        for(int curr : island)
        {
            TemporalOrder o;
            o.task = curr;
            if(successor_depen.find(o.task) != successor_depen.end())
            {
                vector<int> ancestors = successor_depen[o.task];
                for(int anc: ancestors)
                {
                    if(neighbor.conflicted_tasks.find(anc) != neighbor.conflicted_tasks.end())
                    {
                        o.predecessors.insert(anc);
                    }
                }
            }
            if(ancestor_depen.find(o.task) != ancestor_depen.end())
            {
                vector<int> childs = ancestor_depen[o.task];
                for(int succ : childs)
                {
                    if(neighbor.conflicted_tasks.find(succ) != neighbor.conflicted_tasks.end())
                    {
                        o.successors.insert(succ);
                    }
                }
            }
            local.push_back(o);
        }

        // topological sort section
        vector<int> closed;
        set<int> expanded;
        std::queue<int> open;

        // find the children and add them to queue
        for(auto o : local)
        {
            if (o.successors.size() == 0)
            {
                open.push(o.task);
                expanded.insert(o.task);
            }
        }
        
        // topological sort loop
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

    bool ComboTemporalChecker(const Instance* instance, Solution* solution)
    {
        unordered_map<int, vector<int>> tasks_depen;
        tasks_depen = instance->getTaskDependencies(); // this is giving wrong tasks dependencies
        set<int> local_conflict_check = solution->neighbor.conflicted_tasks;
        for(pair<int, vector<int>> dependency : instance->getTaskDependencies())
        {
            // int child_task = distance(tasks_depen.begin(), it);
            int child_task = dependency.first;
            if (local_conflict_check.find(child_task) == local_conflict_check.end()) // we dont want these in conflict set
            {
                int child_agent = solution->getAgentWithTask(child_task);
                int child_index = solution->getLocalTaskIndex(child_agent, child_task);
                int child_timestamp = solution->agents[child_agent].task_paths[child_index].end_time();
                vector<int> child_ancestors = dependency.second;
                for(int ancestor_task: child_ancestors)
                {
                    if (local_conflict_check.find(ancestor_task) == local_conflict_check.end()) // we don't want these in conflict set
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
    // TODO - THIS NEEDS to accept a different set of tasks that are AFFECTED by the placement of conflict tasks
    // IT SHOULD BE A PAIR OF INT AND PATH, pair<int, Path>
    bool ComboTemporalChecker2(vector<Path>* task_path_refs, set<int> affected_ref)
    {
        unordered_map<int, vector<int>> successors = instance.getAncestorToSuccessorDependencies();
        unordered_map<int, vector<int>> ancestors = instance.getTaskDependencies();
        for(auto a: affected_ref)
        {
            vector<int> current_succesors = successors[a];
            int task_timestamp = task_path_refs->at(a).end_time();
            for(int s: current_succesors)
            {
                // PLOGI<< "Affected Task = " << a << " Successor = " << s;
                int successor_timestamp = task_path_refs->at(s).end_time();
                // PLOGI <<" Successor task = " << s <<" Successor end timestamp = "<< successor_timestamp;
                // PLOGI <<" Ancestor task = "<< a <<" Ancestor end timestamp = "<< task_timestamp;
                if(successor_timestamp <= task_timestamp)
                {
                    // PLOGI <<" Regret for Task=" << task << " in Agent = "<< agent << " by pushing local task= " << next_task << " out";
                    PLOGI <<" Found a TEMPORAL VIOLATION, successor task = " << s << " time = " << successor_timestamp << " ancestor task = " << a << " time = " << task_timestamp;
                    // value = INT_MIN;
                    return true; // temporal issue found so move on to next task
                }
            }

            vector<int> current_ancestors = ancestors[a];
            for(int an: current_ancestors)
            {
                // PLOGI <<"Affected Task = " << a <<" Ancestor = " << an;
                int ancestor_timestamp = task_path_refs->at(an).end_time();
                // PLOGI <<" Ancestor task = " << an <<" Ancestor end timestamp = "<< ancestor_timestamp;
                // PLOGI <<" Succesor task = "<< a <<" Successor end timestamp = "<< task_timestamp;
                if(task_timestamp <= ancestor_timestamp)
                {
                    // PLOGI <<" Regret for Task=" << task << " in Agent = "<< agent << " by pushing local task= " << next_task << " out";
                    PLOGI <<" Found a TEMPORAL VIOLATION, successor task = " << a <<" time = " << task_timestamp << " ancestor task = " << an <<" time = "<< ancestor_timestamp;
                    // value = INT_MIN;
                    return true; // temporal issue found so move on to next task                   
                }
            }


        }
        return false; // means no temporal issue will exist
    }

    bool ComboTemporalChecker3(vector<Path>* task_path_refs)
    {
        unordered_map<int, vector<int>> successors = instance.getAncestorToSuccessorDependencies();
        unordered_map<int, vector<int>> ancestors = instance.getTaskDependencies();
        for(auto it = task_path_refs->begin(); it != task_path_refs->end(); it++)
        {
            int task = distance(task_path_refs->begin(), it);
            int task_time = task_path_refs->at(task).end_time();
            if(solution.neighbor.conflicted_tasks.find(task) == solution.neighbor.conflicted_tasks.end())
            {
                if(successors.find(task) != successors.end())
                {
                    vector<int> current_succ = successors[task];
                    for(int succ: current_succ)
                    {
                        int succ_time = task_path_refs->at(succ).end_time();
                        PLOGW << task_time <<" " << succ_time;
                        if (succ_time <= task_time)
                        {
                            PLOGE <<" Found a TEMPORAL VIOLATION, successor task = " << succ << " time = " << succ_time << " ancestor task = " << task << " time = " << task_time;
                            return true;
                        }
                    }
                }
                if(ancestors.find(task) != ancestors.end())
                {
                    vector<int> current_anc = ancestors[task];
                    for(int anc: current_anc)
                    {
                        int anc_time = task_path_refs->at(anc).end_time();
                        PLOGW << anc_time << " " << task_time;
                        if (task_time <= anc_time)
                        {
                            PLOGE <<" Found a TEMPORAL VIOLATION, successor task = " << task << " time = " << task_time << " ancestor task = " << anc << " time = " << anc_time;
                            return true;
                        }
                    }                   
                }
            }
        }
        return false; // means no temporal issue will exist
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

        bool ComboCostChecker2(vector<Path>* task_path_refs)
    {
        int Tcosts = 0;
        int compCosts = 0;
        for (int taskz = 0; taskz < instance.getTasksNum(); taskz++) {
            Tcosts += (task_path_refs->at(taskz).end_time() - task_path_refs->at(taskz).begin_time);
        }
        for (int a = 0; a < instance.getAgentNum(); a++)
        {
            compCosts += solution.agents[a].path.end_time(); // technically would be lower because we haven't updated solution yet
        }
        PLOGI << "Combination total cost = " << Tcosts;
        PLOGI << "Comparative total cost (path end time) =  "<< compCosts;
        PLOGI << "Previous solution cost = " << previous_solution.sum_of_costs;
        if (previous_solution.sum_of_costs < Tcosts)
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
