#include "lns.hpp"
#include "utils.hpp"

bool
isSamePath(const Path& p1, const Path& p2)
{
    if (p1.size() != p2.size())
        return false;
    for (int i = 0; i < (int)p1.size(); i++)
        if (p1.path[i].location != p2.path[i].location)
            return false;
    return true;
}

LNS::LNS(int num_of_iterations, const Instance& instance, int neighbor_size, double time_limit)
  : num_of_iterations(num_of_iterations)
  , instance(instance)
  , neighbor_size(neighbor_size)
  , solution(instance)
  , previous_solution(instance)
  , time_limit(time_limit)
{
    planner_start_time = Time::now();

    ALNS = true;
    decay_factor = 0.01;
    reaction_factor = 0.01;
    replan_time_limit = time_limit / 100;
    // Assigning 1 to the only destroy heuristic that we are using here
    destroy_weights.push_back(1);

    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);
    for (int i = 0; i < N; i++)
        agents.emplace_back(instance, i);
    preprocessing_time = ((fsec)(Time::now() - planner_start_time)).count();
    PLOGD << "Pre-processing time = " << preprocessing_time << " seconds.\n";
}

void
LNS::joinPaths(vector<int> agents_to_compute)
{
    for (int i = 0; i < instance.getAgentNum(); i++) {
        if (std::find(agents_to_compute.begin(), agents_to_compute.end(), i) !=
            agents_to_compute.end()) {
            for (int j = 0; j < (int)agents[i].path_planner->goal_locations.size(); j++) {
                if (j == 0)
                    agents[i].path.path.push_back(agents[i].task_paths[j].front());
                assert((int)agents[i].path.size() - 1 == agents[i].task_paths[j].begin_time);
                for (int k = 1; k < (int)agents[i].task_paths[j].size(); k++)
                    agents[i].path.path.push_back(agents[i].task_paths[j].at(k));
                agents[i].path.timestamps.push_back(agents[i].path.size() - 1);
            }
        }
    }
}

bool
LNS::run()
{
    planner_start_time = Time::now();
    // assign tasks
    greedy_task_assignment(&instance, &solution);
    for (int i = 0; i < instance.getAgentNum(); i++) {
        if ((int)solution.getAgentGlobalTasks(i).size() == 0) {
            vector<int> goal = { instance.getStartLocations()[i] };
            agents[i].path_planner->setGoalLocations(goal);
            agents[i].task_paths.resize(1, Path());
        } else {
            vector<int> task_assignments = solution.getAgentGlobalTasks(i);
            vector<int> task_locations = instance.getTaskLocations(task_assignments);
            agents[i].path_planner->setGoalLocations(task_locations);
            agents[i].task_paths.resize((int)task_locations.size(), Path());
        }
        agents[i].path_planner->compute_heuristics();
    }

    for (int i = 0; i < instance.getAgentNum(); i++)
        for (int j = 1; j < (int)solution.task_assignments[i].size(); j++)
            solution.insertPrecedenceConstraint(solution.task_assignments[i][j - 1],
                                                solution.task_assignments[i][j]);

    for (int i = 0; i < instance.getTasksNum(); i++) {
        vector<int> previous_tasks = instance.getTaskDependencies()[i];
        for (int j = 0; j < (int)previous_tasks.size(); j++) {
            solution.insertPrecedenceConstraint(previous_tasks[j], i);
        }
    }

    // find paths based on the task assignments
    vector<int> planning_order;
    bool success = topological_sort(&instance, &solution.precedence_constraints, planning_order);
    if (!success) {
        PLOGE << "Topological sorting failed\n";
        return success;
    }

    solution.paths.resize(instance.getTasksNum(), Path());
    initial_paths.resize(instance.getTasksNum(), Path());
    for (int id : planning_order) {
        int agent = solution.getAgentWithTask(id), task = id,
            task_position = solution.getLocalTaskIndex(agent, task), start_time = 0;
        if (task_position != 0) {
            int previous_task = solution.task_assignments[agent][task_position - 1];
            assert(!initial_paths[previous_task].empty());
            start_time = initial_paths[previous_task].end_time();
        }
        PLOGI << "Planning for agent " << agent << " and task " << task << endl;
        ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
        build_constraint_table(constraint_table, task);

        initial_paths[id] = agents[agent].path_planner->findPathSegment(
          constraint_table, start_time, task_position, 0);
        if (initial_paths[id].empty()) {
            PLOGE << "No path exists for agent " << agent << " and task " << task << endl;
            return false;
        }

        agents[agent].task_paths[task_position] = initial_paths[id];
        solution.paths[id] = initial_paths[id];
    }

    vector<int> agents_to_compute(instance.getAgentNum());
    std::iota(agents_to_compute.begin(), agents_to_compute.end(), 0);
    joinPaths(agents_to_compute);

    int initial_sum_of_costs = 0;
    for (int i = 0; i < (int)agents.size(); i++) {
        initial_sum_of_costs += agents[i].path.end_time();
    }
    sum_of_costs = initial_sum_of_costs;
    solution.sum_of_costs = sum_of_costs;

    printPaths();

    initial_solution_runtime = ((fsec)(Time::now() - planner_start_time)).count();
    iteration_stats.emplace_back(initial_solution_runtime,
                                 "greedy",
                                 instance.getAgentNum(),
                                 instance.getTasksNum(),
                                 initial_sum_of_costs);
    runtime = initial_solution_runtime;

    PLOGD << "Initial solution cost = " << initial_sum_of_costs
          << ", Runtime = " << initial_solution_runtime << endl;

    while (runtime < time_limit && (int)iteration_stats.size() <= num_of_iterations) {
        runtime = ((fsec)(Time::now() - planner_start_time)).count();
        set<int> conflicted_tasks;
        bool valid = validateSolution(&conflicted_tasks);
        solution.neighbor.conflicted_tasks = conflicted_tasks;
        if (valid) {
            PLOGV << "Solution was found!\n";
            break;
        }
        if (!valid) {

            PLOGE << "The initial solution was not valid!\n";

            previous_solution = solution;

            prepareNextIteration();

            // Compute regret for each of the tasks that are in the conflicting set
            // Pick the best one and repeat the whole process aboe
            while (!solution.neighbor.conflicted_tasks.empty()) {
                computeRegret();
                Regret best_regret = solution.neighbor.regret_max_heap.top();
                // Use the best regret task and insert it in its correct location
                commitBestRegretTask(best_regret);
            }

            for (int i = 0; i < instance.getAgentNum(); i++)
                solution.neighbor.agents[i].path = Path();
            vector<int> agents_to_compute(instance.getAgentNum());
            std::iota(agents_to_compute.begin(), agents_to_compute.end(), 0);
            solution.joinPaths(agents_to_compute);

            printPaths(true);

            solution.sum_of_costs = 0;
            for (int i = 0; i < (int)agents.size(); i++) {
                solution.sum_of_costs += solution.neighbor.agents[i].path.end_time();
            }

            conflicted_tasks.clear();
            valid = validateSolution(&conflicted_tasks);
            solution.neighbor.conflicted_tasks = conflicted_tasks;

            if (previous_solution.neighbor.conflicted_tasks.size() >
                solution.neighbor.conflicted_tasks.size()) {
                // accept this solution
                for (int i = 0; i < instance.getAgentNum(); i++) {
                    agents[i].path = solution.neighbor.agents[i].path;
                    agents[i].task_paths = solution.neighbor.agents[i].task_paths;
                    agents[i].path_planner->setGoalLocations(
                      solution.neighbor.agents[i].path_planner->goal_locations);
                    agents[i].path_planner->compute_heuristics();
                }
            } else if (previous_solution.neighbor.conflicted_tasks.size() ==
                       solution.neighbor.conflicted_tasks.size()) {
                if (previous_solution.sum_of_costs >= solution.sum_of_costs) {
                    // accept this solution
                    for (int i = 0; i < instance.getAgentNum(); i++) {
                        agents[i].path = solution.neighbor.agents[i].path;
                        agents[i].task_paths = solution.neighbor.agents[i].task_paths;
                        agents[i].path_planner->setGoalLocations(
                          solution.neighbor.agents[i].path_planner->goal_locations);
                        agents[i].path_planner->compute_heuristics();
                    }
                }
            }

            PLOGD << "Old sum of costs = " << previous_solution.sum_of_costs << endl;
            PLOGD << "New sum of costs = " << solution.sum_of_costs << endl;
            // Accept the solution only if the new one has lower number of conflicts or it has lower
            // cost of the solution

            assert(false);
        }
    }

    PLOGV << "MAPF-PC-LNS: "
          << "\n\tRuntime = " << runtime << "\n\tIterations = " << iteration_stats.size()
          << "\n\tSolution Cost = " << sum_of_costs
          << "\n\tInitial Solution Cost = " << initial_sum_of_costs
          << "\n\tNumber of failures = " << num_of_failures << endl;

    // change this later
    return true;
}

void
LNS::prepareNextIteration()
{

    PLOGI << "Preparing the solution object for next iteration\n";

    unordered_map<int, int> affected_agents, global_to_local_task_id;
    vector<int> tasks_to_fix;
    for (int ct : solution.neighbor.conflicted_tasks) {
        int agent = solution.getAgentWithTask(ct),
            ct_position = solution.getLocalTaskIndex(agent, ct),
            path_size = (int)solution.paths[ct].size();
        if (ct_position != solution.getAssignedTaskSize(agent) - 1) {
            tasks_to_fix.push_back(solution.getAgentGlobalTasks(agent)[ct_position + 1]);
            path_size +=
              solution.paths[solution.getAgentGlobalTasks(agent)[ct_position + 1]].size();
        }
        solution.neighbor.conflicted_tasks_path_size.insert(make_pair(ct, path_size));
        solution.paths[ct] = Path();
        affected_agents.insert(make_pair(ct, agent));
        solution.clearInterAgentPrecedenceConstraint(ct);
        // needs to happen after clearing precedence constraints
        solution.task_assignments[agent][ct_position] = -1;
        solution.neighbor.agents[agent].task_paths[ct_position] = Path();
        global_to_local_task_id.insert(make_pair(ct, ct_position));
    }

    vector<int> planning_order;
    assert(topological_sort(&instance, &solution.precedence_constraints, planning_order));

    for (pair<int, int> task_agent : affected_agents) {
        int agent = task_agent.second;
        solution.task_assignments[agent].erase(
          std::remove_if(solution.task_assignments[agent].begin(),
                         solution.task_assignments[agent].end(),
                         [](int task) { return task == -1; }),
          solution.task_assignments[agent].end());
        solution.neighbor.agents[agent].task_paths.erase(
          std::remove_if(solution.neighbor.agents[agent].task_paths.begin(),
                         solution.neighbor.agents[agent].task_paths.end(),
                         [](Path p) { return isSamePath(p, Path()); }),
          solution.neighbor.agents[agent].task_paths.end());
        solution.neighbor.agents[agent].path = Path();
        vector<int> task_assignments = solution.getAgentGlobalTasks(agent);
        vector<int> task_locations = instance.getTaskLocations(task_assignments);
        solution.neighbor.agents[agent].path_planner->setGoalLocations(task_locations);
        solution.neighbor.agents[agent].path_planner->compute_heuristics();
    }

    for (int task : planning_order) {

        if (std::find(tasks_to_fix.begin(), tasks_to_fix.end(), task) != tasks_to_fix.end()) {
            int start_time = 0, agent = solution.getAgentWithTask(task);
            int task_position = solution.getLocalTaskIndex(agent, task);
            if (task_position != 0)
                start_time =
                  solution.neighbor.agents[agent].task_paths[task_position - 1].end_time();
            assert(task_position < solution.getAssignedTaskSize(agent) - 1);
            ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
            build_constraint_table(constraint_table, task);
            solution.paths[task] = solution.neighbor.agents[agent].path_planner->findPathSegment(
              constraint_table, start_time, task_position, 0);
            solution.neighbor.agents[agent].task_paths[task_position] = solution.paths[task];
            // Once the path was found fix the begin times for subsequent tasks of the agent
            for (int k = task_position + 1; k < solution.getAssignedTaskSize(agent); k++)
                solution.neighbor.agents[agent].task_paths[k].begin_time =
                  solution.neighbor.agents[agent].task_paths[k - 1].end_time();
        }
    }

    vector<int> agents_to_compute;
    for (pair<int, int> task_agent : affected_agents)
        agents_to_compute.push_back(task_agent.second);
    solution.joinPaths(agents_to_compute);
}

void
LNS::computeRegret()
{
    solution.neighbor.regret_max_heap.clear();
    for (int task : solution.neighbor.conflicted_tasks)
        computeRegretForTask(task);
}

void
LNS::computeRegretForTask(int task)
{

    pairing_heap<Utility, compare<Utility::compare_node>> service_times;
    vector<pair<int, int>> precedence_constraints;
    // Find the precedence constraints involving the task or any other task that is not in the
    // conflicting set
    for (pair<int, int> pc : solution.precedence_constraints) {
        if (pc.first == task || pc.second == task ||
            (std::find(solution.neighbor.conflicted_tasks.begin(),
                       solution.neighbor.conflicted_tasks.end(),
                       pc.first) == solution.neighbor.conflicted_tasks.end() &&
             std::find(solution.neighbor.conflicted_tasks.begin(),
                       solution.neighbor.conflicted_tasks.end(),
                       pc.second) == solution.neighbor.conflicted_tasks.end()))
            precedence_constraints.push_back(pc);
    }
    for (int agent = 0; agent < instance.getAgentNum(); agent++)
        computeRegretForTaskWithAgent(task, agent, &precedence_constraints, &service_times);

    Utility best_utility = service_times.top();
    service_times.pop();
    Utility second_best_utility = service_times.top();
    Regret regret(task,
                  best_utility.agent,
                  best_utility.task_position,
                  second_best_utility.value - best_utility.value);
    solution.neighbor.regret_max_heap.push(regret);
}

void
LNS::computeRegretForTaskWithAgent(
  int task,
  int agent,
  vector<pair<int, int>>* precedence_constraints,
  pairing_heap<Utility, compare<Utility::compare_node>>* service_times)
{

    int first_valid_position = 0;
    vector<int> agent_tasks = solution.getAgentGlobalTasks(agent);
    vector<int> previous_tasks = instance.getTaskDependencies()[task];
    for (int j = solution.getAssignedTaskSize(agent) - 1; j >= 0; j--) {
        if (std::find(previous_tasks.begin(), previous_tasks.end(), agent_tasks[j]) !=
            previous_tasks.end()) {
            first_valid_position = j + 1;
            break;
        }
    }
    for (int j = first_valid_position; j < solution.getAssignedTaskSize(agent) + 1; j++) {

        int distance = 0;
        if (j > 0 && j < solution.getAssignedTaskSize(agent)) {
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(solution.getAgentGlobalTasks(agent)[j - 1]),
              instance.getTaskLocations(task));
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(task),
              instance.getTaskLocations(solution.getAgentGlobalTasks(agent)[j]));
        } else if (j == 0)
            distance += instance.getManhattanDistance(agents[agent].path_planner->start_location,
                                                      instance.getTaskLocations(task));
        else
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(solution.getAgentGlobalTasks(agent)[j - 1]),
              instance.getTaskLocations(task));

        if (distance > solution.neighbor.conflicted_tasks_path_size[task])
            continue;

        vector<Path> task_paths = solution.paths;
        vector<vector<int>> task_assignments = solution.task_assignments;
        vector<pair<int, int>> prec_constraints = *precedence_constraints;
        pair<bool, Utility> success =
          insertTask(task, agent, j, &task_paths, &task_assignments, &prec_constraints);
        if (success.first)
            service_times->push(success.second);
    }
}

pair<bool, Utility>
LNS::insertTask(int task,
                int agent,
                int task_position,
                vector<Path>* task_paths,
                vector<vector<int>>* task_assignments,
                vector<pair<int, int>>* precedence_constraints,
                bool commit)
{

    int start_time = 0, previous_task = -1, next_task = -1;
    double path_size_change = 0;

    vector<Path>& task_paths_ref = *task_paths;
    vector<vector<int>>& task_assignments_ref = *task_assignments;
    vector<pair<int, int>>& prec_constraints_ref = *precedence_constraints;

    int agent_tasks_size = (int)task_assignments_ref[agent].size();

    if (task_position > 0 && task_position < agent_tasks_size) {
        previous_task = task_assignments_ref[agent][task_position - 1];
        next_task = task_assignments_ref[agent][task_position];
        start_time = task_paths_ref[previous_task].end_time();
        path_size_change = (double)task_paths_ref[next_task].size();
        task_paths_ref[next_task] = Path();
        task_assignments_ref[agent].insert(task_assignments_ref[agent].begin() + task_position,
                                           task);
        prec_constraints_ref.erase(std::remove_if(prec_constraints_ref.begin(),
                                                  prec_constraints_ref.end(),
                                                  [previous_task, next_task](pair<int, int> x) {
                                                      return x.first == previous_task &&
                                                             x.second == next_task;
                                                  }),
                                   prec_constraints_ref.end());
        prec_constraints_ref.push_back(make_pair(previous_task, task));
        prec_constraints_ref.push_back(make_pair(task, next_task));

        if (commit) {
            solution.neighbor.agents[agent].task_paths[task_position] = Path();
            solution.neighbor.agents[agent].task_paths.insert(
              solution.neighbor.agents[agent].task_paths.begin() + task_position, Path());
        }

    } else if (task_position == 0) {
        next_task = task_assignments_ref[agent][task_position];
        path_size_change = (double)task_paths_ref[next_task].size();
        task_paths_ref[next_task] = Path();
        task_assignments_ref[agent].insert(task_assignments_ref[agent].begin() + task_position,
                                           task);
        prec_constraints_ref.push_back(make_pair(task, next_task));

        if (commit) {
            solution.neighbor.agents[agent].task_paths[task_position] = Path();
            solution.neighbor.agents[agent].task_paths.insert(
              solution.neighbor.agents[agent].task_paths.begin() + task_position, Path());
        }

    } else if (task_position == agent_tasks_size) {
        previous_task = task_assignments_ref[agent][task_position - 1];
        start_time = task_paths_ref[previous_task].end_time();
        task_assignments_ref[agent].push_back(task);
        prec_constraints_ref.push_back(make_pair(previous_task, task));

        if (commit)
            solution.neighbor.agents[agent].task_paths.push_back(Path());
    }

    bool valid = true;
    if (!commit) {
        vector<int> planning_order;
        valid = topological_sort(&instance, precedence_constraints, planning_order);
        if (!valid)
            return make_pair(valid, Utility());
    }

    vector<int> goal_locations = instance.getTaskLocations(task_assignments_ref[agent]);
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
    if (!commit) {
        MultiLabelSpaceTimeAStar local_planner = MultiLabelSpaceTimeAStar(instance, agent);
        local_planner.setGoalLocations(goal_locations);
        local_planner.compute_heuristics();
        build_constraint_table(constraint_table,
                               task,
                               instance.getTaskLocations()[task],
                               task_paths,
                               task_assignments,
                               precedence_constraints);
        task_paths_ref[task] =
          local_planner.findPathSegment(constraint_table, start_time, task_position, 0);
        if (next_task != -1) {
            start_time = task_paths_ref[task].end_time();
            build_constraint_table(constraint_table,
                                   next_task,
                                   instance.getTaskLocations()[next_task],
                                   task_paths,
                                   task_assignments,
                                   precedence_constraints);
            task_paths_ref[next_task] =
              local_planner.findPathSegment(constraint_table, start_time, task_position + 1, 0);
        }
    } else {
        solution.neighbor.agents[agent].path_planner->setGoalLocations(goal_locations);
        solution.neighbor.agents[agent].path_planner->compute_heuristics();
        build_constraint_table(constraint_table, task);
        task_paths_ref[task] = solution.neighbor.agents[agent].path_planner->findPathSegment(
          constraint_table, start_time, task_position, 0);
        solution.neighbor.agents[agent].task_paths[task_position] = task_paths_ref[task];
        if (next_task != -1) {
            start_time = task_paths_ref[task].end_time();
            build_constraint_table(constraint_table, next_task);
            task_paths_ref[next_task] =
              solution.neighbor.agents[agent].path_planner->findPathSegment(
                constraint_table, start_time, task_position + 1, 0);
            solution.neighbor.agents[agent].task_paths[task_position + 1] =
              task_paths_ref[next_task];
        }
        for (int k = task_position + 1; k < solution.getAssignedTaskSize(agent); k++)
            solution.neighbor.agents[agent].task_paths[k].begin_time =
              solution.neighbor.agents[agent].task_paths[k - 1].end_time();
    }

    if (!commit) {
        double value = -path_size_change + (double)task_paths_ref[task].size();
        if (next_task != -1)
            value += (double)task_paths_ref[next_task].size();

        Utility utility(agent, task_position, value);
        return make_pair(valid, utility);
    } else
        return make_pair(valid, Utility());
}

void
LNS::commitBestRegretTask(Regret best_regret)
{

    pair<bool, Utility> success = insertTask(best_regret.task,
                                             best_regret.agent,
                                             best_regret.task_position,
                                             &solution.paths,
                                             &solution.task_assignments,
                                             &solution.precedence_constraints,
                                             true);
    // The task should be inserted successfully otherwise something is wrong
    assert(success.first);
    solution.neighbor.conflicted_tasks.erase(best_regret.task);
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table,
                            int task,
                            int task_location,
                            vector<Path>* paths,
                            vector<vector<int>>* task_assignments,
                            vector<pair<int, int>>* precedence_constraints)
{
    constraint_table.goal_location = task_location;

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : (*precedence_constraints))
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (set_of_tasks_to_complete.find(current) != set_of_tasks_to_complete.end())
            continue;
        set_of_tasks_to_complete.insert(current);
        for (int agent_task_ancestor : ancestors[current])
            if (set_of_tasks_to_complete.find(agent_task_ancestor) ==
                set_of_tasks_to_complete.end())
                q.push(agent_task_ancestor);
    }
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        assert(!(*paths)[id].empty());
        int task_position = -1, task_set_size = -1;
        for (int i = 0; i < instance.getAgentNum(); i++) {
            for (int j = 0; j < (int)(*task_assignments)[i].size(); j++) {
                if ((*task_assignments)[i][j] == id) {
                    task_position = j;
                    task_set_size = (int)(*task_assignments)[i].size();
                    break;
                }
            }
        }
        bool wait_at_goal = task_position == task_set_size - 1;
        constraint_table.addPath((*paths)[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!(*paths)[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, (*paths)[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table, int task)
{
    constraint_table.goal_location = instance.getTaskLocations(task);

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : solution.precedence_constraints) {
        if (std::find(solution.neighbor.conflicted_tasks.begin(),
                      solution.neighbor.conflicted_tasks.end(),
                      precedence_constraint.first) != solution.neighbor.conflicted_tasks.end() ||
            std::find(solution.neighbor.conflicted_tasks.begin(),
                      solution.neighbor.conflicted_tasks.end(),
                      precedence_constraint.second) != solution.neighbor.conflicted_tasks.end())
            continue;
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);
    }

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (set_of_tasks_to_complete.find(current) != set_of_tasks_to_complete.end())
            continue;
        set_of_tasks_to_complete.insert(current);
        for (int agent_task_ancestor : ancestors[current])
            if (set_of_tasks_to_complete.find(agent_task_ancestor) ==
                set_of_tasks_to_complete.end())
                q.push(agent_task_ancestor);
    }
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        int agent = solution.getAgentWithTask(id);
        int task_position = solution.getLocalTaskIndex(agent, id);
        bool wait_at_goal = task_position == (int)solution.getAssignedTaskSize(agent) - 1;
        constraint_table.addPath(solution.paths[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!solution.paths[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, solution.paths[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

bool
LNS::validateSolution(set<int>* conflicted_tasks)
{

    bool result = true;

    // Check that the precedence constraints are not violated
    vector<pair<int, int>> precedence_constraints = solution.precedence_constraints;
    for (pair<int, int> precedence_constraint : precedence_constraints) {
        if (std::find(solution.neighbor.conflicted_tasks.begin(),
                      solution.neighbor.conflicted_tasks.end(),
                      precedence_constraint.first) != solution.neighbor.conflicted_tasks.end() ||
            std::find(solution.neighbor.conflicted_tasks.begin(),
                      solution.neighbor.conflicted_tasks.end(),
                      precedence_constraint.second) != solution.neighbor.conflicted_tasks.end())
            continue;
        int agent_a = solution.getAgentWithTask(precedence_constraint.first),
            agent_b = solution.getAgentWithTask(precedence_constraint.second);
        int task_position_a = solution.getLocalTaskIndex(agent_a, precedence_constraint.first),
            task_position_b = solution.getLocalTaskIndex(agent_b, precedence_constraint.second);
        if (agents[agent_a].path.timestamps[task_position_a] >=
            agents[agent_b].path.timestamps[task_position_b]) {
            PLOGE << "Temporal conflict between agent " << agent_a << " doing task "
                  << precedence_constraint.first << " and agent " << agent_b << " doing task "
                  << precedence_constraint.second << endl;
            result = false;
            if (conflicted_tasks == nullptr)
                return false;
            else {
                conflicted_tasks->insert(precedence_constraint.first);
                conflicted_tasks->insert(precedence_constraint.second);
            }
        }
    }

    for (int agent_i = 0; agent_i < instance.getAgentNum(); agent_i++)
        for (int agent_j = 0; agent_j < instance.getAgentNum(); agent_j++) {
            if (agent_i == agent_j)
                continue;
            size_t min_path_length = agents[agent_i].path.size() < agents[agent_j].path.size()
                                       ? agents[agent_i].path.size()
                                       : agents[agent_j].path.size();
            for (int timestep = 0; timestep < (int)min_path_length; timestep++) {
                int location_agent_i = agents[agent_i].path.at(timestep).location;
                int location_agent_j = agents[agent_j].path.at(timestep).location;

                // Check that any two agents are not at the same location at the same timestep
                if (location_agent_i == location_agent_j) {
                    pair<int, int> coord = instance.getCoordinate(location_agent_i);
                    PLOGE << "Agents " << agent_i << " and " << agent_j
                          << " collide with each other at (" << coord.first << ", " << coord.second
                          << ") at timestep " << timestep << endl;
                    result = false;
                    if (conflicted_tasks == nullptr)
                        return false;
                    else {
                        for (int goals = 0;
                             goals < (int)solution.getAgentGlobalTasks(agent_i).size();
                             goals++)
                            if (agents[agent_i].path.timestamps[goals] > timestep) {
                                int task_id = solution.getAgentGlobalTasks(agent_i)[goals];
                                solution.neighbor.conflicted_tasks.insert(task_id);
                                break;
                            }
                        for (int goals = 0;
                             goals < (int)solution.getAgentGlobalTasks(agent_j).size();
                             goals++)
                            if (agents[agent_j].path.timestamps[goals] > timestep) {
                                int task_id = solution.getAgentGlobalTasks(agent_j)[goals];
                                solution.neighbor.conflicted_tasks.insert(task_id);
                                break;
                            }
                    }
                }
                // Check that any two agents are not following the same edge in the opposite
                // direction at the same timestep
                else if (timestep < (int)min_path_length - 1 &&
                         location_agent_i == agents[agent_j].path.at(timestep + 1).location &&
                         location_agent_j == agents[agent_i].path.at(timestep + 1).location) {
                    pair<int, int> coord_i = instance.getCoordinate(location_agent_i),
                                   coord_j = instance.getCoordinate(location_agent_j);
                    PLOGE << "Agents " << agent_i << " and " << agent_j
                          << " collide with each other at (" << coord_i.first << ", "
                          << coord_i.second << ") --> (" << coord_j.first << ", " << coord_j.second
                          << ") at timestep " << timestep << endl;
                    result = false;
                    if (conflicted_tasks == nullptr)
                        return false;
                    else {
                        for (int goals = 0;
                             goals < (int)solution.getAgentGlobalTasks(agent_i).size();
                             goals++)
                            if (agents[agent_i].path.timestamps[goals] > timestep) {
                                int task_id = solution.getAgentGlobalTasks(agent_i)[goals];
                                conflicted_tasks->insert(task_id);
                                break;
                            }
                        for (int goals = 0;
                             goals < (int)solution.getAgentGlobalTasks(agent_j).size();
                             goals++)
                            if (agents[agent_j].path.timestamps[goals] > timestep) {
                                int task_id = solution.getAgentGlobalTasks(agent_j)[goals];
                                conflicted_tasks->insert(task_id);
                                break;
                            }
                    }
                }
            }

            // Check that any two agents are not at the same location at the same timestep where
            // one agent might be waiting already
            if (agents[agent_i].path.size() != agents[agent_j].path.size()) {
                int smaller_path_agent =
                  agents[agent_i].path.size() < agents[agent_j].path.size() ? agent_i : agent_j;
                int larger_path_agent =
                  agents[agent_i].path.size() < agents[agent_j].path.size() ? agent_j : agent_i;
                int last_location_of_smaller_path_agent =
                  agents[smaller_path_agent].path.back().location;
                for (int timestep = (int)min_path_length;
                     timestep < (int)agents[larger_path_agent].path.size();
                     timestep++) {
                    int location_of_larger_path_agent =
                      agents[larger_path_agent].path.at(timestep).location;
                    if (last_location_of_smaller_path_agent == location_of_larger_path_agent) {
                        pair<int, int> coord =
                          instance.getCoordinate(last_location_of_smaller_path_agent);
                        PLOGE << "Agents " << agent_i << " and " << agent_j
                              << " collide with each other at (" << coord.first << ", "
                              << coord.second << ") at timestep " << timestep << endl;
                        result = false;
                        if (conflicted_tasks == nullptr)
                            return false;
                        else {
                            for (int goals = 0;
                                 goals < (int)solution.getAgentGlobalTasks(agent_i).size();
                                 goals++) {
                                if (agents[agent_i].path.timestamps[goals] > timestep) {
                                    int task_id = solution.getAgentGlobalTasks(agent_i)[goals];
                                    conflicted_tasks->insert(task_id);
                                    break;
                                }
                                if (agents[agent_i]
                                      .path
                                      .timestamps[(int)agents[agent_i].path.timestamps.size() - 1] <
                                    timestep) {
                                    int last_task_position =
                                      (int)agents[agent_i].path.timestamps.size() - 1;
                                    int task_id =
                                      solution.getAgentGlobalTasks(agent_i)[last_task_position];
                                    conflicted_tasks->insert(task_id);
                                    break;
                                }
                            }
                            for (int goals = 0;
                                 goals < (int)solution.getAgentGlobalTasks(agent_j).size();
                                 goals++) {
                                if (agents[agent_j].path.timestamps[goals] > timestep) {
                                    int task_id = solution.getAgentGlobalTasks(agent_j)[goals];
                                    conflicted_tasks->insert(task_id);
                                    break;
                                }
                                if (agents[agent_j]
                                      .path
                                      .timestamps[(int)agents[agent_j].path.timestamps.size() - 1] <
                                    timestep) {
                                    int last_task_position =
                                      (int)agents[agent_j].path.timestamps.size() - 1;
                                    int task_id =
                                      solution.getAgentGlobalTasks(agent_j)[last_task_position];
                                    conflicted_tasks->insert(task_id);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    return result;
}

void
LNS::printPaths(bool debug) const
{
    for (int i = 0; i < instance.getAgentNum(); i++) {
        if (debug) {
            cout << "Agent " << i << " (cost = " << solution.neighbor.agents[i].path.size() - 1
                 << "): ";
            cout << "\n\tPaths:\n\t";
            for (int t = 0; t < (int)solution.neighbor.agents[i].path.size(); t++) {
                pair<int, int> coord =
                  instance.getCoordinate(solution.neighbor.agents[i].path.at(t).location);
                cout << "(" << coord.first << ", " << coord.second << ")@" << t;
                if (solution.neighbor.agents[i].path.at(t).is_goal)
                    cout << "*";
                if (i != (int)solution.neighbor.agents[i].path.size() - 1)
                    cout << " -> ";
            }
            cout << endl;
            cout << "\tTimestamps:\n\t";
            for (int j = 0; j < (int)solution.neighbor.agents[i].path.timestamps.size(); j++) {
                pair<int, int> goal_coord = instance.getCoordinate(
                  solution.neighbor.agents[i].path_planner->goal_locations[j]);
                cout << "(" << goal_coord.first << ", " << goal_coord.second << ")@"
                     << solution.neighbor.agents[i].path.timestamps[j];
                if (j != (int)solution.neighbor.agents[i].path.timestamps.size() - 1)
                    cout << " -> ";
            }
            cout << endl;
            cout << "\tTasks:\n\t";
            for (int j = 0; j < solution.getAssignedTaskSize(i); j++) {
                cout << solution.getAgentGlobalTasks(i)[j];
                if (j != solution.getAssignedTaskSize(i) - 1)
                    cout << " -> ";
            }
        } else {
            cout << "Agent " << i << " (cost = " << agents[i].path.size() - 1 << "): ";
            cout << "\n\tPaths:\n\t";
            for (int t = 0; t < (int)agents[i].path.size(); t++) {
                pair<int, int> coord = instance.getCoordinate(agents[i].path.at(t).location);
                cout << "(" << coord.first << ", " << coord.second << ")@" << t;
                if (agents[i].path.at(t).is_goal)
                    cout << "*";
                if (i != (int)agents[i].path.size() - 1)
                    cout << " -> ";
            }
            cout << endl;
            cout << "\tTimestamps:\n\t";
            for (int j = 0; j < (int)agents[i].path.timestamps.size(); j++) {
                pair<int, int> goal_coord =
                  instance.getCoordinate(agents[i].path_planner->goal_locations[j]);
                cout << "(" << goal_coord.first << ", " << goal_coord.second << ")@"
                     << agents[i].path.timestamps[j];
                if (j != (int)agents[i].path.timestamps.size() - 1)
                    cout << " -> ";
            }
        }
        cout << endl;
    }
}
