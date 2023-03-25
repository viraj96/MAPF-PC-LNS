#include "lns.hpp"
#include "utils.hpp"

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
                    agents[i].path.path.push_back(agents[i].task_paths[j]->front());
                assert((int)agents[i].path.size() - 1 == agents[i].task_paths[j]->begin_time);
                for (int k = 1; k < (int)agents[i].task_paths[j]->size(); k++)
                    agents[i].path.path.push_back(agents[i].task_paths[j]->at(k));
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
            agents[i].task_paths.resize(1, nullptr);
        } else {
            vector<int> task_assignments = solution.getAgentGlobalTasks(i);
            vector<int> task_locations = instance.getTaskLocations(task_assignments);
            agents[i].path_planner->setGoalLocations(task_locations);
            agents[i].task_paths.resize((int)task_locations.size(), nullptr);
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
        build_constraint_table(constraint_table, agent, task);

        initial_paths[id] = agents[agent].path_planner->findPathSegment(
          constraint_table, start_time, task_position, 0);
        if (initial_paths[id].empty()) {
            PLOGE << "No path exists for agent " << agent << " and task " << task << endl;
            return false;
        }

        agents[agent].task_paths[task_position] = &initial_paths[id];
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
        bool valid = validateSolution(true);
        if (valid) {
            PLOGV << "Solution was found!\n";
            break;
        }
        if (!valid) {

            PLOGE << "The initial solution was not valid!\n";

            previous_solution = solution;

            prepareNextIteration();
            /* assert(validateSolution()); */

            high_resolution_clock::time_point regret_time_start = Time::now();

            // Compute regret for each of the tasks that are in the conflicting set
            // Pick the best one and repeat the whole process above
            for (int task : solution.neighbor.conflicted_tasks) {
                pairing_heap<Utility, compare<Utility::compare_node>> service_times;
                vector<int> previous_tasks = instance.getTaskDependencies()[task];
                vector<pair<int, int>> precedence_constraints;
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
                /* if (task == 18) { */
                /*     for (pair<int, int> pc : precedence_constraints) */
                /*         cout << pc.first << ", " << pc.second << endl; */
                /*     for (int t : solution.neighbor.conflicted_tasks) */
                /*         cout << t << endl; */
                /*     assert(false); */
                /* } */
                for (int i = 0; i < instance.getAgentNum(); i++) {
                    PLOGD << "Trying agent: " << i << endl;
                    int first_valid_position = 0;
                    vector<int> agent_tasks = solution.getAgentGlobalTasks(i);
                    for (int j = (int)agent_tasks.size() - 1; j >= 0; j--) {
                        if (std::find(previous_tasks.begin(),
                                      previous_tasks.end(),
                                      agent_tasks[j]) != previous_tasks.end()) {
                            first_valid_position = j + 1;
                            break;
                        }
                    }
                    for (int j = first_valid_position; j < (int)agent_tasks.size() + 1; j++) {
                        int start_time = 0, previous_task = -1, next_task = -1;
                        double path_size_change = 0;
                        vector<Path> task_paths = solution.paths;
                        vector<vector<int>> task_assignments = solution.task_assignments;
                        vector<pair<int, int>> prec_constraints = precedence_constraints;
                        if (j != 0 && j != (int)agent_tasks.size()) {
                            start_time = task_paths[agent_tasks[j - 1]].end_time();
                            previous_task = agent_tasks[j - 1];
                            next_task = agent_tasks[j];
                            path_size_change = (double)task_paths[next_task].size();
                            task_paths[next_task] = Path();
                            task_assignments[i].insert(task_assignments[i].begin() + j, task);
                            prec_constraints.erase(
                              std::remove_if(prec_constraints.begin(),
                                             prec_constraints.end(),
                                             [previous_task, next_task](pair<int, int> x) {
                                                 return x.first == previous_task &&
                                                        x.second == next_task;
                                             }),
                              prec_constraints.end());
                            prec_constraints.push_back(make_pair(previous_task, task));
                            prec_constraints.push_back(make_pair(task, next_task));
                        } else if (j == 0) {
                            next_task = agent_tasks[j];
                            path_size_change = (double)task_paths[next_task].size();
                            task_paths[next_task] = Path();
                            task_assignments[i].insert(task_assignments[i].begin() + j, task);
                            prec_constraints.push_back(make_pair(task, next_task));
                        } else if (j == (int)agent_tasks.size()) {
                            previous_task = agent_tasks[j - 1];
                            start_time = task_paths[previous_task].end_time();
                            task_assignments[i].push_back(task);
                            prec_constraints.push_back(make_pair(previous_task, task));
                        }

                        vector<int> planning_order;
                        bool valid = topological_sort(&instance, &prec_constraints, planning_order);
                        if (!valid)
                            continue;

                        MultiLabelSpaceTimeAStar local_planner =
                          MultiLabelSpaceTimeAStar(instance, i);
                        vector<int> goal_locations = instance.getTaskLocations(task_assignments[i]);
                        local_planner.setGoalLocations(goal_locations);
                        local_planner.compute_heuristics();

                        ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);

                        build_constraint_table(constraint_table,
                                               task,
                                               instance.getTaskLocations()[task],
                                               &task_paths,
                                               &task_assignments,
                                               &prec_constraints);
                        task_paths[task] =
                          local_planner.findPathSegment(constraint_table, start_time, j, 0);

                        if (next_task != -1) {
                            start_time = task_paths[task].end_time();
                            build_constraint_table(constraint_table,
                                                   next_task,
                                                   instance.getTaskLocations()[next_task],
                                                   &task_paths,
                                                   &task_assignments,
                                                   &prec_constraints);
                            task_paths[next_task] =
                              local_planner.findPathSegment(constraint_table, start_time, j + 1, 0);
                        }

                        double value =
                          sum_of_costs - path_size_change + (double)task_paths[task].size();
                        if (next_task != -1)
                            value += (double)task_paths[next_task].size();

                        /* PLOGD << "Planned for conflicting task: " << task << endl; */
                        /* PLOGD << "Utility was: " << value << endl; */
                        Utility utility(i, j, value);
                        service_times.push(utility);
                    }
                }

                Utility best_utility = service_times.top();
                service_times.pop();
                Utility second_best_utility = service_times.top();
                Regret regret(task,
                              best_utility.agent,
                              best_utility.task_position,
                              second_best_utility.value - best_utility.value);
                solution.neighbor.regret_max_heap.push(regret);
            }

            Regret best_regret = solution.neighbor.regret_max_heap.top();
            PLOGD << best_regret.task << ", " << best_regret.agent << ", "
                  << best_regret.task_position << ", " << best_regret.value << endl;

            double regret_time_end = ((fsec)(Time::now() - regret_time_start)).count();
            PLOGD << "Regret time = " << regret_time_end << endl;

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
    for (int ct : solution.neighbor.conflicted_tasks) {
        int agent = solution.getAgentWithTask(ct),
            ct_position = solution.getLocalTaskIndex(agent, ct);
        solution.paths[ct] = Path();
        affected_agents.insert(make_pair(ct, agent));
        solution.clearInterAgentPrecedenceConstraint(ct);
        // needs to happen after clearing precedence constraints
        solution.task_assignments[agent][ct_position] = -1;
        agents[agent].task_paths[ct_position] = nullptr;
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
        agents[agent].task_paths.erase(std::remove_if(agents[agent].task_paths.begin(),
                                                      agents[agent].task_paths.end(),
                                                      [](Path* p) { return p == nullptr; }),
                                       agents[agent].task_paths.end());
        agents[agent].path = Path();
        vector<int> task_assignments = solution.getAgentGlobalTasks(agent);
        vector<int> task_locations = instance.getTaskLocations(task_assignments);
        agents[agent].path_planner->setGoalLocations(task_locations);
        agents[agent].path_planner->compute_heuristics();
    }

    for (int task : planning_order) {
        if (std::find(solution.neighbor.conflicted_tasks.begin(),
                      solution.neighbor.conflicted_tasks.end(),
                      task) != solution.neighbor.conflicted_tasks.end()) {
            assert(solution.paths[task].empty());
            int start_time = 0, agent = affected_agents[task],
                task_position = global_to_local_task_id[task];
            if (task_position != 0)
                start_time = agents[agent].task_paths[task_position - 1]->end_time();
            if (task_position != (int)solution.task_assignments[agent].size()) {
                int next_task = solution.task_assignments[agent][task_position + 1];
                ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
                build_constraint_table(constraint_table, agent, next_task);
                solution.paths[next_task] = agents[agent].path_planner->findPathSegment(
                  constraint_table, start_time, task_position, 0);
                agents[agent].task_paths[task_position] = &solution.paths[next_task];
                // Once the path was found fix the begin times for subsequent tasks of the
                // agent
                for (int k = task_position + 1; k < (int)solution.task_assignments[agent].size();
                     k++)
                    agents[agent].task_paths[k]->begin_time =
                      agents[agent].task_paths[k - 1]->end_time();
            }
        }
    }

    vector<int> agents_to_compute;
    for (pair<int, int> task_agent : affected_agents)
        agents_to_compute.push_back(task_agent.second);
    joinPaths(agents_to_compute);

    sum_of_costs = 0;
    for (int i = 0; i < (int)agents.size(); i++) {
        sum_of_costs += agents[i].path.end_time();
    }
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
        /* PLOGD << "Task id : " << id << endl; */
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
LNS::build_constraint_table(ConstraintTable& constraint_table, int agent, int task)
{
    int task_position = solution.getLocalTaskIndex(agent, task);
    constraint_table.goal_location = agents[agent].path_planner->goal_locations[task_position];

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
        task_position = solution.getLocalTaskIndex(agent, id);
        bool wait_at_goal =
          task_position == (int)agents[agent].path_planner->goal_locations.size() - 1;
        constraint_table.addPath(*agents[agent].task_paths[task_position], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        int agent = solution.getAgentWithTask(agent_task_ancestor);
        task_position = solution.getLocalTaskIndex(agent, agent_task_ancestor);
        assert(!agents[agent].task_paths[task_position]->empty());
        constraint_table.length_min =
          max(constraint_table.length_min, agents[agent].task_paths[task_position]->end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

bool
LNS::validateSolution(bool extract)
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
            PLOGE << "Temporal conflict between " << agent_a << " doing task "
                  << precedence_constraint.first << " and agent " << agent_b << " doing task "
                  << precedence_constraint.second << endl;
            result = false;
            if (!extract)
                return false;
            else {
                solution.neighbor.conflicted_tasks.insert(precedence_constraint.first);
                solution.neighbor.conflicted_tasks.insert(precedence_constraint.second);
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
                    if (!extract)
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
                    if (!extract)
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
                        if (!extract)
                            return false;
                        else {
                            for (int goals = 0;
                                 goals < (int)solution.getAgentGlobalTasks(agent_i).size();
                                 goals++) {
                                if (agents[agent_i].path.timestamps[goals] > timestep) {
                                    int task_id = solution.getAgentGlobalTasks(agent_i)[goals];
                                    solution.neighbor.conflicted_tasks.insert(task_id);
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
                                    solution.neighbor.conflicted_tasks.insert(task_id);
                                    break;
                                }
                            }
                            for (int goals = 0;
                                 goals < (int)solution.getAgentGlobalTasks(agent_j).size();
                                 goals++) {
                                if (agents[agent_j].path.timestamps[goals] > timestep) {
                                    int task_id = solution.getAgentGlobalTasks(agent_j)[goals];
                                    solution.neighbor.conflicted_tasks.insert(task_id);
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
                                    solution.neighbor.conflicted_tasks.insert(task_id);
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
LNS::printPaths() const
{
    for (int i = 0; i < instance.getAgentNum(); i++) {
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
        cout << endl;
    }
}
