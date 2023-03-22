#include "lns.hpp"
#include "utils.hpp"

LNS::LNS(int num_of_iterations, Instance& instance, int neighbor_size, double time_limit)
  : num_of_iterations(num_of_iterations)
  , instance(instance)
  , neighbor_size(neighbor_size)
  , time_limit(time_limit)
{
    start_time = Time::now();

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
    preprocessing_time = ((fsec)(Time::now() - start_time)).count();
    PLOGD << "Pre-processing time = " << preprocessing_time << " seconds.\n";
}

void
LNS::joinPaths()
{
    for (int i = 0; i < instance.getAgentNum(); i++) {
        for (int j = 0; j < (int)agents[i].path_planner->goal_locations.size(); j++) {
            /* int id = instance.agent_task_to_id(make_pair(i, j)); */
            if (j == 0)
                agents[i].path.path.push_back(agents[i].task_paths[j]->front());
            assert((int)agents[i].path.size() - 1 == agents[i].task_paths[j]->begin_time);
            for (int k = 1; k < (int)agents[i].task_paths[j]->size(); k++)
                agents[i].path.path.push_back(agents[i].task_paths[j]->at(k));
            agents[i].path.timestamps.push_back(agents[i].path.size() - 1);
        }
    }
}

bool
LNS::run()
{
    start_time = Time::now();
    // assign tasks
    instance.id_base.resize(instance.getAgentNum(), 0);
    greedy_task_assignment(&instance);
    for (int i = 0; i < instance.getAgentNum(); i++) {
        if ((int)instance.getAgentTasks(i).size() == 0) {
            vector<int> goal = { instance.getStartLocations()[i] };
            agents[i].path_planner->setGoalLocations(goal);
            agents[i].task_paths.resize(1, nullptr);
        } else {
            vector<int> task_assignments = instance.getAgentTasks(i);
            vector<int> task_locations = instance.getTaskLocations(task_assignments);
            agents[i].path_planner->setGoalLocations(task_locations);
            agents[i].task_paths.resize((int)task_locations.size(), nullptr);
        }
        agents[i].path_planner->compute_heuristics();
        for (int j = 0; j < (int)agents[i].path_planner->goal_locations.size(); j++)
            instance.id_to_agent_task.push_back(make_pair(i, j));
        if (i != 0)
            instance.id_base[i] =
              instance.id_base[i - 1] + (int)agents[i - 1].path_planner->goal_locations.size();
    }

    for (int i = 0; i < instance.getAgentNum(); i++)
        for (int j = 1; j < (int)agents[i].path_planner->goal_locations.size(); j++)
            instance.insertPrecedenceConstraint(instance.agent_task_to_id(make_pair(i, j - 1)),
                                                instance.agent_task_to_id(make_pair(i, j)));
    for (int i = 0; i < instance.getTasksNum(); i++) {
        int agent_b = instance.getAgentWithTask(i);
        int task_b = instance.getLocalTaskIndex(agent_b, i);
        vector<int> previous_tasks = instance.getTaskDependencies()[i];
        for (int j = 0; j < (int)previous_tasks.size(); j++) {
            int agent_a = instance.getAgentWithTask(previous_tasks[j]);
            int task_a = instance.getLocalTaskIndex(agent_a, previous_tasks[j]);
            instance.insertPrecedenceConstraint(
              instance.agent_task_to_id(make_pair(agent_a, task_a)),
              instance.agent_task_to_id(make_pair(agent_b, task_b)));
        }
    }

    // find paths based on the task assignments
    vector<int> planning_order;
    bool success = topological_sort(&instance, planning_order);
    if (!success) {
        PLOGE << "Topological sorting failed\n";
        return success;
    }

    initial_paths.resize(instance.getTasksNum(), Path());
    for (int id : planning_order) {
        pair<int, int> agent_task = instance.id_to_agent_task[id];
        int agent = agent_task.first, task = agent_task.second, start_time = 0;
        if (task != 0) {
            assert(!initial_paths[instance.agent_task_to_id(make_pair(agent, task - 1))].empty());
            start_time =
              initial_paths[instance.agent_task_to_id(make_pair(agent, task - 1))].end_time();
        }
        PLOGI << "Planning for agent " << agent << " and task " << task << endl;
        ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
        build_constraint_table(constraint_table, agent, task);

        initial_paths[id] =
          agents[agent].path_planner->findPathSegment(constraint_table, start_time, task, 0);
        if (initial_paths[id].empty()) {
            PLOGE << "No path exists for agent " << agent << " and task " << task << endl;
            return false;
        }

        agents[agent].task_paths[task] = &initial_paths[id];
    }

    joinPaths();

    int initial_sum_of_costs = 0;
    for (int i = 0; i < (int)agents.size(); i++) {
        initial_sum_of_costs += agents[i].path.end_time();
    }

    printPaths();

    initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
    iteration_stats.emplace_back(initial_solution_runtime,
                                 "greedy",
                                 instance.getAgentNum(),
                                 instance.getTasksNum(),
                                 initial_sum_of_costs);
    runtime = initial_solution_runtime;

    PLOGD << "Initial solution cost = " << initial_sum_of_costs
          << ", Runtime = " << initial_solution_runtime << endl;

    while (runtime < time_limit && (int)iteration_stats.size() <= num_of_iterations) {
        runtime = ((fsec)(Time::now() - start_time)).count();
        set<int> conflicted_tasks;
        bool valid = validateSolution(&conflicted_tasks);
        if (!valid) {
            PLOGE << "The initial solution was not valid!\n";
            for (int ct : conflicted_tasks)
                PLOGD << "Task " << ct << endl;
            return false;
        } else
            return true;
    }

    // change this later
    return true;
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table, int agent, int task)
{
    int agent_task_id = instance.agent_task_to_id(make_pair(agent, task));
    constraint_table.goal_location = agents[agent].path_planner->goal_locations[task];

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : instance.getPrecedenceConstraints())
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ agent_task_id });
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
    set_of_tasks_to_complete.erase(agent_task_id);

    for (int id : set_of_tasks_to_complete) {
        pair<int, int> agent_task = instance.id_to_agent_task[id];
        int agent = agent_task.first, task = agent_task.second;
        bool wait_at_goal = task == (int)agents[agent].path_planner->goal_locations.size() - 1;
        constraint_table.addPath(*agents[agent].task_paths[task], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[agent_task_id]) {
        pair<int, int> agent_task = instance.id_to_agent_task[agent_task_ancestor];
        int agent = agent_task.first, task = agent_task.second;
        assert(!agents[agent].task_paths[task]->empty());
        constraint_table.length_min =
          max(constraint_table.length_min, agents[agent].task_paths[task]->end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

bool
LNS::validateSolution(set<int>* conflicted_tasks)
{

    bool result = true;

    // Check that the precedence constraints are not violated
    vector<pair<int, int>> precedence_constraints = instance.getPrecedenceConstraints();
    for (pair<int, int> precedence_constraint : precedence_constraints) {
        pair<int, int> agent_task_a = instance.id_to_agent_task[precedence_constraint.first];
        pair<int, int> agent_task_b = instance.id_to_agent_task[precedence_constraint.second];
        int agent_a = agent_task_a.first, task_a = agent_task_a.second;
        int agent_b = agent_task_b.first, task_b = agent_task_b.second;
        if (agents[agent_a].path.timestamps[task_a] >= agents[agent_b].path.timestamps[task_b]) {
            PLOGE << "Temporal conflict between " << agent_a << " doing local task " << task_a
                  << " and agent " << agent_b << " doing local task " << task_b << endl;
            result = false;
            if (conflicted_tasks == nullptr)
                return false;
            else {
                conflicted_tasks->insert(task_a);
                conflicted_tasks->insert(task_b);
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
                        for (int goals = 0; goals < (int)instance.getAgentTasks(agent_i).size();
                             goals++)
                            if (agents[agent_i].path.timestamps[goals] > timestep) {
                                conflicted_tasks->insert(instance.getAgentTasks(agent_i)[goals]);
                                break;
                            }
                        for (int goals = 0; goals < (int)instance.getAgentTasks(agent_j).size();
                             goals++)
                            if (agents[agent_j].path.timestamps[goals] > timestep) {
                                conflicted_tasks->insert(instance.getAgentTasks(agent_j)[goals]);
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
                        for (int goals = 0; goals < (int)instance.getAgentTasks(agent_i).size();
                             goals++)
                            if (agents[agent_i].path.timestamps[goals] > timestep) {
                                conflicted_tasks->insert(instance.getAgentTasks(agent_i)[goals]);
                                break;
                            }
                        for (int goals = 0; goals < (int)instance.getAgentTasks(agent_j).size();
                             goals++)
                            if (agents[agent_j].path.timestamps[goals] > timestep) {
                                conflicted_tasks->insert(instance.getAgentTasks(agent_j)[goals]);
                                break;
                            }
                    }
                }
            }

            // Check that any two agents are not at the same location at the same timestep where one
            // agent might be waiting already
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
                            for (int goals = 0; goals < (int)instance.getAgentTasks(agent_i).size();
                                 goals++)
                                if (agents[agent_i].path.timestamps[goals] > timestep) {
                                    conflicted_tasks->insert(
                                      instance.getAgentTasks(agent_i)[goals]);
                                    break;
                                }
                            for (int goals = 0; goals < (int)instance.getAgentTasks(agent_j).size();
                                 goals++)
                                if (agents[agent_j].path.timestamps[goals] > timestep ||
                                    agents[agent_j].path.timestamps
                                        [(int)agents[agent_j].path.timestamps.size() - 1] <
                                      timestep) {
                                    conflicted_tasks->insert(
                                      instance.getAgentTasks(agent_j)[goals]);
                                    break;
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
