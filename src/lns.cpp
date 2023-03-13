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
            agents[i].paths.resize(1, nullptr);
        } else {
            vector<int> task_assignments = instance.getAgentTasks(i);
            vector<int> task_locations = instance.getTaskLocations(task_assignments);
            agents[i].path_planner->setGoalLocations(task_locations);
            agents[i].paths.resize((int)task_locations.size(), nullptr);
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
        ConstraintTable constraint_table;
        build_constraint_table(constraint_table, agent, task);

        initial_paths[id] =
          agents[agent].path_planner->findPathSegment(constraint_table, start_time, task, 0);
        if (initial_paths[id].empty()) {
            PLOGE << "No path exists for agent " << agent << " and task " << task << endl;
            return false;
        }

        agents[agent].paths[task] = &initial_paths[id];
    }

    int initial_sum_of_costs = 0;
    for (int i = 0; i < (int)agents.size(); i++) {
        initial_sum_of_costs += agents[i].paths[(int)agents[i].paths.size() - 1]->end_time();
    }

    PLOGI << "Printing paths to verify initial solution correctness\n";
    for (int i = 0; i < instance.getTasksNum(); i++) {
        int agent = instance.getAgentWithTask(i), task = instance.getLocalTaskIndex(agent, i);
        if (task != -1) {
            PLOGI << "Agent " << agent << " doing task " << i << "\n";
            for (int j = 0; j < (int)agents[agent].paths[task]->path.size(); j++)
                PLOGI << "\t " << agents[agent].paths[task]->path[j].location << " -> ";
            PLOGI << "\n\n";
        }
    }
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
        /* bool valid = validateSolution(); */
    }

    // change this later
    return true;
}

bool
LNS::validateSolution() const
{
    /* int sum = 0; */
    /* for (const Agent& a : agents) */

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
        constraint_table.addPath(*agents[agent].paths[task], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[agent_task_id]) {
        pair<int, int> agent_task = instance.id_to_agent_task[agent_task_ancestor];
        int agent = agent_task.first, task = agent_task.second;
        assert(!agents[agent].paths[task]->empty());
        constraint_table.length_min =
          max(constraint_table.length_min, agents[agent].paths[task]->end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}
