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
    int initial_sum_of_costs = greedy_task_assignment(&instance);
    for (int i = 0; i < instance.getAgentNum(); i++) {
        if ((int)instance.getAgentTasks(i).size() == 0) {
            vector<int> goal = { instance.getStartLocations()[i] };
            agents[i].path_planner->setGoalLocations(goal);
        } else
            agents[i].path_planner->setGoalLocations(instance.getAgentTasks(i));
        for (int j = 0; j < (int)agents[i].path_planner->goal_locations.size(); j++)
            instance.id_to_agent_task.push_back(make_pair(i, j));
        if (i != 0)
            instance.id_base[i] =
              instance.id_base[i - 1] + (int)agents[i - 1].path_planner->goal_locations.size();
    }
    for (int i = 0; i < instance.getAgentNum(); i++)
        for (int j = 1; j < (int)agents[i].path_planner->goal_locations.size(); j++)
            instance.precedence_constraints.push_back(
              make_pair(instance.agent_task_to_id(make_pair(i, j - 1)),
                        instance.agent_task_to_id(make_pair(i, j))));
    for (int i = 0; i < (int)instance.getTaskDependencies().size(); i++) {
        int task_a = i, agent_a = instance.getAgentWithTask(i);
        for (int j = 0; j < (int)instance.getTaskDependencies()[task_a].size(); j++) {
            int task_b = j, agent_b = instance.getAgentWithTask(j);
            instance.precedence_constraints.push_back(
              make_pair(instance.agent_task_to_id(make_pair(agent_a, task_a)),
                        instance.agent_task_to_id(make_pair(agent_b, task_b))));
        }
    }

    // find paths based on the task assignments
    vector<int> planning_order;
    bool success = topological_sort(instance, planning_order);
    if (!success) {
        PLOGE << "Topological sorting failed\n";
        return success;
    }

    vector<Path> initial_paths;
    for (int id : planning_order) {
        pair<int, int> agent_task = instance.id_to_agent_task[id];
        int agent = agent_task.first, task = agent_task.second, start_time = 0;
        if (task != 0) {
            assert(!initial_paths[agent_task_to_id(make_pair(agent, task - 1))].empty());
            start_time = initial_paths[agent_task_to_id(make_pair(agent, task - 1))].end_time();
        }
        PLOGI << "Planning for agent " << agent << " and task " << task << endl;
        ConstraintTable constraint_table;
        build_constraint_table(constraint_table, agent, task);
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
    constraint_table.goal_location = agents[agent].path_planner->goal_locations[task];
}
