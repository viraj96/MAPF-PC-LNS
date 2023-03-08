#include "lns.hpp"
#include "utils.hpp"

LNS::LNS(const Instance& instance, double time_limit, int neighbor_size, int num_of_iterations)
  : instance(instance)
  , time_limit(time_limit)
  , neighbor_size(neighbor_size)
  , num_of_iterations(num_of_iterations)
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
    int initial_sum_of_costs = greedy_task_assignment(instance);
    initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
    iteration_stats.emplace_back(initial_solution_runtime,
                                 "greedy",
                                 instance.getAgentNum(),
                                 instance.getTasksNum(),
                                 initial_sum_of_costs);
    runtime = initial_solution_runtime;

    PLOGD << "Initial solution cost = " << initial_sum_of_costs
          << ", Runtime = " << initial_solution_runtime << endl;

    while (runtime < time_limit && iteration_stats.size() <= num_of_iterations) {
        runtime = ((fsec)(Time::now() - start_time)).count();
        bool valid = validateSolution();
    }
}

bool
LNS::validateSolution() const
{
    /* int sum = 0; */
    /* for (const Agent& a : agents) */
}
