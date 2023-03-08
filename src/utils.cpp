#include "utils.hpp"

int
greedy_task_assignment(Instance* instance)
{
    pq q;
    vector<int> agent_last_timesteps(instance->getAgentNum(), 0);
    vector<int> agent_last_locations = instance->getStartLocations();
    vector<int> task_complete_timesteps(instance->getTasksNum(), -1);

    // We first compute the heuristic value for all the tasks irrespective of the agents
    unique_ptr<SingleAgentSolver> search_engine = make_unique<SingleAgentSolver>((*instance), 0);

    for (int agent = 0; agent < instance->getAgentNum(); agent++)
        q.push(make_pair(0, agent)); // timestep, agent

    int task_counter = 0;
    while (task_counter < instance->getTasksNum()) {
        int timestep, agent;
        tie(timestep, agent) = q.top();

        PLOGD << "Planning for agent " << agent << " at timestep " << timestep << endl;
        /* if (agent == 2 && timestep == 13) */
        /*     assert(false); */
        int last_location_of_agent = agent_last_locations[agent];
        q.pop();

        int best_task_to_service = -1, best_task_to_service_timestep = INT_MAX;
        for (int task = 0; task < instance->getTasksNum(); task++) {
            if (task_complete_timesteps[task] != -1) // task has been assigned before
                continue;

            bool task_ready = true;
            // the time the this agent can service this task and estimated cost of completing that
            // task from the agents location
            int task_timestep =
              agent_last_timesteps[agent] + search_engine->heuristic[task][last_location_of_agent];

            // check for temporal dependencies
            unordered_map<int, vector<int>> task_dependencies = instance->getTaskDependencies();
            if (task_dependencies.find(task) != task_dependencies.end())
                for (int dependent_task : task_dependencies[task]) {
                    if (task_complete_timesteps[dependent_task] < 0) {
                        // the dependent tasks need to be completed before this task can be serviced
                        task_ready = false;
                        break;
                    }
                    task_timestep = max(task_complete_timesteps[dependent_task], task_timestep);
                }

            if (task_ready && task_timestep < best_task_to_service_timestep) {
                best_task_to_service = task;
                best_task_to_service_timestep = task_timestep;
            }
        }

        // Assign the best task found to the agent
        if (best_task_to_service != -1) {
            PLOGD << "Assign task " << best_task_to_service << " to agent " << agent
                  << " with distance "
                  << search_engine->heuristic[best_task_to_service][last_location_of_agent] << endl;
            instance->assignTaskToAgent(agent, best_task_to_service);
            agent_last_timesteps[agent] = best_task_to_service_timestep;
            task_complete_timesteps[best_task_to_service] = best_task_to_service_timestep;
            agent_last_locations[agent] = instance->getTaskLocations()[best_task_to_service];
            task_counter++;
        }

        q.push(make_pair(agent_last_timesteps[agent], agent));
    }

    // compute the sum of costs and return that
    int sum_of_costs = 0;
    for (int agent = 0; agent < instance.getAgentNum(); agent++)
        sum_of_costs += agent_last_timesteps[agent];
    return sum_of_costs;
}
