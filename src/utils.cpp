#include "utils.hpp"

void
greedyTaskAssignment(const Instance* instance, Solution* solution)
{
    pq q;
    vector<int> agentLastTimesteps(instance->getAgentNum(), 0);
    vector<int> agentLastLocations = instance->getStartLocations();
    vector<int> taskCompleteTimesteps(instance->getTasksNum(), -1);

    // We first compute the heuristic value for all the tasks irrespective of the agents
    unique_ptr<SingleAgentSolver> searchEngine =
      make_unique<MultiLabelSpaceTimeAStar>((*instance), 0);

    for (int agent = 0; agent < instance->getAgentNum(); agent++) {
        q.emplace(0, agent); // (key, value) - (timestep, agent)
    }

    int taskCounter = 0;
    while (taskCounter < instance->getTasksNum()) {
        int timestep, agent;
        tie(timestep, agent) = q.top();

        PLOGD << "Planning for agent " << agent << " at timestep " << timestep << endl;
        int lastLocationOfAgent = agentLastLocations[agent];
        q.pop();

        int bestTaskToService = -1, bestTaskToServiceTimestep = INT_MAX;
        for (int task = 0; task < instance->getTasksNum(); task++) {
            if (taskCompleteTimesteps[task] != -1) { // Task has been assigned before
                continue;
            }

            bool taskReady = true;
            // The time this agent can service this task and estimated cost of completing that
            // task from the agent's location
            int taskTimestep =
              agentLastTimesteps[agent] + searchEngine->heuristic[task][lastLocationOfAgent];

            // Check for temporal dependencies
            unordered_map<int, vector<int>> taskDependencies = instance->getTaskDependencies();
            if (taskDependencies.find(task) != taskDependencies.end()) {
                for (int dependentTask : taskDependencies[task]) {
                    if (taskCompleteTimesteps[dependentTask] < 0) {
                        // The dependent tasks need to be completed before this task can be serviced
                        taskReady = false;
                        break;
                    }
                    taskTimestep = max(taskCompleteTimesteps[dependentTask], taskTimestep);
                }
            }

            if (taskReady && taskTimestep < bestTaskToServiceTimestep) {
                bestTaskToService = task;
                bestTaskToServiceTimestep = taskTimestep;
            }
        }

        // Assign the best task found to the agent
        if (bestTaskToService != -1) {
            PLOGD << "Assign task " << bestTaskToService << " to agent " << agent
                  << " with distance "
                  << searchEngine->heuristic[bestTaskToService][lastLocationOfAgent] << endl;
            solution->assignTaskToAgent(agent, bestTaskToService);
            agentLastTimesteps[agent] = bestTaskToServiceTimestep;
            taskCompleteTimesteps[bestTaskToService] = bestTaskToServiceTimestep;
            agentLastLocations[agent] = instance->getTaskLocations()[bestTaskToService];
            taskCounter++;
        }

        q.emplace(agentLastTimesteps[agent], agent);
    }
}

bool
topologicalSort(const Instance* instance,
                 vector<pair<int, int>>* precedenceConstraints,
                 vector<int>& planningOrder)
{
    planningOrder.clear();
    vector<bool> closed(instance->getTasksNum(), false);
    vector<bool> expanded(instance->getTasksNum(), false);

    vector<vector<int>> successors;
    successors.resize(instance->getTasksNum());
    for (pair<int, int> precedenceConstraint : (*precedenceConstraints)) {
        successors[precedenceConstraint.first].push_back(precedenceConstraint.second);
    }

    for (int task = 0; task < instance->getTasksNum(); task++) {
        if (closed[task]) {
            continue;
        }

        stack<int> dfsStack;
        dfsStack.push(task);

        while (!dfsStack.empty()) {

            int currentTask = dfsStack.top();
            dfsStack.pop();
            if (closed[currentTask]) {
                continue;
            }
            if (expanded[currentTask]) {
                closed[currentTask] = true;
                planningOrder.push_back(currentTask);
            } else {
                expanded[currentTask] = true;
                dfsStack.push(currentTask);
                for (int dependentTask : successors[currentTask]) {
                    if (closed[dependentTask]) {
                        continue;
                    }
                    if (expanded[dependentTask]) {
                        PLOGE << "Detected a cycle while running topological sort\n";
                        return false;
                    }
                    dfsStack.push(dependentTask);
                }
            }
        }
    }

    reverse(planningOrder.begin(), planningOrder.end());

    unordered_set<int> tasksOrder;
    for (int task : planningOrder) {
        for (int dependentTask : successors[task]) {
            if (tasksOrder.find(dependentTask) != tasksOrder.end()) {
                PLOGE << "The topological sort violated a precedence constraint\n";
                return false;
            }
        }
        tasksOrder.insert(task);
    }

    assert((int)planningOrder.size() == instance->getTasksNum());
    return true;
}
