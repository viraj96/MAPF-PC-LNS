#include "utils.hpp"

void greedyTaskAssignment(const Instance* instance, Solution* solution) {
  ppqg q;
  vector<int> agentLastTimesteps(instance->getAgentNum(), 0);
  vector<int> agentLastLocations = instance->getStartLocations();
  vector<int> taskCompleteTimesteps(instance->getTasksNum(), -1);

  // We first compute the heuristic value for all the tasks irrespective of the agents
  unique_ptr<SingleAgentSolver> searchEngine =
      make_unique<MultiLabelSpaceTimeAStar>((*instance), 0);

  for (int agent = 0; agent < instance->getAgentNum(); agent++) {
    q.emplace(0, agent);  // (key, value) - (timestep, agent)
  }

  int taskCounter = 0;
  while (taskCounter < instance->getTasksNum()) {
    int timestep, agent;
    tie(timestep, agent) = q.top();

    PLOGD << "Planning for agent " << agent << " at timestep " << timestep
          << endl;
    int lastLocationOfAgent = agentLastLocations[agent];
    q.pop();

    int bestTaskToService = -1, bestTaskToServiceTimestep = INT_MAX;
    for (int task = 0; task < instance->getTasksNum(); task++) {
      if (taskCompleteTimesteps[task] != -1) {  // Task has been assigned before
        continue;
      }

      bool taskReady = true;
      // The time this agent can service this task and estimated cost of completing that
      // task from the agent's location
      int taskTimestep = agentLastTimesteps[agent] +
                         searchEngine->heuristic[task][lastLocationOfAgent];

      // Check for temporal dependencies
      map<int, vector<int>> taskDependencies = instance->getTaskDependencies();
      if (taskDependencies.find(task) != taskDependencies.end()) {
        for (int dependentTask : taskDependencies[task]) {
          if (taskCompleteTimesteps[dependentTask] < 0) {
            // The dependent tasks need to be completed before this task can be serviced
            taskReady = false;
            break;
          }
          taskTimestep =
              max(taskCompleteTimesteps[dependentTask], taskTimestep);
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
            << searchEngine->heuristic[bestTaskToService][lastLocationOfAgent]
            << endl;
      solution->assignTaskToAgent(agent, bestTaskToService);
      agentLastTimesteps[agent] = bestTaskToServiceTimestep;
      taskCompleteTimesteps[bestTaskToService] = bestTaskToServiceTimestep;
      agentLastLocations[agent] =
          instance->getTaskLocations()[bestTaskToService];
      taskCounter++;
    }

    q.emplace(agentLastTimesteps[agent], agent);
  }
}

bool topologicalSort(const Instance* instance,
                     vector<pair<int, int>>* precedenceConstraints,
                     vector<int>& planningOrder) {
  planningOrder.clear();
  vector<bool> closed(instance->getTasksNum(), false);
  vector<bool> expanded(instance->getTasksNum(), false);

  vector<vector<int>> successors;
  successors.resize(instance->getTasksNum());
  for (pair<int, int> precedenceConstraint : (*precedenceConstraints)) {
    successors[precedenceConstraint.first].push_back(
        precedenceConstraint.second);
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

bool isSamePath(const Path& p1, const Path& p2) {
  if (p1.size() != p2.size()) {
    return false;
  }
  for (int i = 0; i < (int)p1.size(); i++) {
    if (p1.path[i].location != p2.path[i].location) {
      return false;
    }
  }
  return true;
}

set<Conflicts> extractNConflicts(int size, const set<Conflicts>& conflicts) {
  int i = 0;
  set<Conflicts> result;
  for (Conflicts conflict : conflicts) {
    if (i >= size) {
      break;
    }
    result.insert(conflict);

    int correspondingTask = -1;
    if (conflict.task % 2 == 0) {
      // This was a pickup task
      correspondingTask = conflict.task + 1;
    }
    else {
      correspondingTask = conflict.task - 1;
    }

    for (Conflicts innerConflict : conflicts) {
      if (innerConflict.task == correspondingTask) {
        result.insert(innerConflict);
        break;
      }
    }

    i++;
  }
  return result;
}

double MovingMetrics::computeMovingMetrics(int numberOfConflicts,
                                           int sumOfCosts) {

  // Compute the utility of this solution
  // Compute the new sample
  int numConflictsSquare = pow(numberOfConflicts, 2),
      numCostSquare = pow(sumOfCosts, 2);
  // Extract the oldest sample
  double oldestNumConflicts = conflictNum[oldestValue],
         oldestNumConflictsSquare = conflictSquareNum[oldestValue],
         oldestNumCost = costNum[oldestValue],
         oldestNumCostSquare = costSquareNum[oldestValue];

  // Update the oldest sample
  conflictNum[oldestValue] = numberOfConflicts;
  conflictSquareNum[oldestValue] = numConflictsSquare;
  costNum[oldestValue] = sumOfCosts;
  costSquareNum[oldestValue] = numCostSquare;

  // Update the oldest sample location
  oldestValue++;
  oldestValue %= size;

  // Compute the new sum based on the adding the new sample and removing the oldest sample
  sumOfNumConflicts += numberOfConflicts - oldestNumConflicts;
  sumOfNumConflictsSquare += numConflictsSquare - oldestNumConflictsSquare;
  sumOfNumCosts += sumOfCosts - oldestNumCost;
  sumOfNumCostsSquare += numCostSquare - oldestNumCostSquare;

  // Compute the moving average and variance of the number of conflicts and sum of costs variables
  double movingNumConflictAverage = sumOfNumConflicts / size,
         movingNumConflictVar =
             (size * sumOfNumConflictsSquare - (pow(sumOfNumConflicts, 2))) /
             (size * (size - 1));
  double movingNumCostAverage = sumOfNumCosts / size,
         movingNumCostVar =
             (size * sumOfNumCostsSquare - (pow(sumOfNumCosts, 2))) /
             (size * (size - 1));

  PLOGD << "Moving average of conflicts = " << movingNumConflictAverage
        << ", Moving average of costs = " << movingNumCostAverage << "\n";

  double utility =
      lnsConflictWeight * ((numberOfConflicts - movingNumConflictAverage) /
                           sqrt(movingNumConflictVar + 1)) +
      lnsCostWeight *
          ((sumOfCosts - movingNumCostAverage) / sqrt(movingNumCostVar + 1));
  return utility;
}
