#include "lns.hpp"
#include <numeric>
#include "utils.hpp"

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

LNS::LNS(int numOfIterations, const Instance& instance, int neighborSize,
         double timeLimit)
    : numOfIterations_(numOfIterations),
      neighborSize_(neighborSize),
      instance_(instance),
      solution_(instance),
      previousSolution_(instance),
      timeLimit_(timeLimit) {
  plannerStartTime_ = Time::now();
}

bool LNS::buildGreedySolution() {

  // Assign tasks
  greedyTaskAssignment(&instance_, &solution_);
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    vector<int> taskLocations =
        instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
    solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[agent].taskPaths.resize(
        solution_.getAgentGlobalTasks(agent).size(), Path());
    solution_.agents[agent].pathPlanner->computeHeuristics();
  }

  // Compute the precedence constraints based on current task assignments
  // Intra-agent precedence constraints
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    for (int task = 1; task < (int)solution_.getAgentGlobalTasks(agent).size();
         task++) {
      solution_.insertPrecedenceConstraint(
          solution_.taskAssignments[agent][task - 1],
          solution_.taskAssignments[agent][task]);
    }
  }
  // Inter-agent precedence constraints - these should be static
  for (int task = 0; task < instance_.getTasksNum(); task++) {
    vector<int> previousTasks = instance_.getTaskDependencies()[task];
    for (int pt : previousTasks) {
      solution_.insertPrecedenceConstraint(pt, task);
    }
  }

  // Find paths based on the task assignments
  // First we need to sort the tasks based on the precedence constraints
  vector<int> planningOrder;
  bool success = topologicalSort(&instance_, &solution_.precedenceConstraints,
                                 planningOrder);
  if (!success) {
    PLOGE << "Topological sorting failed\n";
    return success;
  }

  // Following the topological order we find the paths for each task
  initialPaths.resize(instance_.getTasksNum(), Path());
  solution_.paths.resize(instance_.getTasksNum(), Path());
  for (int id : planningOrder) {

    int agent = solution_.getAgentWithTask(id), task = id,
        taskPosition = solution_.getLocalTaskIndex(agent, task), startTime = 0;
    if (taskPosition != 0) {
      int previousTask = solution_.taskAssignments[agent][taskPosition - 1];
      assert(!initialPaths[previousTask].empty());
      startTime = initialPaths[previousTask].endTime();
    }

    PLOGI << "Planning for agent " << agent << " and task " << task << endl;

    ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
    buildConstraintTable(constraintTable, task);
    initialPaths[id] = solution_.agents[agent].pathPlanner->findPathSegment(
        constraintTable, startTime, taskPosition, 0);
    if (initialPaths[id].empty()) {
      PLOGE << "No path exists for agent " << agent << " and task " << task
            << endl;
      return false;
    }

    solution_.agents[agent].taskPaths[taskPosition] = initialPaths[id];
    solution_.paths[id] = initialPaths[id];
  }

  // Join the individual task paths to form the agent's path
  vector<int> agentsToCompute(instance_.getAgentNum());
  std::iota(agentsToCompute.begin(), agentsToCompute.end(), 0);
  solution_.joinPaths(agentsToCompute);

  // Gather the information
  int initialSumOfCosts = 0;
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    initialSumOfCosts += solution_.agents[agent].path.endTime();
  }
  solution_.sumOfCosts = initialSumOfCosts;
  return true;
}

bool LNS::run() {

  bool success = buildGreedySolution();
  if (!success) {
    return success;
  }

  printPaths();

  initialSolutionRuntime_ = ((fsec)(Time::now() - plannerStartTime_)).count();
  iterationStats.emplace_back(initialSolutionRuntime_, "greedy",
                              instance_.getAgentNum(), instance_.getTasksNum(),
                              solution_.sumOfCosts);
  runtime = initialSolutionRuntime_;

  PLOGD << "Initial solution cost = " << solution_.sumOfCosts
        << ", Runtime = " << initialSolutionRuntime_ << endl;

  // LNS loop
  while (runtime < timeLimit_ &&
         (int)iterationStats.size() <= numOfIterations_) {

    runtime = ((fsec)(Time::now() - plannerStartTime_)).count();

    // Extract the set of conflicting tasks
    set<int> conflictedTasks;
    bool valid = validateSolution(&conflictedTasks);

    if (valid) {
      PLOGV << "Solution was found!\n";
      break;
    }

    // Solution was not valid as we found some conflicts!
    PLOGE << "The initial solution was not valid!\n";
    solution_.neighbor.conflictedTasks = conflictedTasks;
    previousSolution_ = solution_;

    prepareNextIteration();

    // Compute regret for each of the tasks that are in the conflicting set
    // Pick the best one and repeat the whole process aboe
    while (!solution_.neighbor.conflictedTasks.empty()) {
      computeRegret();
      Regret bestRegret = solution_.neighbor.regretMaxHeap.top();
      // Use the best regret task and insert it in its correct location
      commitBestRegretTask(bestRegret);
    }

    // Join the individual paths that were found for each agent
    for (int i = 0; i < instance_.getAgentNum(); i++) {
      solution_.agents[i].path = Path();
    }

    vector<int> agentsToCompute(instance_.getAgentNum());
    std::iota(agentsToCompute.begin(), agentsToCompute.end(), 0);
    solution_.joinPaths(agentsToCompute);

    printPaths();

    // Compute the updated sum of costs
    solution_.sumOfCosts = 0;
    for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
      solution_.sumOfCosts += solution_.agents[agent].path.endTime();
    }

    PLOGD << "Old sum of costs = " << previousSolution_.sumOfCosts << endl;
    PLOGD << "New sum of costs = " << solution_.sumOfCosts << endl;

    // Accept the solution only if the new one has lower number of conflicts or it has lower
    // cost of the solution
    conflictedTasks.clear();
    valid = validateSolution(&conflictedTasks);

    PLOGD << "Number of conflicts in old solution: "
          << previousSolution_.neighbor.conflictedTasks.size() << endl;
    PLOGD << "Number of conflicts in new solution: " << conflictedTasks.size()
          << endl;

    if (previousSolution_.neighbor.conflictedTasks.size() <
        conflictedTasks.size()) {
      // Reject this solution
      for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        delete solution_.agents[agent].pathPlanner;
      }
      solution_ = previousSolution_;
      PLOGD << "Rejecting this solution!\n";
    } else if (previousSolution_.neighbor.conflictedTasks.size() ==
               conflictedTasks.size()) {
      if (previousSolution_.sumOfCosts < solution_.sumOfCosts) {
        // Reject this solution
        for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
          delete solution_.agents[agent].pathPlanner;
        }
        solution_ = previousSolution_;
        PLOGD << "Rejecting this solution!\n";
      }
    } else {
      // Accept this solution
      for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        delete previousSolution_.agents[agent].pathPlanner;
      }
    }
  }

  PLOGV << "MAPF-PC-LNS: "
        << "\n\tRuntime = " << runtime
        << "\n\tIterations = " << iterationStats.size()
        << "\n\tSolution Cost = " << sumOfCosts
        << "\n\tNumber of failures = " << numOfFailures << endl;

  return true;
}

void LNS::prepareNextIteration() {
  // Remove the conflicted tasks from the agents paths and recompile their paths
  PLOGI << "Preparing the solution object for next iteration\n";

  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    solution_.agents[agent].pathPlanner =
        new MultiLabelSpaceTimeAStar(instance_, agent);
    vector<int> taskLocations =
        instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
    solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[agent].pathPlanner->computeHeuristics();
  }

  vector<int>
      tasksToFix;  // If t_id is deleted then t_id + 1 task needs to be fixed
  unordered_map<int, int> affectedAgents, globalToLocalTaskId;
  for (int ct : solution_.neighbor.conflictedTasks) {

    int agent = solution_.getAgentWithTask(ct),
        ctPosition = solution_.getLocalTaskIndex(agent, ct),
        pathSize = (int)solution_.paths[ct]
                       .size();  // path_size is used for heuristic estimate
    PLOGD << "Conflicting task: " << ct << ", Agent: " << agent << endl;

    // If the conflicted task was not the last local task of this agent then t_id + 1 exists
    if (ctPosition != (int)solution_.getAgentGlobalTasks(agent).size() - 1) {
      int nextTask = solution_.getAgentGlobalTasks(agent, ctPosition + 1);
      if (solution_.neighbor.conflictedTasks.find(nextTask) ==
          solution_.neighbor.conflictedTasks.end()) {
        tasksToFix.push_back(nextTask);
      }
      pathSize += solution_.paths[nextTask].size();
      PLOGD << "Next task: " << nextTask << endl;
    }

    // TODO: What happens when the next task is also a conflicting task! Then how does it affect the path size computation?
    solution_.paths[ct] = Path();
    affectedAgents.insert(make_pair(ct, agent));
    globalToLocalTaskId.insert(make_pair(ct, ctPosition));
    solution_.neighbor.conflictedTasksPathSize.insert(make_pair(ct, pathSize));

    // Marking past information about this conflicting task
    solution_.clearIntraAgentPrecedenceConstraint(ct);
    // Needs to happen after clearing precedence constraints
    solution_.taskAssignments[agent][ctPosition] = -1;
    solution_.agents[agent].taskPaths[ctPosition] = Path();
  }

  vector<int> planningOrder;
  assert(topologicalSort(&instance_, &solution_.precedenceConstraints,
                         planningOrder));

  // Marking past information about conflicting tasks
  for (pair<int, int> taskAgent : affectedAgents) {
    int agent = taskAgent.second;

    // For an affected agent there can be multiple conflicting tasks so need to do it this way
    solution_.agents[agent].path = Path();
    solution_.taskAssignments[agent].erase(
        std::remove_if(solution_.taskAssignments[agent].begin(),
                       solution_.taskAssignments[agent].end(),
                       [](int task) { return task == -1; }),
        solution_.taskAssignments[agent].end());
    solution_.agents[agent].taskPaths.erase(
        std::remove_if(solution_.agents[agent].taskPaths.begin(),
                       solution_.agents[agent].taskPaths.end(),
                       [](const Path& p) { return isSamePath(p, Path()); }),
        solution_.agents[agent].taskPaths.end());

    vector<int> taskLocations =
        instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
    solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[agent].pathPlanner->computeHeuristics();
  }

  // Find the paths for the tasks whose previous tasks were removed
  for (int task : planningOrder) {
    if (std::find(tasksToFix.begin(), tasksToFix.end(), task) !=
        tasksToFix.end()) {

      PLOGD << "Going to find path for next task: " << task << endl;

      int startTime = 0, agent = solution_.getAgentWithTask(task);
      int taskPosition = solution_.getLocalTaskIndex(agent, task);

      if (taskPosition != 0) {
        startTime =
            solution_.agents[agent].taskPaths[taskPosition - 1].endTime();
      }
      assert(taskPosition <=
             (int)solution_.getAgentGlobalTasks(agent).size() - 1);

      ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
      buildConstraintTable(constraintTable, task);
      solution_.paths[task] =
          solution_.agents[agent].pathPlanner->findPathSegment(
              constraintTable, startTime, taskPosition, 0);
      solution_.agents[agent].taskPaths[taskPosition] = solution_.paths[task];

      // Once the path was found fix the begin times for subsequent tasks of the agent
      for (int k = taskPosition + 1;
           k < (int)solution_.getAgentGlobalTasks(agent).size(); k++) {
        solution_.agents[agent].taskPaths[k].beginTime =
            solution_.agents[agent].taskPaths[k - 1].endTime();
      }
    }
  }

  vector<int> agentsToCompute;
  for (pair<int, int> taskAgent : affectedAgents) {
    agentsToCompute.push_back(taskAgent.second);
  }
  solution_.joinPaths(agentsToCompute);
}

void LNS::computeRegret() {
  solution_.neighbor.regretMaxHeap.clear();
  for (int task : solution_.neighbor.conflictedTasks) {
    computeRegretForTask(task);
  }
}

void LNS::computeRegretForTask(int task) {
  pairing_heap<Utility, compare<Utility::CompareNode>> serviceTimes;

  // Find the precedence constraints involving the task or any other task that is not in the conflicting set
  vector<pair<int, int>> precedenceConstraints;
  for (pair<int, int> pc : solution_.precedenceConstraints) {
    if (pc.first == task || pc.second == task ||
        (std::find_if(solution_.neighbor.conflictedTasks.begin(),
                      solution_.neighbor.conflictedTasks.end(), [pc](int task) {
                        return pc.first == task || pc.second == task;
                      })) == solution_.neighbor.conflictedTasks.end()) {
      precedenceConstraints.push_back(pc);
    }
  }

  // Compute the set of tasks that are needed to complete before we can complete this task
  vector<int> successors;
  vector<vector<int>> ancestors;
  ancestors.resize(instance_.getTasksNum());
  for (pair<int, int> precedenceConstraint : precedenceConstraints) {
    if (precedenceConstraint.first == task) {
      successors.push_back(precedenceConstraint.second);
    }
    ancestors[precedenceConstraint.second].push_back(
        precedenceConstraint.first);
  }

  stack<int> q({task});
  unordered_set<int> previousTasks;
  int earliestTimestep = 0, latestTimestep = INT_MAX;
  PLOGD << "Finding earliest timestep for task " << task << endl;
  while (!q.empty()) {
    int current = q.top();
    q.pop();
    if (previousTasks.find(current) != previousTasks.end()) {
      continue;
    }
    previousTasks.insert(current);
    if (current != task && !solution_.paths[current].empty()) {
      int agent = solution_.getAgentWithTask(current);
      int taskIdx = solution_.getLocalTaskIndex(agent, current);
      if (earliestTimestep < solution_.agents[agent].path.timeStamps[taskIdx]) {
        PLOGD << "Going to update earliest timestep from " << earliestTimestep
              << endl;
        earliestTimestep = solution_.agents[agent].path.timeStamps[taskIdx];
        PLOGD << "New earliest timestep " << earliestTimestep << endl;
        PLOGD << "Came from " << agent << ", " << current << " at " << taskIdx
              << endl;
      }
    }
    for (int agentTaskAncestor : ancestors[current]) {
      if (previousTasks.find(agentTaskAncestor) == previousTasks.end()) {
        q.push(agentTaskAncestor);
      }
    }
  }
  previousTasks.erase(task);

  PLOGD << "Finding latest timestep for task " << task << endl;
  for (int succ : successors) {
    if (solution_.paths[succ].empty()) {
      continue;
    }
    int agent = solution_.getAgentWithTask(succ);
    int taskIdx = solution_.getLocalTaskIndex(agent, succ);
    if (latestTimestep > solution_.agents[agent].path.timeStamps[taskIdx]) {
      PLOGD << "Going to update latest timestep from " << latestTimestep
            << endl;
      latestTimestep = solution_.agents[agent].path.timeStamps[taskIdx];
      PLOGD << "New latest timestep " << latestTimestep << endl;
      PLOGD << "Came from " << agent << ", " << succ << " at " << taskIdx
            << endl;
    }
  }

  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    computeRegretForTaskWithAgent(task, agent, earliestTimestep, latestTimestep,
                                  &precedenceConstraints, &serviceTimes);
  }

  Utility bestUtility = serviceTimes.top();
  serviceTimes.pop();
  Utility secondBestUtility = serviceTimes.top();
  Regret regret(task, bestUtility.agent, bestUtility.taskPosition,
                secondBestUtility.value - bestUtility.value);
  solution_.neighbor.regretMaxHeap.push(regret);
}

void LNS::computeRegretForTaskWithAgent(
    int task, int agent, int earliestTimestep, int latestTimestep,
    vector<pair<int, int>>* precedenceConstraints,
    pairing_heap<Utility, compare<Utility::CompareNode>>* serviceTimes) {

  // Compute the first position along the agent's task assignments where we can insert this task
  int firstValidPosition = 0,
      lastValidPosition = solution_.getAgentGlobalTasks(agent).size() + 1;
  vector<int> agentTasks = solution_.getAgentGlobalTasks(agent);
  for (int j = (int)solution_.getAgentGlobalTasks(agent).size() - 1; j >= 0;
       j--) {
    if (solution_.agents[agent].path.timeStamps[j] <= earliestTimestep) {
      firstValidPosition = j + 1;
      break;
    }
  }
  // Compute the last position along the agent's task assignment where we can insert this task
  for (int j = 0; j < (int)solution_.getAgentGlobalTasks(agent).size(); j++) {
    if (solution_.agents[agent].path.timeStamps[j] >= latestTimestep) {
      lastValidPosition = j;
      break;
    }
  }

  assert(firstValidPosition >= 0);
  assert(lastValidPosition <=
         (int)solution_.getAgentGlobalTasks(agent).size() + 1);

  for (int j = firstValidPosition; j < lastValidPosition; j++) {

    // Compute the distance estimate it would take to finish the insertion
    int distance = 0;
    if (j > 0 && j < (int)solution_.getAgentGlobalTasks(agent).size()) {
      distance += instance_.getManhattanDistance(
          instance_.getTaskLocations(
              solution_.getAgentGlobalTasks(agent, j - 1)),
          instance_.getTaskLocations(task));
      distance += instance_.getManhattanDistance(
          instance_.getTaskLocations(task),
          instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent, j)));
    } else if (j == 0) {
      distance += instance_.getManhattanDistance(
          solution_.agents[agent].pathPlanner->startLocation,
          instance_.getTaskLocations(task));
    } else {
      distance += instance_.getManhattanDistance(
          instance_.getTaskLocations(
              solution_.getAgentGlobalTasks(agent, j - 1)),
          instance_.getTaskLocations(task));
    }

    // If the computed distance estimated is longer than the original path size then why bother
    if (distance > solution_.neighbor.conflictedTasksPathSize[task]) {
      continue;
    }

    vector<Path> taskPaths = solution_.paths;
    vector<vector<int>> taskAssignments = solution_.taskAssignments;
    vector<pair<int, int>> precConstraints = *precedenceConstraints;
    Utility utility = insertTask(task, agent, j, &taskPaths, &taskAssignments,
                                 &precConstraints);
    serviceTimes->push(utility);
  }
}

Utility LNS::insertTask(int task, int agent, int taskPosition,
                        vector<Path>* taskPaths,
                        vector<vector<int>>* taskAssignments,
                        vector<pair<int, int>>* precedenceConstraints,
                        bool commit) {

  double pathSizeChange = 0;
  int startTime = 0, previousTask = -1, nextTask = -1;

  vector<Path>& taskPathsRef = *taskPaths;
  vector<vector<int>>& taskAssignmentsRef = *taskAssignments;
  vector<pair<int, int>>& precConstraintsRef = *precedenceConstraints;

  int agentTasksSize = (int)taskAssignmentsRef[agent].size();

  // In this case we are inserting a task between two existing tasks
  if (taskPosition > 0 && taskPosition < agentTasksSize) {

    previousTask = taskAssignmentsRef[agent][taskPosition - 1];
    nextTask = taskAssignmentsRef[agent][taskPosition];

    startTime = taskPathsRef[previousTask].endTime();
    pathSizeChange = (double)taskPathsRef[nextTask].size();

    taskPathsRef[nextTask] = Path();  // This path will change

    taskAssignmentsRef[agent].insert(
        taskAssignmentsRef[agent].begin() + taskPosition, task);
    precConstraintsRef.erase(
        std::remove_if(precConstraintsRef.begin(), precConstraintsRef.end(),
                       [previousTask, nextTask](pair<int, int> x) {
                         return x.first == previousTask && x.second == nextTask;
                       }),
        precConstraintsRef.end());
    precConstraintsRef.emplace_back(previousTask, task);
    precConstraintsRef.emplace_back(task, nextTask);

    if (commit) {
      solution_.agents[agent].taskPaths[taskPosition] = Path();
      solution_.agents[agent].taskPaths.insert(
          solution_.agents[agent].taskPaths.begin() + taskPosition, Path());
    }

  } else if (taskPosition ==
             0) {  // In this case we are inserting at the very start

    nextTask = taskAssignmentsRef[agent][taskPosition];
    pathSizeChange = (double)taskPathsRef[nextTask].size();

    taskPathsRef[nextTask] = Path();

    taskAssignmentsRef[agent].insert(
        taskAssignmentsRef[agent].begin() + taskPosition, task);
    precConstraintsRef.emplace_back(task, nextTask);

    if (commit) {
      solution_.agents[agent].taskPaths[taskPosition] = Path();
      solution_.agents[agent].taskPaths.insert(
          solution_.agents[agent].taskPaths.begin() + taskPosition, Path());
    }

  } else if (taskPosition ==
             agentTasksSize) {  // In this case we are inserting at the very end

    previousTask = taskAssignmentsRef[agent][taskPosition - 1];
    startTime = taskPathsRef[previousTask].endTime();

    taskAssignmentsRef[agent].push_back(task);

    precConstraintsRef.emplace_back(previousTask, task);

    if (commit) {
      solution_.agents[agent].taskPaths.emplace_back();
    }
  }

  vector<int> goalLocations =
      instance_.getTaskLocations(taskAssignmentsRef[agent]);
  ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
  if (!commit) {
    MultiLabelSpaceTimeAStar localPlanner =
        MultiLabelSpaceTimeAStar(instance_, agent);
    localPlanner.setGoalLocations(goalLocations);
    localPlanner.computeHeuristics();

    buildConstraintTable(constraintTable, task,
                         instance_.getTaskLocations(task), taskPaths,
                         taskAssignments, precedenceConstraints);
    taskPathsRef[task] = localPlanner.findPathSegment(
        constraintTable, startTime, taskPosition, 0);

    if (nextTask != -1) {
      startTime = taskPathsRef[task].endTime();

      buildConstraintTable(constraintTable, nextTask,
                           instance_.getTaskLocations()[nextTask], taskPaths,
                           taskAssignments, precedenceConstraints);
      taskPathsRef[nextTask] = localPlanner.findPathSegment(
          constraintTable, startTime, taskPosition + 1, 0);
    }
  } else {

    solution_.agents[agent].pathPlanner->setGoalLocations(goalLocations);
    solution_.agents[agent].pathPlanner->computeHeuristics();

    buildConstraintTable(constraintTable, task);
    taskPathsRef[task] = solution_.agents[agent].pathPlanner->findPathSegment(
        constraintTable, startTime, taskPosition, 0);
    solution_.agents[agent].taskPaths[taskPosition] = taskPathsRef[task];

    if (nextTask != -1) {
      startTime = taskPathsRef[task].endTime();

      buildConstraintTable(constraintTable, nextTask);
      taskPathsRef[nextTask] =
          solution_.agents[agent].pathPlanner->findPathSegment(
              constraintTable, startTime, taskPosition + 1, 0);
      solution_.agents[agent].taskPaths[taskPosition + 1] =
          taskPathsRef[nextTask];
    }

    for (int k = taskPosition + 1;
         k < (int)solution_.getAgentGlobalTasks(agent).size(); k++) {
      solution_.agents[agent].taskPaths[k].beginTime =
          solution_.agents[agent].taskPaths[k - 1].endTime();
    }
  }

  if (!commit) {
    double value = -pathSizeChange + (double)taskPathsRef[task].size();
    if (nextTask != -1) {
      value += (double)taskPathsRef[nextTask].size();
    }

    Utility utility(agent, taskPosition, value);
    return utility;
  }
  return {};
}

void LNS::commitBestRegretTask(Regret bestRegret) {

  PLOGD << "Commiting for task " << bestRegret.task << " to agent "
        << bestRegret.agent << " with regret = " << bestRegret.value << endl;
  insertTask(bestRegret.task, bestRegret.agent, bestRegret.taskPosition,
             &solution_.paths, &solution_.taskAssignments,
             &solution_.precedenceConstraints, true);
  solution_.neighbor.conflictedTasks.erase(bestRegret.task);
}

void LNS::buildConstraintTable(ConstraintTable& constraintTable, int task,
                               int taskLocation, vector<Path>* paths,
                               vector<vector<int>>* taskAssignments,
                               vector<pair<int, int>>* precedenceConstraints) {
  constraintTable.goalLocation = taskLocation;

  vector<vector<int>> ancestors;
  ancestors.resize(instance_.getTasksNum());
  for (pair<int, int> precedenceConstraint : (*precedenceConstraints)) {
    ancestors[precedenceConstraint.second].push_back(
        precedenceConstraint.first);
  }

  unordered_set<int> setOfTasksToComplete;
  stack<int> q({task});
  while (!q.empty()) {
    int current = q.top();
    q.pop();
    if (setOfTasksToComplete.find(current) != setOfTasksToComplete.end()) {
      continue;
    }
    setOfTasksToComplete.insert(current);
    for (int agentTaskAncestor : ancestors[current]) {
      if (setOfTasksToComplete.find(agentTaskAncestor) ==
          setOfTasksToComplete.end()) {
        q.push(agentTaskAncestor);
      }
    }
  }
  setOfTasksToComplete.erase(task);

  for (int id : setOfTasksToComplete) {
    assert(!(*paths)[id].empty());
    int taskPosition = -1, taskSetSize = -1;
    for (int i = 0; i < instance_.getAgentNum(); i++) {
      for (int j = 0; j < (int)(*taskAssignments)[i].size(); j++) {
        if ((*taskAssignments)[i][j] == id) {
          taskPosition = j;
          taskSetSize = (int)(*taskAssignments)[i].size();
          break;
        }
      }
    }
    bool waitAtGoal = taskPosition == taskSetSize - 1;
    constraintTable.addPath((*paths)[id], waitAtGoal);
  }

  for (int agentTaskAncestor : ancestors[task]) {
    assert(!(*paths)[agentTaskAncestor].empty());
    constraintTable.lengthMin = max(constraintTable.lengthMin,
                                    (*paths)[agentTaskAncestor].endTime() + 1);
  }
  constraintTable.latestTimestep =
      max(constraintTable.latestTimestep, constraintTable.lengthMin);
}

void LNS::buildConstraintTable(ConstraintTable& constraintTable, int task) {
  constraintTable.goalLocation = instance_.getTaskLocations(task);

  vector<vector<int>> ancestors;
  ancestors.resize(instance_.getTasksNum());

  // Only care about tasks that are not in conflict right now
  for (pair<int, int> precedenceConstraint : solution_.precedenceConstraints) {
    if (std::find_if(solution_.neighbor.conflictedTasks.begin(),
                     solution_.neighbor.conflictedTasks.end(),
                     [precedenceConstraint](int task) {
                       return precedenceConstraint.first == task ||
                              precedenceConstraint.second == task;
                     }) != solution_.neighbor.conflictedTasks.end()) {
      continue;
    }
    ancestors[precedenceConstraint.second].push_back(
        precedenceConstraint.first);
  }

  unordered_set<int> setOfTasksToComplete;
  stack<int> q({task});
  while (!q.empty()) {
    int current = q.top();
    q.pop();
    if (setOfTasksToComplete.find(current) != setOfTasksToComplete.end()) {
      continue;
    }
    setOfTasksToComplete.insert(current);
    for (int agentTaskAncestor : ancestors[current]) {
      if (setOfTasksToComplete.find(agentTaskAncestor) ==
          setOfTasksToComplete.end()) {
        q.push(agentTaskAncestor);
      }
    }
  }
  setOfTasksToComplete.erase(task);

  for (int id : setOfTasksToComplete) {
    int agent = solution_.getAgentWithTask(id);
    int taskPosition = solution_.getLocalTaskIndex(agent, id);
    bool waitAtGoal =
        taskPosition == (int)solution_.getAgentGlobalTasks(agent).size() - 1;
    constraintTable.addPath(solution_.paths[id], waitAtGoal);
  }

  for (int agentTaskAncestor : ancestors[task]) {
    assert(!solution_.paths[agentTaskAncestor].empty());
    constraintTable.lengthMin =
        max(constraintTable.lengthMin,
            solution_.paths[agentTaskAncestor].endTime() + 1);
  }
  constraintTable.latestTimestep =
      max(constraintTable.latestTimestep, constraintTable.lengthMin);
}

bool LNS::validateSolution(set<int>* conflictedTasks) {

  bool result = true;

  // Check that the precedence constraints are not violated
  for (pair<int, int> precedenceConstraint : solution_.precedenceConstraints) {

    // Only care about tasks that are not in conflict
    if (std::find_if(solution_.neighbor.conflictedTasks.begin(),
                     solution_.neighbor.conflictedTasks.end(),
                     [precedenceConstraint](int task) {
                       return precedenceConstraint.first == task ||
                              precedenceConstraint.second == task;
                     }) != solution_.neighbor.conflictedTasks.end()) {
      continue;
    }

    int agentA = solution_.getAgentWithTask(precedenceConstraint.first),
        agentB = solution_.getAgentWithTask(precedenceConstraint.second);
    int taskPositionA =
            solution_.getLocalTaskIndex(agentA, precedenceConstraint.first),
        taskPositionB =
            solution_.getLocalTaskIndex(agentB, precedenceConstraint.second);

    if (solution_.agents[agentA].path.timeStamps[taskPositionA] >=
        solution_.agents[agentB].path.timeStamps[taskPositionB]) {
      PLOGE << "Temporal conflict between agent " << agentA << " doing task "
            << precedenceConstraint.first << " and agent " << agentB
            << " doing task " << precedenceConstraint.second << endl;
      result = false;
      if (conflictedTasks == nullptr) {
        return false;
      }
      conflictedTasks->insert(precedenceConstraint.first);
      conflictedTasks->insert(precedenceConstraint.second);
    }
  }

  for (int agentI = 0; agentI < instance_.getAgentNum(); agentI++) {
    for (int agentJ = 0; agentJ < instance_.getAgentNum(); agentJ++) {
      if (agentI == agentJ) {
        continue;
      }
      size_t minPathLength = solution_.agents[agentI].path.size() <
                                     solution_.agents[agentJ].path.size()
                                 ? solution_.agents[agentI].path.size()
                                 : solution_.agents[agentJ].path.size();
      for (int timestep = 0; timestep < (int)minPathLength; timestep++) {
        int locationAgentI =
            solution_.agents[agentI].path.at(timestep).location;
        int locationAgentJ =
            solution_.agents[agentJ].path.at(timestep).location;

        // Check that any two agents are not at the same location at the same timestep
        if (locationAgentI == locationAgentJ) {
          pair<int, int> coord = instance_.getCoordinate(locationAgentI);
          PLOGE << "Agents " << agentI << " and " << agentJ
                << " collide with each other at (" << coord.first << ", "
                << coord.second << ") at timestep " << timestep << endl;
          result = false;
          if (conflictedTasks == nullptr) {
            return false;
          }
          for (int taskIdx = 0;
               taskIdx < (int)solution_.getAgentGlobalTasks(agentI).size();
               taskIdx++) {
            if (solution_.agents[agentI].path.timeStamps[taskIdx] > timestep) {
              conflictedTasks->insert(
                  solution_.getAgentGlobalTasks(agentI, taskIdx));
              break;
            }
          }
          for (int taskIdx = 0;
               taskIdx < (int)solution_.getAgentGlobalTasks(agentJ).size();
               taskIdx++) {
            if (solution_.agents[agentJ].path.timeStamps[taskIdx] > timestep) {
              conflictedTasks->insert(
                  solution_.getAgentGlobalTasks(agentJ, taskIdx));
              break;
            }
          }
        }
        // Check that any two agents are not following the same edge in the opposite
        // direction at the same timestep
        else if (timestep < (int)minPathLength - 1 &&
                 locationAgentI ==
                     solution_.agents[agentJ].path.at(timestep + 1).location &&
                 locationAgentJ ==
                     solution_.agents[agentI].path.at(timestep + 1).location) {
          pair<int, int> coordI = instance_.getCoordinate(locationAgentI),
                         coordJ = instance_.getCoordinate(locationAgentJ);
          PLOGE << "Agents " << agentI << " and " << agentJ
                << " collide with each other at (" << coordI.first << ", "
                << coordI.second << ") --> (" << coordJ.first << ", "
                << coordJ.second << ") at timestep " << timestep << endl;
          result = false;
          if (conflictedTasks == nullptr) {
            return false;
          }
          for (int taskIdx = 0;
               taskIdx < (int)solution_.getAgentGlobalTasks(agentI).size();
               taskIdx++) {
            if (solution_.agents[agentI].path.timeStamps[taskIdx] > timestep) {
              conflictedTasks->insert(
                  solution_.getAgentGlobalTasks(agentI, taskIdx));
              break;
            }
          }
          for (int taskIdx = 0;
               taskIdx < (int)solution_.getAgentGlobalTasks(agentJ).size();
               taskIdx++) {
            if (solution_.agents[agentJ].path.timeStamps[taskIdx] > timestep) {
              conflictedTasks->insert(
                  solution_.getAgentGlobalTasks(agentJ, taskIdx));
              break;
            }
          }
        }
      }

      // Check that any two agents are not at the same location at the same timestep where
      // one agent might be waiting already
      if (solution_.agents[agentI].path.size() !=
          solution_.agents[agentJ].path.size()) {
        int smallerPathAgent = solution_.agents[agentI].path.size() <
                                       solution_.agents[agentJ].path.size()
                                   ? agentI
                                   : agentJ;
        int largerPathAgent = solution_.agents[agentI].path.size() <
                                      solution_.agents[agentJ].path.size()
                                  ? agentJ
                                  : agentI;
        int lastLocationOfSmallerPathAgent =
            solution_.agents[smallerPathAgent].path.back().location;
        for (int timestep = (int)minPathLength;
             timestep < (int)solution_.agents[largerPathAgent].path.size();
             timestep++) {
          int locationOfLargerPathAgent =
              solution_.agents[largerPathAgent].path.at(timestep).location;
          if (lastLocationOfSmallerPathAgent == locationOfLargerPathAgent) {
            pair<int, int> coord =
                instance_.getCoordinate(locationOfLargerPathAgent);
            PLOGE << "Agents " << agentI << " and " << agentJ
                  << " collide with each other at (" << coord.first << ", "
                  << coord.second << ") at timestep " << timestep << endl;
            result = false;
            if (conflictedTasks == nullptr) {
              return false;
            }
            if (solution_.agents[agentI].path.timeStamps
                    [(int)solution_.getAgentGlobalTasks(agentI).size() - 1] <
                timestep) {
              conflictedTasks->insert(solution_.getAgentGlobalTasks(
                  agentI,
                  (int)solution_.getAgentGlobalTasks(agentI).size() - 1));
            } else {
              for (int taskIdx = 0;
                   taskIdx < (int)solution_.getAgentGlobalTasks(agentI).size();
                   taskIdx++) {
                if (solution_.agents[agentI].path.timeStamps[taskIdx] >
                    timestep) {
                  conflictedTasks->insert(
                      solution_.getAgentGlobalTasks(agentI, taskIdx));
                  break;
                }
              }
            }
            if (solution_.agents[agentJ].path.timeStamps
                    [(int)solution_.getAgentGlobalTasks(agentJ).size() - 1] <
                timestep) {
              conflictedTasks->insert(solution_.getAgentGlobalTasks(
                  agentJ,
                  (int)solution_.getAgentGlobalTasks(agentJ).size() - 1));
            } else {
              for (int taskIdx = 0;
                   taskIdx < (int)solution_.getAgentGlobalTasks(agentJ).size();
                   taskIdx++) {
                if (solution_.agents[agentJ].path.timeStamps[taskIdx] >
                    timestep) {
                  conflictedTasks->insert(
                      solution_.getAgentGlobalTasks(agentJ, taskIdx));
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
  return result;
}

void LNS::printPaths() const {
  for (int i = 0; i < instance_.getAgentNum(); i++) {
    cout << "Agent " << i << " (cost = " << solution_.agents[i].path.size() - 1
         << "): ";
    cout << "\n\tPaths:\n\t";
    for (int t = 0; t < (int)solution_.agents[i].path.size(); t++) {
      pair<int, int> coord =
          instance_.getCoordinate(solution_.agents[i].path.at(t).location);
      cout << "(" << coord.first << ", " << coord.second << ")@" << t;
      if (solution_.agents[i].path.at(t).isGoal) {
        cout << "*";
      }
      if (i != (int)solution_.agents[i].path.size() - 1) {
        cout << " -> ";
      }
    }
    cout << endl;
    cout << "\tTimestamps:\n\t";
    for (int j = 0; j < (int)solution_.getAgentGlobalTasks(i).size(); j++) {
      pair<int, int> goalCoord = instance_.getCoordinate(
          solution_.agents[i].pathPlanner->goalLocations[j]);
      cout << "(" << goalCoord.first << ", " << goalCoord.second << ")@"
           << solution_.agents[i].path.timeStamps[j];
      if (j != (int)solution_.getAgentGlobalTasks(i).size() - 1) {
        cout << " -> ";
      }
    }
    cout << endl;
    cout << "\tTasks:\n\t";
    for (int j = 0; j < (int)solution_.getAgentGlobalTasks(i).size(); j++) {
      cout << solution_.getAgentGlobalTasks(i)[j];
      if (j != (int)solution_.getAgentGlobalTasks(i).size() - 1) {
        cout << " -> ";
      }
    }
    cout << endl;
  }
}
