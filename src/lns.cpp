#include "lns.hpp"
#include <numeric>
#include "common.hpp"
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

  vector<pair<int, int>> precedenceConstraints;
  for (pair<int, int> precConstraint :
       instance_.getInputPrecedenceConstraints()) {
    precedenceConstraints.emplace_back(precConstraint.first,
                                       precConstraint.second);
  }

  // Compute the precedence constraints based on current task assignments
  // Intra-agent precedence constraints
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    for (int task = 1; task < (int)solution_.getAgentGlobalTasks(agent).size();
         task++) {
      solution_.agents[agent].insertPrecedenceConstraint(
          solution_.agents[agent].taskAssignments[task - 1],
          solution_.agents[agent].taskAssignments[task]);
      precedenceConstraints.emplace_back(
          solution_.agents[agent].taskAssignments[task - 1],
          solution_.agents[agent].taskAssignments[task]);
    }
  }

  // Find paths based on the task assignments
  // First we need to sort the tasks based on the precedence constraints
  vector<int> planningOrder;
  bool success =
      topologicalSort(&instance_, &precedenceConstraints, planningOrder);
  if (!success) {
    PLOGE << "Topological sorting failed\n";
    return success;
  }

  // Following the topological order we find the paths for each task
  initialPaths.resize(instance_.getTasksNum(), Path());
  for (int id : planningOrder) {

    int agent = solution_.getAgentWithTask(id), task = id,
        taskPosition = solution_.getLocalTaskIndex(agent, task), startTime = 0;
    if (taskPosition != 0) {
      int previousTask =
          solution_.agents[agent].taskAssignments[taskPosition - 1];
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

  Neighbor lnsNeighborhood;

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
    previousSolution_ = solution_;
    lnsNeighborhood.conflictedTasks = conflictedTasks;

    prepareNextIteration(&lnsNeighborhood);
    for (int task : lnsNeighborhood.conflictedTasks) {
      lnsNeighborhood.commitedTasks.insert(make_pair(task, false));
    }

    // Compute regret for each of the tasks that are in the conflicting set
    // Pick the best one and repeat the whole process aboe
    while (!lnsNeighborhood.conflictedTasks.empty()) {
      computeRegret(&lnsNeighborhood);
      Regret bestRegret = lnsNeighborhood.regretMaxHeap.top();
      // Use the best regret task and insert it in its correct location
      commitBestRegretTask(bestRegret, &lnsNeighborhood);
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
          << lnsNeighborhood.conflictedTasks.size() << endl;
    PLOGD << "Number of conflicts in new solution: " << conflictedTasks.size()
          << endl;

    if (lnsNeighborhood.conflictedTasks.size() <
        conflictedTasks.size()) {
      // Reject this solution
      for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        delete solution_.agents[agent].pathPlanner;
      }
      solution_ = previousSolution_;
      PLOGD << "Rejecting this solution!\n";
    } else if (lnsNeighborhood.conflictedTasks.size() ==
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

void LNS::prepareNextIteration(Neighbor* neighbor) {
  PLOGI << "Preparing the solution object for the next iteration\n";

  vector<vector<int>> successors;
  successors.resize(instance_.getTasksNum());
  // Find the tasks that are following the earliest conflicting task as their paths need to be invalidated
  for (pair<int, int> precConstraint :
       instance_.getInputPrecedenceConstraints()) {
    if (neighbor->conflictedTasks.find(precConstraint.first) !=
        neighbor->conflictedTasks.end()) {
      successors[precConstraint.first].push_back(precConstraint.second);
    }
  }

  stack<int> q;
  for (int conflictTask : neighbor->conflictedTasks) {
    q.push(conflictTask);
  }
  while (!q.empty() && (int)neighbor->conflictedTasks.size() <= neighborSize_) {
    int current = q.top();
    q.pop();
    if (neighbor->conflictedTasks.find(current) != neighbor->conflictedTasks.end()) {
      continue;
    }
    neighbor->conflictedTasks.insert(current);
    for (int succ : successors[current]) {
      if (neighbor->conflictedTasks.find(succ) == neighbor->conflictedTasks.end()) {
        q.push(succ);
      }
    }
  }

  // If t_id is deleted then t_id + 1 task needs to be fixed
  vector<int> tasksToFix;
  unordered_map<int, int> affectedAgents;
  for (int invalidTask : neighbor->conflictedTasks) {

    int agent = solution_.getAgentWithTask(invalidTask),
        taskPosition = solution_.getLocalTaskIndex(agent, invalidTask),
        pathSize = (int)solution_.agents[agent].taskPaths[invalidTask]
                       .size();  // path_size is used for heuristic estimate
    PLOGD << "Invalidating task: " << invalidTask << ", Agent: " << agent << endl;

    // If the invalidated task was not the last local task of this agent then t_id + 1 exists
    if (taskPosition != (int)solution_.getAgentGlobalTasks(agent).size() - 1) {
      int nextTask = solution_.getAgentGlobalTasks(agent, taskPosition + 1);
      if (neighbor->conflictedTasks.find(nextTask) == neighbor->conflictedTasks.end()) {
        tasksToFix.push_back(nextTask);
      }
      PLOGD << "Next task: " << nextTask << endl;
    }

    affectedAgents.insert(make_pair(invalidTask, agent));
    neighbor->conflictedTasksPathSize.insert(make_pair(invalidTask, pathSize));

    // Marking past information about this conflicting task
    solution_.taskAgentMap[invalidTask] = UNASSIGNED;
    solution_.agents[agent].clearIntraAgentPrecedenceConstraint(invalidTask);
    // Needs to happen after clearing precedence constraints
    solution_.agents[agent].taskAssignments[taskPosition] = -1;
    solution_.agents[agent].taskPaths[taskPosition] = Path();
  }

  // Marking past information about conflicting tasks
  for (pair<int, int> taskAgent : affectedAgents) {
    int agent = taskAgent.second;

    // For an affected agent there can be multiple conflicting tasks so need to do it this way
    solution_.agents[agent].path = Path();
    solution_.agents[agent].taskAssignments.erase(
        std::remove_if(solution_.agents[agent].taskAssignments.begin(),
                       solution_.agents[agent].taskAssignments.end(),
                       [](int task) { return task == -1; }),
        solution_.agents[agent].taskAssignments.end());
    solution_.agents[agent].taskPaths.erase(
        std::remove_if(solution_.agents[agent].taskPaths.begin(),
                       solution_.agents[agent].taskPaths.end(),
                       [](const Path& p) {
                         return isSamePath(p, Path());
                       }),
        solution_.agents[agent].taskPaths.end());

    vector<int> taskLocations =
        instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
    solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[agent].pathPlanner->computeHeuristics();
  }

  // Find the paths for the tasks whose previous tasks were removed
  for (int task : instance_.getInputPlanningOrder()) {
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
      solution_.agents[agent].taskPaths[taskPosition] =
          solution_.agents[agent].pathPlanner->findPathSegment(
              constraintTable, startTime, taskPosition, 0);

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

void LNS::computeRegret(Neighbor* neighbor) {
  neighbor->regretMaxHeap.clear();
  for (int conflictTask : neighbor->conflictedTasks) {
    computeRegretForTask(conflictTask, neighbor);
  }
}

void LNS::computeRegretForTask(int task, Neighbor* neighbor) {
  // TODO: We might need to keep track of these service times later!
  pairing_heap<Utility, compare<Utility::CompareNode>> serviceTimes;

  // v2: The task has to start after the earliest time step but needs to finish before the latest time step
  int earliestTimestep = 0, latestTimestep = INT_MAX;
  vector<int> inputPlanningOrder = instance_.getInputPlanningOrder();
  for (int i = 1; i < (int)inputPlanningOrder.size() - 1; i++) {
    if (inputPlanningOrder[i] == task) {
      int previousTask = inputPlanningOrder[i - 1], nextTask = inputPlanningOrder[i + 1];
      // If the previous task was in the conflict set then if we have committed its path, we check the solution object otherwise the previous solution object
      if (neighbor->commitedTasks.count(previousTask) > 0 && !neighbor->commitedTasks[previousTask]) {
          int previousTaskAgent = previousSolution_.taskAgentMap[previousTask], previousTaskLocalIndex = previousSolution_.getLocalTaskIndex(previousTaskAgent, previousTask);
          assert(!previousSolution_.agents[previousTaskAgent].taskPaths[previousTaskLocalIndex].empty());
          earliestTimestep = max(earliestTimestep, previousSolution_.agents[previousTaskAgent].taskPaths[previousTaskLocalIndex].endTime());
      }
      else {
          int previousTaskAgent = solution_.taskAgentMap[previousTask];
          assert(previousTaskAgent != UNASSIGNED);
          int previousTaskLocalIndex = solution_.getLocalTaskIndex(previousTaskAgent, previousTask);
          earliestTimestep = max(earliestTimestep, solution_.agents[previousTaskAgent].taskPaths[previousTaskLocalIndex].endTime());
      }
      
      // If the next task was in conflict set then we dont need latest time step. Otherwise that path would be in the solution object
      if (neighbor->conflictedTasks.find(nextTask) == neighbor->conflictedTasks.end()) {
        int nextTaskAgent = solution_.taskAgentMap[nextTask];
        assert(nextTaskAgent != UNASSIGNED);
        int nextTaskPosition = solution_.getLocalTaskIndex(nextTaskAgent, nextTask);
        latestTimestep = min(latestTimestep, solution_.agents[nextTaskAgent].taskPaths[nextTaskPosition].beginTime);
      }
      break;
    }
  }

  // TODO: Need to gather the paths of all the tasks before this task! Not just agent specific tasks paths. Otherwise we cannot build the constraint table.
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    // Gather the task assignments of this agent with any previous invalidated tasks included as needed relative to the target task
    vector<int> temporaryTaskAssignments = solution_.getAgentGlobalTasks(agent);
    int taskIndex = find(inputPlanningOrder.begin(), inputPlanningOrder.end(), task) - inputPlanningOrder.begin();
    for (int i = 0; i < taskIndex; i++) {
      if (neighbor->conflictedTasks.find(inputPlanningOrder[i]) != neighbor->conflictedTasks.end()) {
        if (!neighbor->commitedTasks[inputPlanningOrder[i]] && previousSolution_.taskAgentMap[inputPlanningOrder[i]] == agent) {
          int localTaskPosition = previousSolution_.getLocalTaskIndex(agent, inputPlanningOrder[i]);
          temporaryTaskAssignments.insert(temporaryTaskAssignments.begin() + localTaskPosition, inputPlanningOrder[i]);
        }
      } 
    }

    // Gather the corresponding task paths and stitch them together
    vector<Path> temporaryTaskPaths;
    for (int i = 0; i < (int)temporaryTaskAssignments.size(); i++) {
      int temporaryTask = temporaryTaskAssignments[i];
      if (neighbor->commitedTasks.count(temporaryTask) > 0 && !neighbor->commitedTasks[temporaryTask]) {
          int temporaryTaskAgent = previousSolution_.taskAgentMap[temporaryTask];
          assert(temporaryTaskAgent != UNASSIGNED);
          int temporaryTaskLocalIndex = solution_.getLocalTaskIndex(temporaryTaskAgent, temporaryTask);
          Path temporaryTaskPath = previousSolution_.agents[temporaryTaskAgent].taskPaths[temporaryTaskLocalIndex];
          // Fix the begin time such that this task starts when the previous task ends
          if (i != 0) {
            temporaryTaskPath.beginTime = temporaryTaskPaths[i - 1].endTime();
          }
          temporaryTaskPaths.push_back(temporaryTaskPath);
      }
      else {
        int temporaryTaskAgent = solution_.taskAgentMap[temporaryTask];
        assert(temporaryTaskAgent != UNASSIGNED);
        int temporaryTaskLocalIndex = solution_.getLocalTaskIndex(temporaryTaskAgent, temporaryTask);
        Path temporaryTaskPath = solution_.agents[temporaryTaskAgent].taskPaths[temporaryTaskLocalIndex];
        // Fix the begin time such that this task starts when the previous task ends
        if (i != 0) {
          temporaryTaskPath.beginTime = temporaryTaskPaths[i - 1].endTime();
        }
        temporaryTaskPaths.push_back(temporaryTaskPath);
      }
    }
    ValidTimeRanges taskTimeRange = {earliestTimestep, latestTimestep};
    TaskRegretPacket regretPacket = {taskTimeRange, task, agent, -1};
    computeRegretForTaskWithAgent(regretPacket, &temporaryTaskAssignments, &temporaryTaskPaths, neighbor, &serviceTimes);
  }

  Utility bestUtility = serviceTimes.top();
  serviceTimes.pop();
  Utility secondBestUtility = serviceTimes.top();
  Regret regret(task, bestUtility.agent, bestUtility.taskPosition,
                secondBestUtility.value - bestUtility.value);
  neighbor->regretMaxHeap.push(regret);
}

void LNS::computeRegretForTaskWithAgent(
    TaskRegretPacket regretPacket, vector<int>* taskAssignments, vector<Path>* taskPaths,
    Neighbor* neighbor, pairing_heap<Utility, compare<Utility::CompareNode>>* serviceTimes) {
    
  // v2: Append the agent's internal precedence constraints to the input precedence constraints
  vector<pair<int, int>> precedenceConstraints = instance_.getInputPrecedenceConstraints();
  for (int i = 0; i < (int)taskAssignments->size() - 1; i++) {
    precedenceConstraints.emplace_back((*taskAssignments)[i], (*taskAssignments)[i + 1]);
  }

  // Compute the first position along the agent's task assignments where we can insert this task
  int firstValidPosition = 0, lastValidPosition = taskAssignments->size() + 1;
  for (int j = (int)taskAssignments->size() - 1; j >= 0; j--) {
    if ((*taskPaths)[j].endTime() <= regretPacket.timeRange.earliestTimestep) {
      firstValidPosition = j + 1;
      break;
    }
  }
  // Compute the last position along the agent's task assignment where we can insert this task
  for (int j = 0; j < (int)taskAssignments->size(); j++) {
    if ((*taskPaths)[j].endTime() >= regretPacket.timeRange.latestTimestep) {
      lastValidPosition = j; // Could be j + 1 but then need to change the loop to not be inclusive
      break;
    }
  }

  assert(firstValidPosition >= 0);
  assert(lastValidPosition <= (int)taskAssignments->size());

  for (int j = firstValidPosition; j <= lastValidPosition; j++) {

    // Compute the distance estimate it would take to finish the insertion
    int distance = 0;
    if (j > 0) {
      distance += instance_.getManhattanDistance(instance_.getTaskLocations((*taskAssignments)[j - 1]), instance_.getTaskLocations(regretPacket.task));
    }
    else {
     distance += instance_.getManhattanDistance(solution_.agents[regretPacket.agent].pathPlanner->startLocation, instance_.getTaskLocations(regretPacket.task)); 
    }

    // If the computed distance estimated is longer than the original path size then why bother
    if (distance > neighbor->conflictedTasksPathSize[regretPacket.task]) {
      continue;
    }

    regretPacket.taskPosition = j;
    vector<Path> temporaryTaskPaths = *taskPaths;
    vector<int> temporaryTaskAssignments = *taskAssignments;
    vector<pair<int, int>> temporaryPrecedenceConstraints = precedenceConstraints;
    // Create a copy of the task assignments, paths and corresponding precedence constraints so that they dont get modified
    Utility utility = insertTask(regretPacket, &temporaryTaskPaths, &temporaryTaskAssignments,
                                 &temporaryPrecedenceConstraints);
    serviceTimes->push(utility);
  }
}

// Need the task paths, assignments and precedence constraints as pointers so that we can reuse this code when commiting as we can make in-place changes to these data-structures
Utility LNS::insertTask(TaskRegretPacket regretPacket,
                        vector<Path>* taskPaths,
                        vector<int>* taskAssignments,
                        vector<pair<int, int>>* precedenceConstraints,
                        bool commit) {

  double pathSizeChange = 0;
  int startTime = 0, previousTask = -1, nextTask = -1;

  vector<Path>& taskPathsRef = *taskPaths;
  vector<int>& taskAssignmentsRef = *taskAssignments;
  vector<pair<int, int>>& precedenceConstraintsRef = *precedenceConstraints;

  int agentTasksSize = (int)taskAssignmentsRef.size();

  // In this case we are inserting a task between two existing tasks
  if (regretPacket.taskPosition < agentTasksSize) {

    pathSizeChange = (double)taskPathsRef[nextTask].size();

    taskPathsRef[regretPacket.taskPosition] = Path();  // This path will change

    taskAssignmentsRef.insert(
        taskAssignmentsRef.begin() + regretPacket.taskPosition, regretPacket.task);
    
    nextTask = taskAssignmentsRef[regretPacket.taskPosition];
    precedenceConstraintsRef.emplace_back(regretPacket.task, nextTask);

    // If we are NOT inserting at the start position then we need to take care of the previous task as well
    if (regretPacket.taskPosition != 0) {
      startTime = taskPathsRef[regretPacket.taskPosition - 1].endTime();
      previousTask = taskAssignmentsRef[regretPacket.taskPosition - 1];
      precedenceConstraintsRef.erase(
          std::remove_if(precedenceConstraintsRef.begin(), precedenceConstraintsRef.end(),
                        [previousTask, nextTask](pair<int, int> x) {
                          return x.first == previousTask && x.second == nextTask;
                        }),
          precedenceConstraintsRef.end());
      precedenceConstraintsRef.emplace_back(previousTask, regretPacket.task);
    }

    if (commit) {
      // Invalidate the path of the next task
      taskPathsRef[regretPacket.taskPosition] = Path();
      // Insert an empty path between previous and next tasks
      taskPathsRef.insert(taskPathsRef.begin() + regretPacket.taskPosition, Path());
    }

  }
  // In this case we are inserting at the very end
  else if (regretPacket.taskPosition == agentTasksSize) {  

    previousTask = taskAssignmentsRef[regretPacket.taskPosition - 1];
    startTime = taskPathsRef[regretPacket.taskPosition - 1].endTime();

    taskAssignmentsRef.push_back(regretPacket.task);

    precedenceConstraintsRef.emplace_back(previousTask, regretPacket.task);

    if (commit) {
      taskPathsRef.emplace_back();
    }
  }

  vector<int> goalLocations = instance_.getTaskLocations(taskAssignmentsRef);
  ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
  if (!commit) {
    MultiLabelSpaceTimeAStar localPlanner = MultiLabelSpaceTimeAStar(instance_, regretPacket.agent);
    localPlanner.setGoalLocations(goalLocations);
    localPlanner.computeHeuristics();

    buildConstraintTable(constraintTable, regretPacket.task,
                         goalLocations[regretPacket.taskPosition], &taskPathsRef,
                         &taskAssignmentsRef, &precedenceConstraintsRef);
    taskPathsRef[regretPacket.taskPosition] = localPlanner.findPathSegment(
        constraintTable, startTime, regretPacket.taskPosition, 0);

    if (nextTask != -1) {
      startTime = taskPathsRef[regretPacket.taskPosition].endTime();

      buildConstraintTable(constraintTable, nextTask,
                           goalLocations[regretPacket.taskPosition + 1], &taskPathsRef,
                           &taskAssignmentsRef, &precedenceConstraintsRef);
      taskPathsRef[regretPacket.taskPosition + 1] = localPlanner.findPathSegment(
          constraintTable, startTime, regretPacket.taskPosition + 1, 0);
    }
  } else {

    solution_.agents[regretPacket.agent].pathPlanner->setGoalLocations(goalLocations);
    solution_.agents[regretPacket.agent].pathPlanner->computeHeuristics();

    buildConstraintTable(constraintTable, regretPacket.task);
    taskPathsRef[regretPacket.taskPosition] = solution_.agents[regretPacket.agent].pathPlanner->findPathSegment(
        constraintTable, startTime, regretPacket.taskPosition, 0);

    if (nextTask != -1) {
      startTime = taskPathsRef[regretPacket.taskPosition].endTime();

      buildConstraintTable(constraintTable, nextTask);
      taskPathsRef[regretPacket.taskPosition + 1] =
          solution_.agents[regretPacket.agent].pathPlanner->findPathSegment(
              constraintTable, startTime, regretPacket.taskPosition + 1, 0);
    }

    for (int k = regretPacket.taskPosition + 1; k < (int)taskAssignmentsRef.size(); k++) {
      taskPathsRef[k].beginTime = taskPathsRef[k - 1].endTime();
    }
  }

  if (!commit) {
    double value = -pathSizeChange + (double)taskPathsRef[regretPacket.taskPosition].size();
    if (nextTask != -1) {
      value += (double)taskPathsRef[regretPacket.taskPosition].size();
    }

    Utility utility(regretPacket.agent, regretPacket.taskPosition, value);
    return utility;
  }
  return {};
}

void LNS::commitBestRegretTask(Regret bestRegret, Neighbor *neighbor) {

  PLOGD << "Commiting for task " << bestRegret.task << " to agent " << bestRegret.agent << " with regret = " << bestRegret.value << endl;
  TaskRegretPacket bestRegretPacket = {{}, bestRegret.task, bestRegret.agent, bestRegret.taskPosition};
  vector<pair<int, int>> precedenceConstraints = instance_.getInputPrecedenceConstraints();
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    precedenceConstraints.insert(precedenceConstraints.end(), solution_.agents[agent].intraPrecedenceConstraints.begin(), solution_.agents[agent].intraPrecedenceConstraints.end());
  }
  insertTask(bestRegretPacket,
             &solution_.agents[bestRegretPacket.agent].taskPaths, &solution_.agents[bestRegretPacket.agent].taskAssignments,
             &precedenceConstraints, true);
  neighbor->conflictedTasks.erase(bestRegret.task);
}

void LNS::buildConstraintTable(ConstraintTable& constraintTable, int task,
                               int taskLocation, vector<Path>* taskPaths,
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
      if (setOfTasksToComplete.find(agentTaskAncestor) == setOfTasksToComplete.end()) {
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
