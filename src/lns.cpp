#include "lns.hpp"
#include <boost/process.hpp>
#include <boost/process/pipe.hpp>
#include <numeric>
#include <optional>
#include <utility>
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

set<int> extractNConflicts(int size, const set<int>& conflicts) {
  int i = 0;
  set<int> result;
  for (int conflict : conflicts) {
    if (i >= size) {
      break;
    }
    result.insert(conflict);
    i++;
  }
  return result;
}

LNS::LNS(int numOfIterations, const Instance& instance, int neighborSize,
         double timeLimit, string initialStrategy)
    : numOfIterations_(numOfIterations),
      neighborSize_(neighborSize),
      instance_(instance),
      solution_(instance),
      previousSolution_(instance),
      timeLimit_(timeLimit),
      initialSolutionStrategy(std::move(initialStrategy)) {
  plannerStartTime_ = Time::now();
}

bool LNS::buildGreedySolutionWithMAPFPC(const string& variant) {

  initialPaths.resize(instance_.getTasksNum(), Path());
  bool readingTaskAssignments = false, readingTaskPaths = false;

  string solver;
  if (variant == "sota_cbs") {
    solver = "CBS";
  }
  else if (variant == "sota_pbs") {
    solver = "PBS";
  }
  else {
    PLOGE << "Initial solution solver using MAPF-PC variant not supported\n";
    return false;
  }

  // The path to the command when you use the vscode launch file
  string command = "./MAPF-PC/build/bin/task_assignment -m " +
                   instance_.getMapName() + " -a " +
                   instance_.getAgentTaskFName() + " -k " +
                   std::to_string(instance_.getAgentNum()) + " -t " +
                   std::to_string(instance_.getTasksNum()) + " --solver " + solver;

  // Run a child process to spawn the MAPC-PC codebase with the current map and agent informations
  namespace bp = boost::process;
  bp::ipstream inputStream;
  bp::child child(command, bp::std_out > inputStream);

  // The output sequence of the MAPF-PC codebase is as follows:
  // 1. Output TASK ASSIGNMENTS
  // Output Agent # and then followed by the task sequences in order
  // 2. Output some other internal stuff
  // 3. Output TASK PATHS
  // Output Agent # and then followed by the task locations (non-linearized) with @ after the first location with begin time right after and -> between each location

  PLOGD << "Exit code of MAPF-PC " << child.exit_code() << endl;
  int agent = -1;
  string line;
  while (std::getline(inputStream, line) && !line.empty()) {

    // If the agent variable exceeds the total number of agents we are working with then we have read all the task assignments or all the task paths
    if (agent >= instance_.getAgentNum()) {
      if (readingTaskAssignments) {
        readingTaskAssignments = false;
      } else if (readingTaskPaths) {
        readingTaskPaths = false;
      }
    }

    // Check if the following set of lines will be for task assignments or task paths
    if (strcmp(line.c_str(), "TASK ASSIGNMENTS") == 0) {
      readingTaskAssignments = true;
    } else if (strcmp(line.c_str(), "TASK PATHS") == 0) {
      agent = -1;
      readingTaskPaths = true;
    }

    // Use the Agent # to increment the agent variable
    if (line.find("Agent") != string::npos) {
      agent++;
    }
    // Otherwise if we are supposed to read the task assignments then we split that line using ',' as the delimiter and extract the tokens one by one
    // Eg: Agent 1
    //     1, 2, 3, 4,
    else if (readingTaskAssignments && agent > -1) {
      string token;
      size_t pos = 0;
      while ((pos = line.find(',')) != string::npos) {
        token = line.substr(0, pos);
        solution_.assignTaskToAgent(agent, stoi(token));
        line.erase(0, pos + 1);
      }
      solution_.agents[agent].taskPaths.resize(
          solution_.agents[agent].taskAssignments.size(), Path());
    }
    // If we are not reading the task assignments then we must be reading the task paths.
    // Eg: Agent 1
    //     6 @ 0 -> 22 -> 23 @ 2 -> 24 @ 3 -> 25 -> 26 ->
    else if (readingTaskPaths && agent > -1) {
      string token;
      Path taskPath;
      size_t pos = 0;
      int taskIndex = -1;
      while ((pos = line.find("->")) != string::npos) {
        token = line.substr(0, pos);
        if (token.find('@') != string::npos) {
          // This location is the first location for this task as it contains the "@" character after which we will have this task's begin time
          // Split the token to get the location and the begin time and use that

          // If the taskPath variable is not empty then add that taskPath to the solution object as that pertains to the previous task
          if (!taskPath.empty()) {
            solution_.agents[agent].taskPaths[taskIndex] = taskPath;
            initialPaths[solution_.agents[agent].taskAssignments[taskIndex]] =
                taskPath;
            taskPath = Path();
          }
          taskIndex++;
          size_t localPos = 0;
          string localToken;
          localPos = token.find('@');
          PathEntry pEntry = {false, stoi(token.substr(0, localPos))};

          // If there was a previous task path then the start location of this task would be what was the last location of that previous task
          if (taskIndex > 0) {
            PathEntry previousPEntry = {false, solution_.agents[agent]
                                                   .taskPaths[taskIndex - 1]
                                                   .back()
                                                   .location};
            taskPath.path.push_back(previousPEntry);
          }

          taskPath.path.push_back(pEntry);
          token.erase(0, localPos + 1);
          if (taskIndex > 0) {
            // Leftover token should now be the begin time information
            taskPath.beginTime = stoi(token) - 1;
          } else {
            taskPath.beginTime = 0;
          }
          // If there was a previus task then mark the last location of that task as goal
          if (taskIndex > 0) {
            solution_.agents[agent]
                .taskPaths[taskIndex - 1]
                .path[solution_.agents[agent]
                          .taskPaths[taskIndex - 1]
                          .path.size() -
                      1]
                .isGoal = true;
          }
        } else {
          PathEntry pEntry = {false, stoi(token)};
          taskPath.path.push_back(pEntry);
        }
        line.erase(0, pos + 2);
      }
      // Add the last task path
      if (!taskPath.empty()) {
        solution_.agents[agent].taskPaths[taskIndex] = taskPath;
        initialPaths[solution_.agents[agent].taskAssignments[taskIndex]] =
            taskPath;
        taskPath = Path();
      }
    }
  }

  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    vector<int> taskLocations =
        instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
    solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[agent].pathPlanner->computeHeuristics();
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

void LNS::extractFeasibleSolution() {

  incumbentSolution_.numOfCols = instance_.numOfCols;
  incumbentSolution_.sumOfCosts = solution_.sumOfCosts;
  if ((int)incumbentSolution_.agentPaths.size() == 0) {
    incumbentSolution_.agentPaths.resize(instance_.getAgentNum());
  }
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    if (!solution_.agents[agent].taskAssignments.empty()) {
      incumbentSolution_.agentPaths[agent] = solution_.agents[agent].path;
    }
    else {
      incumbentSolution_.agentPaths[agent] = Path();
    }
  }
}


bool LNS::run() {

  bool success = false;
  if (initialSolutionStrategy == "greedy") {
    // Run the greedy task assignment and subsequent path finding algorithm
    success = buildGreedySolution();
  } else if (initialSolutionStrategy.find("sota") != string::npos) {
    // Run the greedy task assignment and use CBS-PC for finding the paths of agents
    success = buildGreedySolutionWithMAPFPC(initialSolutionStrategy);
  }

  // If the initial solution strategy failed then we cannot do anything!
  if (!success) {
    return success;
  }

  printPaths();

  initialSolutionRuntime_ = ((fsec)(Time::now() - plannerStartTime_)).count();
  runtime = initialSolutionRuntime_;

  PLOGD << "Initial solution cost = " << solution_.sumOfCosts
        << ", Runtime = " << initialSolutionRuntime_ << endl;

  set<int> conflictedTasks;
  bool valid = validateSolution(&conflictedTasks);
  lnsNeighborhood.conflictedTasks =
      extractNConflicts(neighborSize_, conflictedTasks);

  bool feasibleSolutionUpdated = false;
  if (valid) {
    feasibleSolutionUpdated = true;
    extractFeasibleSolution();
  }

  iterationStats.emplace_back(initialSolutionRuntime_, "greedy",
                              instance_.getAgentNum(), instance_.getTasksNum(),
                              solution_.sumOfCosts, feasibleSolutionUpdated);
  set<int> oldConflictedTasks = lnsNeighborhood.conflictedTasks;

  // LNS loop
  while (runtime < timeLimit_) {

    int oldSolutionConflictNum;
    lnsNeighborhood.patchedTasks.clear();
    lnsNeighborhood.regretMaxHeap.clear();
    lnsNeighborhood.commitedTasks.clear();
    lnsNeighborhood.conflictedTasksPathSize.clear();

    // If we rejected the solution in the previous iteration then we need to reset the conflicted tasks!
    if (numOfFailures > 0) {
      lnsNeighborhood.conflictedTasks = oldConflictedTasks;
    } else {
      oldConflictedTasks = lnsNeighborhood.conflictedTasks;
    }

    // This is the case where we accepted the solution at the end of the loop in the previous iteration
    if (numOfFailures == 0) {
      // If we rejected in the previous iteration then we can reuse the previous iteration computations!
      // We can only clear the service time heap map here as at this point we are certain that we wont need it anymore
      lnsNeighborhood.serviceTimesHeapMap.clear();

      // Note: Do not clear conflictedTasks here since they get assigned at the end of the loop in the previous iteration

      previousSolution_ = solution_;

      // This accounts only for the tasks that were in conflict but not the tasks whose paths would also have been invalidated.
      // We now account for the additional tasks added to keep track of whether its the first iteration of this LNS loop
      oldSolutionConflictNum = (int)lnsNeighborhood.conflictedTasks.size();
    }

    PLOGD << "Printing neighborhood conflict tasks\n";
    PLOGD << "Size: " << lnsNeighborhood.conflictedTasks.size() << "\n";
    for (int conflictTask : lnsNeighborhood.conflictedTasks) {
      PLOGD << "Conflicted Task : " << conflictTask << "\n";
    }

    prepareNextIteration();

    // This needs to happen after prepare iteration since we updated the conflictedTasks variable in the prepare next iteration function
    for (int task : lnsNeighborhood.conflictedTasks) {
      lnsNeighborhood.commitedTasks.insert(make_pair(task, false));
    }
    lnsNeighborhood.immutableConflictedTasks = lnsNeighborhood.conflictedTasks;

    // Compute regret for each of the tasks that are in the conflicting set
    // Pick the best one and repeat the whole process again
    while (!lnsNeighborhood.conflictedTasks.empty()) {
      computeRegret((int)lnsNeighborhood.conflictedTasks.size() - lnsNeighborhood.additionalTasksAdded ==
                    oldSolutionConflictNum);
      Regret bestRegret = lnsNeighborhood.regretMaxHeap.top();
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

    // Compute the updated sum of costs
    solution_.sumOfCosts = 0;
    for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
      if (!solution_.agents[agent].taskAssignments.empty()) {
        solution_.sumOfCosts +=
            solution_.agents[agent].taskPaths.back().endTime();
      }
    }

    PLOGD << "Old sum of costs = " << previousSolution_.sumOfCosts << endl;
    PLOGD << "New sum of costs = " << solution_.sumOfCosts << endl;

    // Accept the solution only if the new one has lower number of conflicts or it has lower
    // cost of the solution

    // Extract the set of conflicting tasks
    conflictedTasks.clear();
    valid = validateSolution(&conflictedTasks);

    PLOGD << "Number of conflicts in old solution: " << oldSolutionConflictNum
          << endl;
    PLOGD << "Number of conflicts in new solution: " << conflictedTasks.size()
          << endl;

    if (!valid) {
      // Solution was not valid as we found some conflicts!
      feasibleSolutionUpdated = false;
      PLOGE << "The solution was not valid!\n";
    }
    else {
      extractFeasibleSolution();
      feasibleSolutionUpdated = true;
    }

    if (oldSolutionConflictNum < (int)conflictedTasks.size()) {
      // Reject this solution
      solution_ = previousSolution_;
      numOfFailures++;
      PLOGD << "Rejecting this solution!\n";
    } else if (oldSolutionConflictNum == (int)conflictedTasks.size() && oldSolutionConflictNum != 0) {
      if (previousSolution_.sumOfCosts <= solution_.sumOfCosts) {

        double acceptanceProb =
            exp((previousSolution_.sumOfCosts - solution_.sumOfCosts) /
                saTemperature);
        if ((double)rand() / (RAND_MAX) > acceptanceProb) {
          // Use simulated annealing to potentially accept worse solutions!
          numOfFailures = 0;
          lnsNeighborhood.conflictedTasks =
              extractNConflicts(neighborSize_, conflictedTasks);
        } else {
          // Reject this solution
          solution_ = previousSolution_;
          numOfFailures++;
          PLOGD << "Rejecting this solution!\n";
        }
      } else {
        // Accept this solution
        numOfFailures = 0;
        lnsNeighborhood.conflictedTasks =
            extractNConflicts(neighborSize_, conflictedTasks);
      }
    } else {
      // Accept this solution
      numOfFailures = 0;
      lnsNeighborhood.conflictedTasks =
          extractNConflicts(neighborSize_, conflictedTasks);
    }

    runtime = ((fsec)(Time::now() - plannerStartTime_)).count();
    iterationStats.emplace_back(runtime, "LNS", instance_.getAgentNum(),
                                instance_.getTasksNum(), solution_.sumOfCosts, feasibleSolutionUpdated);
    saTemperature *= saCoolingCoefficient;
  }

  printPaths();

  std::cout << "MAPF-PC-LNS: "
            << "\n\tRuntime = " << runtime
            << "\n\tIterations = " << iterationStats.size()
            << "\n\tSolution Cost = " << solution_.sumOfCosts
            << "\n\tNumber of failures = " << numOfFailures << endl;

  return valid;
}

void LNS::prepareNextIteration() {
  PLOGI << "Preparing the solution object for the next iteration\n";

  // Find the tasks that are following the earliest conflicting task as their paths need to be invalidated
  vector<vector<int>> successors = instance_.getSuccessors();
  lnsNeighborhood.additionalTasksAdded = (int)lnsNeighborhood.conflictedTasks.size();

  // We need to include all the successors of the original conflicted tasks to ensure that we dont try to find their paths later down the line because otherwise we will face errors since the ancestors of those successor tasks wont have paths.
  for (int conflictTask : lnsNeighborhood.conflictedTasks) {
    set<int> successorsOfConflictTask = reachableSet(conflictTask, successors);
    lnsNeighborhood.conflictedTasks.merge(successorsOfConflictTask);
  }
  lnsNeighborhood.additionalTasksAdded = (int)lnsNeighborhood.conflictedTasks.size() - lnsNeighborhood.additionalTasksAdded;

  // If t_id is deleted then t_id + 1 task needs to be fixed
  set<int> tasksToFix, affectedAgents;
  for (int invalidTask : lnsNeighborhood.conflictedTasks) {

    int agent = solution_.getAgentWithTask(invalidTask),
        taskPosition = solution_.getLocalTaskIndex(agent, invalidTask);
    PLOGD << "Invalidating task: " << invalidTask << ", Agent: " << agent
          << endl;

    // If the invalidated task was not the last local task of this agent then t_id + 1 exists
    // If the invalid task was not the last task
    if (invalidTask != solution_.getAgentGlobalTasks(agent).back()) {
      int nextTask = solution_.getAgentGlobalTasks(agent, taskPosition + 1);
      // Next task can still be undefined in the case where that task was removed in the previous iteration of this loop
      if (lnsNeighborhood.conflictedTasks.find(nextTask) ==
              lnsNeighborhood.conflictedTasks.end() &&
          nextTask != UNDEFINED) {
        tasksToFix.insert(nextTask);
        PLOGD << "Next task: " << nextTask << endl;
      }
    }

    affectedAgents.insert(agent);

    // Marking past information about this conflicting task
    solution_.taskAgentMap[invalidTask] = UNASSIGNED;
    solution_.agents[agent].clearIntraAgentPrecedenceConstraint(invalidTask);
    // Needs to happen after clearing precedence constraints
    solution_.agents[agent].taskAssignments[taskPosition] = UNDEFINED;

    lnsNeighborhood.conflictedTasksPathSize.insert(make_pair(
        invalidTask, solution_.agents[agent].taskPaths[taskPosition].size()));
    solution_.agents[agent].taskPaths[taskPosition] = Path();
  }

  // Marking past information about conflicting tasks
  for (int affAgent : affectedAgents) {

    // For an affected agent there can be multiple conflicting tasks so need to do it this way
    solution_.agents[affAgent].path = Path();
    solution_.agents[affAgent].taskAssignments.erase(
        std::remove_if(solution_.agents[affAgent].taskAssignments.begin(),
                       solution_.agents[affAgent].taskAssignments.end(),
                       [](int task) { return task == UNDEFINED; }),
        solution_.agents[affAgent].taskAssignments.end());
    solution_.agents[affAgent].taskPaths.erase(
        std::remove_if(solution_.agents[affAgent].taskPaths.begin(),
                       solution_.agents[affAgent].taskPaths.end(),
                       [](const Path& p) { return p.empty(); }),
        solution_.agents[affAgent].taskPaths.end());

    vector<int> taskLocations =
        instance_.getTaskLocations(solution_.getAgentGlobalTasks(affAgent));
    solution_.agents[affAgent].pathPlanner->setGoalLocations(taskLocations);
    solution_.agents[affAgent].pathPlanner->computeHeuristics();
  }

  lnsNeighborhood.patchedTasks = tasksToFix;

  // Find the paths for the tasks whose previous tasks were removed
  for (int task : instance_.getInputPlanningOrder()) {
    if (tasksToFix.count(task) > 0) {

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
      patchAgentTaskPaths(agent, taskPosition);
    }
  }
}

void LNS::computeRegret(bool firstIteration) {
  lnsNeighborhood.regretMaxHeap.clear();
  for (int conflictTask : lnsNeighborhood.conflictedTasks) {
    // If we rejected the last iteration solution, then we are starting again. In the first loop of this iteration we can reuse the computation we did in the first loop of the last iteration. Rest would need to be computed again.
    if (numOfFailures > 0 && firstIteration) {
      // Compute f3 - f2, f4 - f3 etc as needed.
      pairing_heap<Utility, compare<Utility::CompareUtilities>>
          conflictTaskServiceTimes =
              lnsNeighborhood.serviceTimesHeapMap[conflictTask];
      int nextValidUtilityCounter = numOfFailures;
      while (nextValidUtilityCounter > 0) {
        conflictTaskServiceTimes.pop();
        nextValidUtilityCounter--;
      }
      Utility bestUtility = conflictTaskServiceTimes.top();
      conflictTaskServiceTimes.pop();
      Utility nextBestValidUtility = conflictTaskServiceTimes.top();
      Regret regret(conflictTask, bestUtility.agent, bestUtility.taskPosition, bestUtility.pathLength, bestUtility.agentTasksLen, 
                    nextBestValidUtility.value - bestUtility.value);
      lnsNeighborhood.regretMaxHeap.push(regret);
    } else {
      computeRegretForTask(conflictTask, firstIteration);
    }
  }
}

void LNS::computeRegretForTask(int task, bool firstIteration) {
  pairing_heap<Utility, compare<Utility::CompareUtilities>> serviceTimes;

  // The task has to start after the earliest time step but needs to finish before the latest time step. However we cannot give any guarantee on the latest timestep so we only work with the earliest timestep
  int earliestTimestep = 0;
  vector<int> inputPlanningOrder = instance_.getInputPlanningOrder();

  vector<vector<int>> ancestors = instance_.getAncestors();
  set<int> ancestorsOfTask = reachableSet(task, ancestors);
  ancestorsOfTask.erase(task);

  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {

    // Gather the task assignments of this agent with any previous invalidated tasks included as needed relative to the target task
    vector<int> temporaryTaskAssignments = solution_.getAgentGlobalTasks(agent),
                newlyInsertedTasks;
    int taskIndex =
        find(inputPlanningOrder.begin(), inputPlanningOrder.end(), task) -
        inputPlanningOrder.begin();
    for (int i = 0; i < taskIndex; i++) {
      // We only need those tasks who are the actual ancestors of this task
      if (ancestorsOfTask.count(inputPlanningOrder[i]) > 0 &&
          lnsNeighborhood.commitedTasks.count(inputPlanningOrder[i]) > 0 &&
          !lnsNeighborhood.commitedTasks[inputPlanningOrder[i]] &&
          previousSolution_.taskAgentMap[inputPlanningOrder[i]] == agent) {
        int localTaskPositionRelativeToSolution = extractOldLocalTaskIndex(
            inputPlanningOrder[i],
            previousSolution_.agents[agent].taskAssignments);
        temporaryTaskAssignments.insert(temporaryTaskAssignments.begin() +
                                            localTaskPositionRelativeToSolution,
                                        inputPlanningOrder[i]);
        newlyInsertedTasks.push_back(inputPlanningOrder[i]);
      }
    }

    // Gather the paths of the tasks that need to be done before this task
    vector<Path> temporaryTaskPaths;
    temporaryTaskPaths.resize(instance_.getTasksNum());
    // Gather the corresponding task paths and stitch them together
    for (int i = 0; i < (int)temporaryTaskAssignments.size(); i++) {
      int temporaryTask = temporaryTaskAssignments[i];
      if (lnsNeighborhood.commitedTasks.count(temporaryTask) > 0 &&
          !lnsNeighborhood.commitedTasks[temporaryTask]) {
        int temporaryTaskAgent = previousSolution_.taskAgentMap[temporaryTask];
        assert(temporaryTaskAgent != UNASSIGNED);
        int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
            temporaryTaskAgent, temporaryTask);
        Path temporaryTaskPath = previousSolution_.agents[temporaryTaskAgent]
                                     .taskPaths[temporaryTaskLocalIndex];
        // Fix the begin time such that this task starts when the previous task ends
        if (i != 0) {
          temporaryTaskPath.beginTime =
              temporaryTaskPaths[temporaryTaskAssignments[i - 1]].endTime();
        }
        temporaryTaskPaths[temporaryTask] = temporaryTaskPath;
      } else {
        // In this case the previous task was recently inserted because it was in the conflict set and not committed to. Further the next task was patched in the prepare iteration function
        // This condition depends on the fact that the previous if condition would have triggered in earlier iteration of this loop
        if (i > 0 &&
            find(newlyInsertedTasks.begin(), newlyInsertedTasks.end(),
                 temporaryTaskAssignments[i - 1]) != newlyInsertedTasks.end() &&
            lnsNeighborhood.patchedTasks.count(temporaryTask) > 0) {
          int temporaryTaskAgent =
              previousSolution_.taskAgentMap[temporaryTask];
          assert(temporaryTaskAgent != UNASSIGNED);
          int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
              temporaryTaskAgent, temporaryTask);
          Path temporaryTaskPath = previousSolution_.agents[temporaryTaskAgent]
                                       .taskPaths[temporaryTaskLocalIndex];
          temporaryTaskPath.beginTime =
              temporaryTaskPaths[temporaryTaskAssignments[i - 1]].endTime();
          temporaryTaskPaths[temporaryTask] = temporaryTaskPath;
        } else {
          int temporaryTaskAgent = solution_.taskAgentMap[temporaryTask];
          assert(temporaryTaskAgent != UNASSIGNED);
          int temporaryTaskLocalIndex =
              solution_.getLocalTaskIndex(temporaryTaskAgent, temporaryTask);
          Path temporaryTaskPath = solution_.agents[temporaryTaskAgent]
                                       .taskPaths[temporaryTaskLocalIndex];
          // Fix the begin time such that this task starts when the previous task ends
          if (i != 0) {
            temporaryTaskPath.beginTime =
                temporaryTaskPaths[temporaryTaskAssignments[i - 1]].endTime();
          }
          temporaryTaskPaths[temporaryTask] = temporaryTaskPath;
        }
      }
    }

    // Gather the paths of the tasks that need to be done before this task but have been assigned to some other agents
    // This is for agents apart from "agent" i.e the loop variable
    set<int> agentsNewlyAddedTasksBelongTo;
    for (int i = 0; i < taskIndex; i++) {
      int ancestorTask = inputPlanningOrder[i];
      // Only consider those tasks that are the actual ancestors of this task
      if (ancestorsOfTask.count(ancestorTask) > 0 &&
          temporaryTaskPaths[ancestorTask].empty()) {
        if (lnsNeighborhood.commitedTasks.count(ancestorTask) > 0 &&
            !lnsNeighborhood.commitedTasks[ancestorTask]) {
          int temporaryTaskAgent = previousSolution_.taskAgentMap[ancestorTask];
          assert(temporaryTaskAgent != UNASSIGNED);
          int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
              temporaryTaskAgent, ancestorTask);
          Path temporaryTaskPath = previousSolution_.agents[temporaryTaskAgent]
                                       .taskPaths[temporaryTaskLocalIndex];
          // TODO: Should we need to fix the begin times of these paths?
          temporaryTaskPaths[ancestorTask] = temporaryTaskPath;
          agentsNewlyAddedTasksBelongTo.insert(temporaryTaskAgent);
        }
        // In this case some ancestor was inserted that was not there before in the previous if condition of this loop. Then we need to use the previous solution's path for the next task in some agent's queue
        else {
          int temporaryTaskAgent = solution_.taskAgentMap[ancestorTask];
          assert(temporaryTaskAgent != UNASSIGNED);
          if (agentsNewlyAddedTasksBelongTo.count(temporaryTaskAgent) > 0 &&
              lnsNeighborhood.patchedTasks.count(ancestorTask) > 0) {
            int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
                temporaryTaskAgent, ancestorTask);
            Path temporaryTaskPath =
                previousSolution_.agents[temporaryTaskAgent]
                    .taskPaths[temporaryTaskLocalIndex];
            // TODO: Should we need to fix the begin times of these paths?
            temporaryTaskPaths[ancestorTask] = temporaryTaskPath;
          } else {
            int temporaryTaskLocalIndex =
                solution_.getLocalTaskIndex(temporaryTaskAgent, ancestorTask);
            Path temporaryTaskPath = solution_.agents[temporaryTaskAgent]
                                         .taskPaths[temporaryTaskLocalIndex];
            // TODO: Should we need to fix the begin times of these paths?
            temporaryTaskPaths[ancestorTask] = temporaryTaskPath;
          }
        }
      }
    }

    vector<vector<int>> ancestors = instance_.getAncestors();
    set<int> setOfTasksToComplete = reachableSet(task, ancestors);
    setOfTasksToComplete.erase(task);

    for (int ancestor : setOfTasksToComplete) {
      assert(!temporaryTaskPaths[ancestor].empty());
      earliestTimestep =
          max(earliestTimestep, temporaryTaskPaths[ancestor].endTime());
    }

    // Append the agent's internal precedence constraints to the input precedence constraints
    vector<pair<int, int>> precedenceConstraints =
        instance_.getInputPrecedenceConstraints();
    for (int i = 0; i < (int)temporaryTaskAssignments.size() - 1; i++) {
      precedenceConstraints.emplace_back(temporaryTaskAssignments[i],
                                         temporaryTaskAssignments[i + 1]);
    }

    TaskRegretPacket regretPacket = {task, agent, -1, earliestTimestep};
    computeRegretForTaskWithAgent(regretPacket, &temporaryTaskAssignments,
                                  &temporaryTaskPaths, &precedenceConstraints,
                                  &serviceTimes);
  }

  // Only need to keep track of the service times when none of the conflict tasks were committed. Next iteration onwards we would need to recompute them again
  if (firstIteration) {
    lnsNeighborhood.serviceTimesHeapMap.insert(make_pair(task, serviceTimes));
  }

  // TODO: Add checks to ensure that there are atleast 2 entries in the heap!
  Utility bestUtility = serviceTimes.top();
  serviceTimes.pop();
  Utility secondBestUtility = serviceTimes.top();
  Regret regret(task, bestUtility.agent, bestUtility.taskPosition, bestUtility.pathLength, bestUtility.agentTasksLen,
                secondBestUtility.value - bestUtility.value);
  lnsNeighborhood.regretMaxHeap.push(regret);
}

void LNS::computeRegretForTaskWithAgent(
    TaskRegretPacket regretPacket, vector<int>* taskAssignments,
    vector<Path>* taskPaths, vector<pair<int, int>>* precedenceConstraints,
    pairing_heap<Utility, compare<Utility::CompareUtilities>>* serviceTimes) {

  // Compute the first position along the agent's task assignments where we can insert this task
  int firstValidPosition = 0;
  for (int j = (int)taskAssignments->size() - 1; j >= 0; j--) {
    int taskID = (*taskAssignments)[j];
    if ((*taskPaths)[taskID].endTime() <= regretPacket.earliestTimestep) {
      firstValidPosition = j + 1;
      break;
    }
  }

  assert(firstValidPosition >= 0);

  for (int j = firstValidPosition; j <= (int)taskAssignments->size(); j++) {

    regretPacket.taskPosition = j;
    // Create a copy of the task assignments, paths and corresponding precedence constraints so that they dont get modified
    vector<Path> temporaryTaskPaths = *taskPaths;
    vector<int> temporaryTaskAssignments = *taskAssignments;
    vector<pair<int, int>> temporaryPrecedenceConstraints =
        *precedenceConstraints;
    Utility utility =
        insertTask(regretPacket, &temporaryTaskPaths, &temporaryTaskAssignments,
                   &temporaryPrecedenceConstraints);
    serviceTimes->push(utility);
  }
}

// Need the task paths, assignments and precedence constraints as pointers so that we can reuse this code when commiting as we can make in-place changes to these data-structures
Utility LNS::insertTask(TaskRegretPacket regretPacket, vector<Path>* taskPaths,
                        vector<int>* taskAssignments,
                        vector<pair<int, int>>* precedenceConstraints) {

  double pathSizeChange = 0;
  int startTime = 0, previousTask = -1, nextTask = -1;

  // The task paths are all the task paths when we dont commit but if we commit they will be agent specific task paths
  vector<Path>& taskPathsRef = *taskPaths;
  vector<int>& taskAssignmentsRef = *taskAssignments;
  vector<pair<int, int>>& precedenceConstraintsRef = *precedenceConstraints;

  int agentTasksSize = (int)taskAssignmentsRef.size();

  // In this case we are inserting a task not at the last position
  if (regretPacket.taskPosition < agentTasksSize) {

    nextTask = taskAssignmentsRef[regretPacket.taskPosition];
    taskAssignmentsRef.insert(
        taskAssignmentsRef.begin() + regretPacket.taskPosition,
        regretPacket.task);

    // Invalidate the path of the next task
    // Compute the path size of the next task before you remove it!
    taskPathsRef[regretPacket.task] = Path();
    pathSizeChange = (double)taskPathsRef[nextTask].size();
    taskPathsRef[nextTask] = Path();

    precedenceConstraintsRef.emplace_back(regretPacket.task, nextTask);

    // If we are NOT inserting at the start position then we need to take care of the previous task as well
    if (regretPacket.taskPosition != 0) {
      previousTask = taskAssignmentsRef[regretPacket.taskPosition - 1];
      // TODO: Technically the task can start being processed before the previous task ends. This is more conservative but need to check if there are better ways to tackle this.
      startTime = taskPathsRef[previousTask].endTime();
      precedenceConstraintsRef.erase(
          std::remove_if(
              precedenceConstraintsRef.begin(), precedenceConstraintsRef.end(),
              [previousTask, nextTask](pair<int, int> x) {
                return x.first == previousTask && x.second == nextTask;
              }),
          precedenceConstraintsRef.end());
      precedenceConstraintsRef.emplace_back(previousTask, regretPacket.task);
    }
  }
  // In this case we are inserting at the very end
  else if (regretPacket.taskPosition == agentTasksSize && agentTasksSize != 0) {

    previousTask = taskAssignmentsRef[regretPacket.taskPosition - 1];
    startTime = taskPathsRef[previousTask].endTime();

    taskAssignmentsRef.push_back(regretPacket.task);
    precedenceConstraintsRef.emplace_back(previousTask, regretPacket.task);
  } else if (agentTasksSize == 0) {
    // This is the rare-case when the agent has no tasks assigned to it.
    assert(regretPacket.taskPosition == 0);

    startTime = 0;
    taskAssignmentsRef.push_back(regretPacket.task);
  }

  if (nextTask != -1) {
    // The task paths reference does not have ancestor information about next task, so we need to add those in
    set<int> agentsNewlyAddedTasksBelongTo;
    vector<int> planningOrder = instance_.getInputPlanningOrder();

    vector<vector<int>> ancestors = instance_.getAncestors();
    set<int> ancestorsOfNextTask = reachableSet(nextTask, ancestors);
    ancestorsOfNextTask.erase(nextTask);

    int nextTaskIndex =
        find(planningOrder.begin(), planningOrder.end(), nextTask) -
        planningOrder.begin();
    for (int i = 0; i < nextTaskIndex; i++) {
      int nextTaskAncestor = planningOrder[i];
      // The ancestor of the next task is not assigned. It can either be in the previous solution if and only if the ancestor was in the conflict set but not committed it. Otherwise it must be in the solution.
      // We only care about the ancestor that is part of the actual ancestor of this next task
      if (ancestorsOfNextTask.count(nextTaskAncestor) > 0 &&
          taskPathsRef[nextTaskAncestor].empty()) {
        if (lnsNeighborhood.commitedTasks.count(nextTaskAncestor) > 0 &&
            !lnsNeighborhood.commitedTasks[nextTaskAncestor]) {
          int nextTaskAncestorAgent =
              previousSolution_.taskAgentMap[nextTaskAncestor];
          assert(nextTaskAncestorAgent != UNASSIGNED);
          int nextTaskAncestorLocalIndex = previousSolution_.getLocalTaskIndex(
              nextTaskAncestorAgent, nextTaskAncestor);
          // TODO: Should we change the begin time of this task?
          taskPathsRef[nextTaskAncestor] =
              previousSolution_.agents[nextTaskAncestorAgent]
                  .taskPaths[nextTaskAncestorLocalIndex];
          // If we are updating the regret packet's agent task queue then we need to update the precedence constraints as well
          if (nextTaskAncestorAgent == regretPacket.agent) {
            taskAssignmentsRef.insert(
                taskAssignmentsRef.begin() + nextTaskAncestorLocalIndex,
                nextTaskAncestor);
            int localPreviousTask = -1, localNextTask = -1;
            if (nextTaskAncestorLocalIndex != 0) {
              previousTask = taskAssignmentsRef[nextTaskAncestorLocalIndex - 1];
            }
            if (nextTaskAncestorLocalIndex !=
                (int)taskAssignmentsRef.size() - 1) {
              nextTask = taskAssignmentsRef[nextTaskAncestorLocalIndex + 1];
            }
            precedenceConstraintsRef.erase(
                std::remove_if(
                    precedenceConstraintsRef.begin(),
                    precedenceConstraintsRef.end(),
                    [localPreviousTask, localNextTask](pair<int, int> x) {
                      return ((x.first == localPreviousTask &&
                               x.second == localNextTask));
                    }),
                precedenceConstraintsRef.end());
            if (localPreviousTask != -1) {
              precedenceConstraintsRef.insert(
                  precedenceConstraintsRef.begin() +
                      nextTaskAncestorLocalIndex - 1,
                  make_pair(localPreviousTask, nextTaskAncestor));
            }
            if (localNextTask != -1) {
              precedenceConstraintsRef.insert(
                  precedenceConstraintsRef.begin() + nextTaskAncestorLocalIndex,
                  make_pair(nextTaskAncestor, localNextTask));
            }
          }
          agentsNewlyAddedTasksBelongTo.insert(nextTaskAncestorAgent);
        } else {
          int nextTaskAncestorAgent = solution_.taskAgentMap[nextTaskAncestor];
          // This agent would have to be different than the regret packet's agent. We dont need to update task assignments or precedence constraints in this case as the regret packet's agent's queue should not be affected here as we are using the current solution anyways
          assert(nextTaskAncestorAgent != UNASSIGNED);
          if (agentsNewlyAddedTasksBelongTo.count(nextTaskAncestorAgent) > 0 &&
              lnsNeighborhood.patchedTasks.count(nextTaskAncestor) > 0) {
            int nextTaskAncestorLocalIndex =
                previousSolution_.getLocalTaskIndex(nextTaskAncestorAgent,
                                                    nextTaskAncestor);
            // TODO: Should we change the begin time of this task?
            taskPathsRef[nextTaskAncestor] =
                previousSolution_.agents[nextTaskAncestorAgent]
                    .taskPaths[nextTaskAncestorLocalIndex];
          } else {
            int nextTaskAncestorLocalIndex = solution_.getLocalTaskIndex(
                nextTaskAncestorAgent, nextTaskAncestor);
            // TODO: Should we change the begin time of this task?
            taskPathsRef[nextTaskAncestor] =
                solution_.agents[nextTaskAncestorAgent]
                    .taskPaths[nextTaskAncestorLocalIndex];
          }
        }
      }
    }

    int taskPosition = find(taskAssignmentsRef.begin(),
                            taskAssignmentsRef.end(), regretPacket.task) -
                       taskAssignmentsRef.begin();
    vector<int> goalLocations = instance_.getTaskLocations(taskAssignmentsRef);
    ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);

    MultiLabelSpaceTimeAStar localPlanner =
        MultiLabelSpaceTimeAStar(instance_, regretPacket.agent);
    localPlanner.setGoalLocations(goalLocations);
    localPlanner.computeHeuristics();

    buildConstraintTable(constraintTable, regretPacket.task,
                         goalLocations[regretPacket.taskPosition],
                         &taskPathsRef, &precedenceConstraintsRef);
    taskPathsRef[regretPacket.task] = localPlanner.findPathSegment(
        constraintTable, startTime, taskPosition, 0);
    startTime = taskPathsRef[regretPacket.task].endTime();

    // Need to recompute the positions as we might add paths for parent tasks before reaching here!
    int nextTaskPosition =
        find(taskAssignmentsRef.begin(), taskAssignmentsRef.end(), nextTask) -
        taskAssignmentsRef.begin();
    buildConstraintTable(constraintTable, nextTask,
                         goalLocations[nextTaskPosition], &taskPathsRef,
                         &precedenceConstraintsRef);
    taskPathsRef[nextTask] = localPlanner.findPathSegment(
        constraintTable, startTime, nextTaskPosition, 0);
  } else {

    vector<int> goalLocations = instance_.getTaskLocations(taskAssignmentsRef);
    ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);

    MultiLabelSpaceTimeAStar localPlanner =
        MultiLabelSpaceTimeAStar(instance_, regretPacket.agent);
    localPlanner.setGoalLocations(goalLocations);
    localPlanner.computeHeuristics();

    buildConstraintTable(constraintTable, regretPacket.task,
                         goalLocations[regretPacket.taskPosition],
                         &taskPathsRef, &precedenceConstraintsRef);
    taskPathsRef[regretPacket.task] = localPlanner.findPathSegment(
        constraintTable, startTime, regretPacket.taskPosition, 0);
  }

  auto value = (double)taskPathsRef[regretPacket.task].size();
  if (nextTask != -1) {
    value += (double)taskPathsRef[nextTask].size();
  }
  auto pathLength = value;
  value -= (lnsNeighborhood.conflictedTasksPathSize.at(regretPacket.task) +
            pathSizeChange);

  Utility utility(regretPacket.agent, regretPacket.taskPosition, pathLength, (int)taskAssignmentsRef.size(), value);
  return utility;
}

void LNS::commitAncestorTaskOf(
    int globalTask, std::optional<pair<bool, int>> committingNextTask) {
  // We are going to commit some ancestor of this global task. We need to ensure that the paths of all the required ancestors of this task are in order before we can commit the global task and any next task that may exist
  // If the boolean flag commitingNextTask is set then it means that the global task was the next task of some other task and we need to ensure that the ancestors of this next task are in order. This additional check is required as the first if condition changes depending on it.
  // The corresponding integer entry would be the global task id of the main task that we wanted to commit.

  vector<int> inputPlanningOrder = instance_.getInputPlanningOrder();

  vector<vector<int>> ancestors = instance_.getAncestors();
  set<int> ancestorsOfTask = reachableSet(globalTask, ancestors);
  ancestorsOfTask.erase(globalTask);

  int taskIndex =
      find(inputPlanningOrder.begin(), inputPlanningOrder.end(), globalTask) -
      inputPlanningOrder.begin();

  for (int i = 0; i < taskIndex; i++) {

    int ancestorTask = inputPlanningOrder[i];

    // We only care about the tasks that are the actual ancestors of this task!
    if (ancestorsOfTask.count(ancestorTask) == 0) {
      continue;
    }

    if (ancestorTask == globalTask) {
      break;
    }

    // This case relates to the tasks in the conflict set that need to be addressed before the task can be committed
    if (lnsNeighborhood.commitedTasks.count(ancestorTask) > 0 &&
        !lnsNeighborhood.commitedTasks[ancestorTask]) {

      if (committingNextTask.has_value() &&
          committingNextTask.value().second == ancestorTask) {
        // If we are committing next task of some actual other task then that actual task would not be considered committed unless this next task is committed as well. However that actual task will come before this next task in the planning order so we need to skip that one!
        continue;
      }

      int ancestorTaskAgent = previousSolution_.taskAgentMap[ancestorTask];
      assert(ancestorTaskAgent != UNASSIGNED);

      PLOGD << "Commiting ancestor task " << ancestorTask << " to agent "
            << ancestorTaskAgent << " using previous solution" << endl;

      int ancestorTaskPositionRelativeToSolution = extractOldLocalTaskIndex(
          ancestorTask,
          previousSolution_.agents[ancestorTaskAgent].taskAssignments);
      int ancestorTaskPosition =
          previousSolution_.getLocalTaskIndex(ancestorTaskAgent, ancestorTask);

      Path ancestorPath = previousSolution_.agents[ancestorTaskAgent]
                              .taskPaths[ancestorTaskPosition];

      solution_.agents[ancestorTaskAgent].pathPlanner->goalLocations.insert(
          solution_.agents[ancestorTaskAgent]
                  .pathPlanner->goalLocations.begin() +
              ancestorTaskPositionRelativeToSolution,
          instance_.getTaskLocations(ancestorTask));
      solution_.agents[ancestorTaskAgent].pathPlanner->computeHeuristics();

      solution_.agents[ancestorTaskAgent].taskAssignments.insert(
          solution_.agents[ancestorTaskAgent].taskAssignments.begin() +
              ancestorTaskPositionRelativeToSolution,
          ancestorTask);

      solution_.agents[ancestorTaskAgent].insertIntraAgentPrecedenceConstraint(
          ancestorTask, ancestorTaskPositionRelativeToSolution);

      solution_.agents[ancestorTaskAgent].taskPaths.insert(
          solution_.agents[ancestorTaskAgent].taskPaths.begin() +
              ancestorTaskPositionRelativeToSolution,
          ancestorPath);

      solution_.taskAgentMap[ancestorTask] = ancestorTaskAgent;

      // The ancestor of the task that was in the conflict set has now been committed using its old path, hence we need to mark it as resolved now
      markResolved(ancestorTask);

      // Now if the next task on this agent's task queue was patched during the prepare iteration function phase then we need to remove it from that object and reuse its old path!
      if (ancestorTask !=
              (int)solution_.agents[ancestorTaskAgent].taskAssignments.back() &&
          lnsNeighborhood.patchedTasks.count(
              solution_.agents[ancestorTaskAgent]
                  .taskAssignments[ancestorTaskPositionRelativeToSolution +
                                   1]) > 0) {

        int localNextTask =
            solution_.agents[ancestorTaskAgent]
                .taskAssignments[ancestorTaskPositionRelativeToSolution + 1];
        int localNextTaskOldPosition = previousSolution_.getLocalTaskIndex(
            ancestorTaskAgent, localNextTask);
        // Dont want to mess with the begin time of this task because if some other ancestor task would get in between these two then the begin time would be wrong if edited here
        solution_.agents[ancestorTaskAgent]
            .taskPaths[ancestorTaskPositionRelativeToSolution + 1] =
            previousSolution_.agents[ancestorTaskAgent]
                .taskPaths[localNextTaskOldPosition];
        lnsNeighborhood.patchedTasks.erase(localNextTask);
      }
      // Fix the begin times based on whatever new paths were added
      patchAgentTaskPaths(ancestorTaskAgent, 0);
    }
  }
}

void LNS::commitBestRegretTask(Regret bestRegret) {

  PLOGD << "Commiting for task " << bestRegret.task << " to agent "
        << bestRegret.agent << " with regret = " << bestRegret.value << endl;

  TaskRegretPacket bestRegretPacket = {
      bestRegret.task, bestRegret.agent, bestRegret.taskPosition, {}};

  commitAncestorTaskOf(bestRegret.task, std::nullopt);

  // At this point any previously empty paths must be resolved and we can use the insert task function to commit to the actual best regret task
  insertBestRegretTask(bestRegretPacket);
  markResolved(bestRegret.task);
}

void LNS::insertBestRegretTask(TaskRegretPacket bestRegretPacket) {

  int startTime = 0, previousTask = -1, nextTask = -1;

  vector<pair<int, int>> precedenceConstraints =
      instance_.getInputPrecedenceConstraints();
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    precedenceConstraints.insert(
        precedenceConstraints.end(),
        solution_.agents[agent].intraPrecedenceConstraints.begin(),
        solution_.agents[agent].intraPrecedenceConstraints.end());
  }

  int agentTasksSize =
      (int)solution_.agents[bestRegretPacket.agent].taskAssignments.size();

  // In this case we are inserting a task not at the last position
  if (bestRegretPacket.taskPosition < agentTasksSize) {

    nextTask = solution_.agents[bestRegretPacket.agent]
                   .taskAssignments[bestRegretPacket.taskPosition];
    solution_.agents[bestRegretPacket.agent].taskAssignments.insert(
        solution_.agents[bestRegretPacket.agent].taskAssignments.begin() +
            bestRegretPacket.taskPosition,
        bestRegretPacket.task);

    solution_.agents[bestRegretPacket.agent]
        .taskPaths[bestRegretPacket.taskPosition] = Path();

    precedenceConstraints.emplace_back(bestRegretPacket.task, nextTask);

    // If we are NOT inserting at the start position then we need to take care of the previous task as well
    if (bestRegretPacket.taskPosition != 0) {
      previousTask = solution_.agents[bestRegretPacket.agent]
                         .taskAssignments[bestRegretPacket.taskPosition - 1];
      // TODO: Technically the task can start being processed before the previous task ends. This is more conservative but need to check if there are better ways to tackle this.
      startTime = solution_.agents[bestRegretPacket.agent]
                      .taskPaths[bestRegretPacket.taskPosition - 1]
                      .endTime();
      precedenceConstraints.erase(
          std::remove_if(
              precedenceConstraints.begin(), precedenceConstraints.end(),
              [previousTask, nextTask](pair<int, int> x) {
                return x.first == previousTask && x.second == nextTask;
              }),
          precedenceConstraints.end());
      precedenceConstraints.emplace_back(previousTask, bestRegretPacket.task);
    }
  }
  // In this case we are inserting at the very end
  else if (bestRegretPacket.taskPosition == agentTasksSize &&
           agentTasksSize != 0) {

    previousTask = solution_.agents[bestRegretPacket.agent]
                       .taskAssignments[bestRegretPacket.taskPosition - 1];
    startTime = solution_.agents[bestRegretPacket.agent]
                    .taskPaths[bestRegretPacket.taskPosition - 1]
                    .endTime();

    solution_.agents[bestRegretPacket.agent].taskAssignments.push_back(
        bestRegretPacket.task);
    precedenceConstraints.emplace_back(previousTask, bestRegretPacket.task);

    solution_.agents[bestRegretPacket.agent].taskPaths.emplace_back();
  } else if (agentTasksSize == 0) {
    // The rare-case when the agent has no tasks assigned to them
    assert(bestRegretPacket.taskPosition == 0);

    startTime = 0;
    solution_.agents[bestRegretPacket.agent].taskAssignments.push_back(
        bestRegretPacket.task);
    solution_.agents[bestRegretPacket.agent].taskPaths.emplace_back();
  }

  if (nextTask != -1) {

    commitAncestorTaskOf(
        nextTask, std::make_optional(make_pair(true, bestRegretPacket.task)));

    vector<int> goalLocations = instance_.getTaskLocations(
        solution_.agents[bestRegretPacket.agent].taskAssignments);
    ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);

    solution_.agents[bestRegretPacket.agent].pathPlanner->setGoalLocations(
        goalLocations);
    solution_.agents[bestRegretPacket.agent].pathPlanner->computeHeuristics();

    // Need to recompute this task position as we may have added tasks in the agent's task queue when trying to account for the next task parents
    int taskPosition =
        find(solution_.agents[bestRegretPacket.agent].taskAssignments.begin(),
             solution_.agents[bestRegretPacket.agent].taskAssignments.end(),
             bestRegretPacket.task) -
        solution_.agents[bestRegretPacket.agent].taskAssignments.begin();

    // Insert an empty path at that task position
    solution_.agents[bestRegretPacket.agent].taskPaths.insert(
        solution_.agents[bestRegretPacket.agent].taskPaths.begin() +
            taskPosition,
        Path());

    buildConstraintTable(constraintTable, bestRegretPacket.task);
    solution_.agents[bestRegretPacket.agent].taskPaths[taskPosition] =
        solution_.agents[bestRegretPacket.agent].pathPlanner->findPathSegment(
            constraintTable, startTime, taskPosition, 0);
    solution_.agents[bestRegretPacket.agent]
        .insertIntraAgentPrecedenceConstraint(bestRegretPacket.task,
                                              bestRegretPacket.taskPosition);
    solution_.taskAgentMap[bestRegretPacket.task] = bestRegretPacket.agent;

    buildConstraintTable(constraintTable, nextTask);
    int nextTaskPosition =
        find(solution_.agents[bestRegretPacket.agent].taskAssignments.begin(),
             solution_.agents[bestRegretPacket.agent].taskAssignments.end(),
             nextTask) -
        solution_.agents[bestRegretPacket.agent].taskAssignments.begin();
    assert(nextTaskPosition - 1 >= 0);
    startTime = solution_.agents[bestRegretPacket.agent]
                    .taskPaths[nextTaskPosition - 1]
                    .endTime();
    solution_.agents[bestRegretPacket.agent].taskPaths[nextTaskPosition] =
        solution_.agents[bestRegretPacket.agent].pathPlanner->findPathSegment(
            constraintTable, startTime, nextTaskPosition, 0);
  } else {

    vector<int> goalLocations = instance_.getTaskLocations(
        solution_.agents[bestRegretPacket.agent].taskAssignments);
    ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);

    solution_.agents[bestRegretPacket.agent].pathPlanner->setGoalLocations(
        goalLocations);
    solution_.agents[bestRegretPacket.agent].pathPlanner->computeHeuristics();

    buildConstraintTable(constraintTable, bestRegretPacket.task);
    solution_.agents[bestRegretPacket.agent]
        .taskPaths[bestRegretPacket.taskPosition] =
        solution_.agents[bestRegretPacket.agent].pathPlanner->findPathSegment(
            constraintTable, startTime, bestRegretPacket.taskPosition, 0);
    solution_.agents[bestRegretPacket.agent]
        .insertIntraAgentPrecedenceConstraint(bestRegretPacket.task,
                                              bestRegretPacket.taskPosition);
    solution_.taskAgentMap[bestRegretPacket.task] = bestRegretPacket.agent;
  }

  patchAgentTaskPaths(bestRegretPacket.agent, 0);
}

void LNS::buildConstraintTable(ConstraintTable& constraintTable, int task,
                               int taskLocation, vector<Path>* taskPaths,
                               vector<pair<int, int>>* precedenceConstraints) {

  constraintTable.goalLocation = taskLocation;

  vector<vector<int>> ancestors = instance_.getAncestors();
  // TODO: We used input precedence constraints here but to me it seems like the input precedence constraints should be augmented by the precedence constraints of the agent we are considering here as well!
  // for (pair<int, int> precedenceConstraint : *precedenceConstraints) {
  //   ancestors[precedenceConstraint.second].push_back(
  //       precedenceConstraint.first);
  // }

  set<int> setOfTasksToComplete = reachableSet(task, ancestors);
  setOfTasksToComplete.erase(task);

  // Identify which of the tasks that need to be done before the target task was/is the final task of its assigned agent
  map<int, int> lastTasks;
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    // Loop through these tasks to ensure that once it finishes the last tasks map is fully updated
    for (int ancestorTask : setOfTasksToComplete) {
      // If the task was not in commited tasks set then it must be in the conflict set and its path would only be available in the previous solution
      if (lnsNeighborhood.commitedTasks.count(ancestorTask) > 0 &&
          !lnsNeighborhood.commitedTasks[ancestorTask]) {
        if (previousSolution_.taskAgentMap[ancestorTask] == agent &&
            previousSolution_.agents[agent].taskAssignments.back() ==
                ancestorTask) {
          lastTasks.insert(make_pair(agent, ancestorTask));
        }
      }
      // Otherwise it must be in the solution.
      else {
        if (solution_.taskAgentMap[ancestorTask] == agent &&
            solution_.agents[agent].taskAssignments.back() == ancestorTask) {
          lastTasks.insert(make_pair(agent, ancestorTask));
        }
      }
    }
  }

  // Loop through the last task map to gather the actual final tasks of the agents
  vector<bool> finalTasks(instance_.getTasksNum(), false);
  for (pair<int, int> lastTask : lastTasks) {
    finalTasks[lastTask.second] = true;
  }

  // Add the paths of the prior tasks to the constraint table with information about whether they were their agent's final tasks or not
  for (int ancestorTask : setOfTasksToComplete) {
    assert(!(*taskPaths)[ancestorTask].empty());
    bool waitAtGoal = finalTasks[ancestorTask];
    constraintTable.addPath((*taskPaths)[ancestorTask], waitAtGoal);
  }

  for (int ancestorTask : ancestors[task]) {
    assert(!(*taskPaths)[ancestorTask].empty());
    constraintTable.lengthMin = max(constraintTable.lengthMin,
                                    (*taskPaths)[ancestorTask].endTime() + 1);
  }
  constraintTable.latestTimestep =
      max(constraintTable.latestTimestep, constraintTable.lengthMin);
}

void LNS::buildConstraintTable(ConstraintTable& constraintTable, int task) {

  constraintTable.goalLocation = instance_.getTaskLocations(task);

  vector<pair<int, int>> precedenceConstraints =
      instance_.getInputPrecedenceConstraints();

  vector<vector<int>> ancestors = instance_.getAncestors();
  set<int> setOfTasksToComplete = reachableSet(task, ancestors);
  setOfTasksToComplete.erase(task);

  for (int ancestorTask : setOfTasksToComplete) {
    int ancestorTaskAgent = solution_.taskAgentMap[ancestorTask];
    assert(ancestorTaskAgent != UNASSIGNED);
    int ancestorTaskPosition =
        solution_.getLocalTaskIndex(ancestorTaskAgent, ancestorTask);
    bool waitAtGoal =
        ancestorTaskPosition ==
        (int)solution_.getAgentGlobalTasks(ancestorTaskAgent).size() - 1;
    constraintTable.addPath(
        solution_.agents[ancestorTaskAgent].taskPaths[ancestorTaskPosition],
        waitAtGoal);
  }

  for (int ancestorTask : ancestors[task]) {
    int ancestorTaskAgent = solution_.getAgentWithTask(ancestorTask);
    int ancestorTaskPosition =
        solution_.getLocalTaskIndex(ancestorTaskAgent, ancestorTask);
    assert(!solution_.agents[ancestorTaskAgent]
                .taskPaths[ancestorTaskPosition]
                .empty());
    constraintTable.lengthMin =
        max(constraintTable.lengthMin, solution_.agents[ancestorTaskAgent]
                                               .taskPaths[ancestorTaskPosition]
                                               .endTime() +
                                           1);
  }

  constraintTable.latestTimestep =
      max(constraintTable.latestTimestep, constraintTable.lengthMin);
}

int LNS::extractOldLocalTaskIndex(int task, vector<int> taskQueue) {
  int localTaskPositionOffset = 0;
  // We need to compute the offset as we can invalidate multiple tasks associated with an agent. This means that simply querying the previous solution agent's task index is not enough as the it would be more than the actual task position value for the current solution
  for (int localTask : taskQueue) {
    // We dont need to bother for the tasks that come after the current one since we are considering them in planning order
    if (localTask == task) {
      break;
    }
    if (lnsNeighborhood.immutableConflictedTasks.count(localTask) > 0) {
      localTaskPositionOffset++;
    }
  }
  int index = 0;
  for (; index < (int)taskQueue.size(); index++) {
    if (taskQueue[index] == task) {
      break;
    }
  }
  assert(index < (int)taskQueue.size());
  return index - localTaskPositionOffset;
}

set<int> LNS::reachableSet(int source, vector<vector<int>> edgeList) {
  set<int> result;
  stack<int> q({source});
  while (!q.empty()) {
    int current = q.top();
    q.pop();
    if (result.count(current) > 0) {
      continue;
    }
    result.insert(current);
    for (int sink : edgeList[current]) {
      if (result.count(sink) == 0) {
        q.push(sink);
      }
    }
  }
  return result;
}

void LNS::markResolved(int globalTask) {
  assert(lnsNeighborhood.conflictedTasks.count(globalTask) > 0);
  lnsNeighborhood.conflictedTasks.erase(globalTask);
  lnsNeighborhood.conflictedTasksPathSize.erase(globalTask);
  lnsNeighborhood.commitedTasks[globalTask] = true;
}

void LNS::patchAgentTaskPaths(int agent, int taskPosition) {
  for (int k = taskPosition + 1;
       k < (int)solution_.agents[agent].taskAssignments.size(); k++) {
    solution_.agents[agent].taskPaths[k].beginTime =
        solution_.agents[agent].taskPaths[k - 1].endTime();
  }
}

bool LNS::validateSolution(set<int>* conflictedTasks) {

  bool result = true;

  vector<pair<int, int>> precedenceConstraints =
      instance_.getInputPrecedenceConstraints();
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    precedenceConstraints.insert(
        precedenceConstraints.end(),
        solution_.agents[agent].intraPrecedenceConstraints.begin(),
        solution_.agents[agent].intraPrecedenceConstraints.end());
  }

  // Check that the precedence constraints are not violated
  for (pair<int, int> precedenceConstraint : precedenceConstraints) {

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
      if (solution_.agents[agentI].taskAssignments.empty() ||
          solution_.agents[agentJ].taskAssignments.empty()) {
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
        // Check that any two agents are not following the same edge in the opposite direction at the same timestep
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

      // Check that any two agents are not at the same location at the same timestep where one agent might be waiting already
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
    int agentPathSize = 0;
    if (!solution_.agents[i].taskAssignments.empty()) {
      agentPathSize = solution_.agents[i].path.endTime();
    }
    cout << "Agent " << i << " (cost = " << agentPathSize << "): ";
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

Solution& Solution::operator=(const Solution& other) {
  if (this == &other) {
    return *this;
  }
  this->numOfTasks = other.numOfTasks;
  this->numOfAgents = other.numOfAgents;
  this->sumOfCosts = other.sumOfCosts;

  this->agents = other.agents;
  this->taskAgentMap = other.taskAgentMap;

  return *this;
}
