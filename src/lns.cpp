#include "lns.hpp"
#include <stdio.h>
#include <boost/process.hpp>
#include <boost/process/pipe.hpp>
#include <numeric>
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

bool LNS::buildGreedySolutionWithCBSPC() {

  initialPaths.resize(instance_.getTasksNum(), Path());
  bool readingTaskAssignments = false, readingTaskPaths = false;
  // The path to the command when you use the vscode launch file
  string command =
      "./MAPF-PC/build/bin/task_assignment -m "
      "./MAPF-PC/sample_input/empty-16-16.map -a "
      "./MAPF-PC/sample_input/agents_goals.txt -k " +
      std::to_string(instance_.getAgentNum()) + " -t " +
      std::to_string(instance_.getTasksNum()) + " --solver CBS";

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
            PathEntry previousPEntry = {false, solution_.agents[agent].taskPaths[taskIndex - 1].back().location};
            taskPath.path.push_back(previousPEntry);
          }

          taskPath.path.push_back(pEntry);
          token.erase(0, localPos + 1);
          if (taskIndex > 0) {
            // Leftover token should now be the begin time information
            taskPath.beginTime = stoi(token) - 1;
          }
          else {
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

bool LNS::run() {

  bool success = false;
  if (initialSolutionStrategy == "greedy") {
    // Run the greedy task assignment and subsequent path finding algorithm
    success = buildGreedySolution();
  } else if (initialSolutionStrategy == "sota") {
    // Run the greedy task assignment and use CBS-PC for finding the paths of agents
    success = buildGreedySolutionWithCBSPC();
  }

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

  bool valid = validateSolution(&lnsNeighborhood.conflictedTasks);

  if (valid) {
    PLOGV << "Solution was found!\n";
    PLOGV << "MAPF-PC-LNS: "
          << "\n\tRuntime = " << runtime
          << "\n\tIterations = " << iterationStats.size()
          << "\n\tSolution Cost = " << solution_.sumOfCosts
          << "\n\tNumber of failures = " << numOfFailures << endl;
    return valid;
  }

  set<int> oldConflictedTasks = lnsNeighborhood.conflictedTasks;

  // LNS loop
  while (!valid && runtime < timeLimit_ &&
         (int)iterationStats.size() <= numOfIterations_) {

    int oldSolutionConflictNum;
    lnsNeighborhood.patchedTasks.clear();
    lnsNeighborhood.regretMaxHeap.clear();
    lnsNeighborhood.commitedTasks.clear();
    // lnsNeighborhood.conflictedTasksPathSize.clear();

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
      // TODO: Should we account for the invalidated paths tasks here as well?
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
    // Pick the best one and repeat the whole process aboe
    while (!lnsNeighborhood.conflictedTasks.empty()) {
      computeRegret((int)lnsNeighborhood.conflictedTasks.size() ==
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

    // Extract the set of conflicting tasks
    set<int> conflictedTasks;
    valid = validateSolution(&conflictedTasks);

    PLOGD << "Number of conflicts in old solution: " << oldSolutionConflictNum
          << endl;
    PLOGD << "Number of conflicts in new solution: " << conflictedTasks.size()
          << endl;

    if (!valid) {
      // Solution was not valid as we found some conflicts!
      PLOGE << "The solution was not valid!\n";
    }

    if (oldSolutionConflictNum < (int)conflictedTasks.size()) {
      // Reject this solution
      solution_ = previousSolution_;
      numOfFailures++;
      PLOGD << "Rejecting this solution!\n";
    } else if (oldSolutionConflictNum == (int)conflictedTasks.size()) {
      if (previousSolution_.sumOfCosts < solution_.sumOfCosts) {

        double acceptanceProb =
            exp((previousSolution_.sumOfCosts - solution_.sumOfCosts) /
                saTemperature);
        if ((double)rand() / (RAND_MAX) > acceptanceProb) {
          // Use simulated annealing to potentially accept worse solutions!
          numOfFailures = 0;
          lnsNeighborhood.conflictedTasks = conflictedTasks;
        } else {
          // Reject this solution
          solution_ = previousSolution_;
          numOfFailures++;
          PLOGD << "Rejecting this solution!\n";
        }
      } else {
        // Accept this solution
        numOfFailures = 0;
        lnsNeighborhood.conflictedTasks = conflictedTasks;
      }
    } else {
      // Accept this solution
      numOfFailures = 0;
      lnsNeighborhood.conflictedTasks = conflictedTasks;
    }

    runtime = ((fsec)(Time::now() - plannerStartTime_)).count();
    iterationStats.emplace_back(runtime, "LNS", instance_.getAgentNum(),
                                instance_.getTasksNum(), solution_.sumOfCosts);
    saTemperature *= saCoolingCoefficient;
  }

  std::cout << "MAPF-PC-LNS: "
            << "\n\tRuntime = " << runtime
            << "\n\tIterations = " << iterationStats.size()
            << "\n\tSolution Cost = " << solution_.sumOfCosts
            << "\n\tNumber of failures = " << numOfFailures << endl;

  return valid;
}

void LNS::prepareNextIteration() {
  PLOGI << "Preparing the solution object for the next iteration\n";

  vector<vector<int>> successors;
  successors.resize(instance_.getTasksNum());
  // Find the tasks that are following the earliest conflicting task as their paths need to be invalidated
  for (pair<int, int> precConstraint :
       instance_.getInputPrecedenceConstraints()) {
    // if (lnsNeighborhood.conflictedTasks.find(precConstraint.first) !=
    //     lnsNeighborhood.conflictedTasks.end()) {
    successors[precConstraint.first].push_back(precConstraint.second);
    // }
  }

  // TODO: Successor computation here is wrong!
  stack<int> q;
  for (int conflictTask : lnsNeighborhood.conflictedTasks) {
    q.push(conflictTask);
  }
  while (!q.empty() &&
         (int)lnsNeighborhood.conflictedTasks.size() <= neighborSize_) {
    int current = q.top();
    q.pop();
    if (lnsNeighborhood.conflictedTasks.find(current) !=
        lnsNeighborhood.conflictedTasks.end()) {
      continue;
    }
    lnsNeighborhood.conflictedTasks.insert(current);
    for (int succ : successors[current]) {
      if (lnsNeighborhood.conflictedTasks.find(succ) ==
          lnsNeighborhood.conflictedTasks.end()) {
        q.push(succ);
      }
    }
  }

  // If t_id is deleted then t_id + 1 task needs to be fixed
  set<int> tasksToFix, affectedAgents;
  for (int invalidTask : lnsNeighborhood.conflictedTasks) {

    int agent = solution_.getAgentWithTask(invalidTask),
        taskPosition = solution_.getLocalTaskIndex(agent, invalidTask),
        pathSize = (int)solution_.agents[agent]
                       .taskPaths[taskPosition]
                       .size();  // path_size is used for heuristic estimate
    PLOGD << "Invalidating task: " << invalidTask << ", Agent: " << agent
          << endl;

    // If the invalidated task was not the last local task of this agent then t_id + 1 exists
    if (taskPosition < (int)solution_.getAgentGlobalTasks(agent).size() - 1) {
      int nextTask = solution_.getAgentGlobalTasks(agent, taskPosition + 1);
      // Next task can still be undefined in the case where that task was removed in the previous iteration of this loop
      if (lnsNeighborhood.conflictedTasks.find(nextTask) ==
              lnsNeighborhood.conflictedTasks.end() &&
          nextTask != -1) {
        tasksToFix.insert(nextTask);
        PLOGD << "Next task: " << nextTask << endl;
      }
    }

    affectedAgents.insert(agent);
    // lnsNeighborhood.conflictedTasksPathSize.insert(
    //     make_pair(invalidTask, pathSize));

    // Marking past information about this conflicting task
    solution_.taskAgentMap[invalidTask] = UNASSIGNED;
    solution_.agents[agent].clearIntraAgentPrecedenceConstraint(invalidTask);
    // Needs to happen after clearing precedence constraints
    solution_.agents[agent].taskAssignments[taskPosition] = -1;
    solution_.agents[agent].taskPaths[taskPosition] = Path();
  }

  // Marking past information about conflicting tasks
  for (int affAgent : affectedAgents) {

    // For an affected agent there can be multiple conflicting tasks so need to do it this way
    solution_.agents[affAgent].path = Path();
    solution_.agents[affAgent].taskAssignments.erase(
        std::remove_if(solution_.agents[affAgent].taskAssignments.begin(),
                       solution_.agents[affAgent].taskAssignments.end(),
                       [](int task) { return task == -1; }),
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
    if (tasksToFix.find(task) != tasksToFix.end()) {

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
  agentsToCompute.reserve(affectedAgents.size());
  for (int affAgent : affectedAgents) {
    agentsToCompute.push_back(affAgent);
  }
  solution_.joinPaths(agentsToCompute);
}

void LNS::computeRegret(bool firstIteration) {
  lnsNeighborhood.regretMaxHeap.clear();
  for (int conflictTask : lnsNeighborhood.conflictedTasks) {
    // If we rejected the last iteration solution, then we are starting again. In the first loop of this iteration we can reuse the computation we did in the first loop of the last iteration. Rest would need to be computed again.
    if (numOfFailures > 0 && firstIteration) {
      // Compute f3 - f2, f4 - f3 etc as needed.

      // // debug for now
      // PLOGD << "Conflict Task: " << conflictTask << "\n";
      // auto allservicetimes = lnsNeighborhood.serviceTimesHeapMap[conflictTask];
      // while(!allservicetimes.empty()) {
      //   auto util = allservicetimes.top();
      //   PLOGD << "Util: Agent = " << util.agent << ", Value = " << util.value << ", Task Position = " << util.taskPosition << "\n";
      //   allservicetimes.pop();
      // }

      pairing_heap<Utility, compare<Utility::CompareNode>>
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
      Regret regret(conflictTask, bestUtility.agent, bestUtility.taskPosition,
                    nextBestValidUtility.value - bestUtility.value);
      lnsNeighborhood.regretMaxHeap.push(regret);
    } else {
      computeRegretForTask(conflictTask, firstIteration);
    }
  }
}

void LNS::computeRegretForTask(int task, bool firstIteration) {
  pairing_heap<Utility, compare<Utility::CompareNode>> serviceTimes;

  // v2: The task has to start after the earliest time step but needs to finish before the latest time step
  int earliestTimestep = 0;
  vector<int> inputPlanningOrder = instance_.getInputPlanningOrder();
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {

    // Gather the task assignments of this agent with any previous invalidated tasks included as needed relative to the target task
    vector<int> temporaryTaskAssignments = solution_.getAgentGlobalTasks(agent),
                newlyInsertedTask;
    int taskIndex =
        find(inputPlanningOrder.begin(), inputPlanningOrder.end(), task) -
        inputPlanningOrder.begin();
    for (int i = 0; i < taskIndex; i++) {
      if (lnsNeighborhood.conflictedTasks.find(inputPlanningOrder[i]) !=
          lnsNeighborhood.conflictedTasks.end()) {
        if (!lnsNeighborhood.commitedTasks[inputPlanningOrder[i]] &&
            previousSolution_.taskAgentMap[inputPlanningOrder[i]] == agent) {
          int localTaskPositionOffset = 0;
          // We need to compute the offset as we can invalidate multiple tasks associated with an agent. This means that simply querying the previous solution agent's task index is not enough as the it would be more than the actual task position value for the current solution
          for (int localTask :
               previousSolution_.agents[agent].taskAssignments) {
            // We dont need to bother for the tasks that come after the current one since we are considering them in planning order
            if (localTask == inputPlanningOrder[i]) {
              break;
            }
            if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                    lnsNeighborhood.immutableConflictedTasks.end() &&
                localTask != inputPlanningOrder[i]) {
              localTaskPositionOffset++;
            }
          }
          int localTaskPosition = previousSolution_.getLocalTaskIndex(
                                      agent, inputPlanningOrder[i]) -
                                  localTaskPositionOffset;
          temporaryTaskAssignments.insert(
              temporaryTaskAssignments.begin() + localTaskPosition,
              inputPlanningOrder[i]);
          newlyInsertedTask.push_back(inputPlanningOrder[i]);
        }
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
        // Same logic to compute the offset in case we invalidate multiple tasks on the same agent's queue
        int temporaryTaskLocalIndexOffset = 0;
        for (int localTask :
             previousSolution_.agents[temporaryTaskAgent].taskAssignments) {
          if (localTask == temporaryTask) {
            break;
          }
          if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                  lnsNeighborhood.immutableConflictedTasks.end() &&
              localTask != temporaryTask) {
            temporaryTaskLocalIndexOffset++;
          }
        }
        int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
                                          temporaryTaskAgent, temporaryTask) -
                                      temporaryTaskLocalIndexOffset;
        Path temporaryTaskPath = previousSolution_.agents[temporaryTaskAgent]
                                     .taskPaths[temporaryTaskLocalIndex];
        // Fix the begin time such that this task starts when the previous task ends
        if (i != 0) {
          temporaryTaskPath.beginTime =
              temporaryTaskPaths[temporaryTaskAssignments[i - 1]].endTime();
        }
        temporaryTaskPaths[temporaryTask] = temporaryTaskPath;
      }
      // In this case the previous task was recently inserted because it was in the conflict set and not committed to. Further the next task was patched in the prepare iteration function
      // This condition depends on the fact that the previous if condition would have triggered in earlier iteration of this loop
      else if (i > 0 &&
               find(newlyInsertedTask.begin(), newlyInsertedTask.end(),
                    temporaryTaskAssignments[i - 1]) !=
                   newlyInsertedTask.end() &&
               lnsNeighborhood.patchedTasks.find(temporaryTask) !=
                   lnsNeighborhood.patchedTasks.end()) {
        int temporaryTaskAgent = previousSolution_.taskAgentMap[temporaryTask];
        assert(temporaryTaskAgent != UNASSIGNED);
        // Same logic to compute the offset in case we invalidate multiple tasks on the same agent's queue
        int temporaryTaskLocalIndexOffset = 0;
        for (int localTask :
             previousSolution_.agents[temporaryTaskAgent].taskAssignments) {
          if (localTask == temporaryTask) {
            break;
          }
          if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                  lnsNeighborhood.immutableConflictedTasks.end() &&
              localTask != temporaryTask) {
            temporaryTaskLocalIndexOffset++;
          }
        }
        int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
                                          temporaryTaskAgent, temporaryTask) -
                                      temporaryTaskLocalIndexOffset;
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
    // Gather the paths of the tasks that need to be done before this task but have been assigned to some other agents
    set<int>
        agentsNewlyAddedTasksBelongTo;  // This is for agents apart from "agent" i.e the loop variable
    for (int i = 0; i < taskIndex; i++) {
      int ancestorTask = inputPlanningOrder[i];
      if (temporaryTaskPaths[ancestorTask].empty()) {
        if (lnsNeighborhood.commitedTasks.count(ancestorTask) > 0 &&
            !lnsNeighborhood.commitedTasks[ancestorTask]) {
          int temporaryTaskAgent = previousSolution_.taskAgentMap[ancestorTask];
          assert(temporaryTaskAgent != UNASSIGNED);
          // Same logic to compute the offset in case we invalidate multiple tasks on the same agent's queue
          int temporaryTaskLocalIndexOffset = 0;
          for (int localTask :
               previousSolution_.agents[temporaryTaskAgent].taskAssignments) {
            if (localTask == ancestorTask) {
              break;
            }
            if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                    lnsNeighborhood.immutableConflictedTasks.end() &&
                localTask != ancestorTask) {
              temporaryTaskLocalIndexOffset++;
            }
          }
          int temporaryTaskLocalIndex = previousSolution_.getLocalTaskIndex(
                                            temporaryTaskAgent, ancestorTask) -
                                        temporaryTaskLocalIndexOffset;
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
          if (agentsNewlyAddedTasksBelongTo.find(temporaryTaskAgent) !=
                  agentsNewlyAddedTasksBelongTo.end() &&
              lnsNeighborhood.patchedTasks.find(ancestorTask) !=
                  lnsNeighborhood.patchedTasks.end()) {
            // Same logic to compute the offset in case we invalidate multiple tasks on the same agent's queue
            int temporaryTaskLocalIndexOffset = 0;
            for (int localTask :
                 previousSolution_.agents[temporaryTaskAgent].taskAssignments) {
              if (localTask == ancestorTask) {
                break;
              }
              if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                      lnsNeighborhood.immutableConflictedTasks.end() &&
                  localTask != ancestorTask) {
                temporaryTaskLocalIndexOffset++;
              }
            }
            int temporaryTaskLocalIndex =
                previousSolution_.getLocalTaskIndex(temporaryTaskAgent,
                                                    ancestorTask) -
                temporaryTaskLocalIndexOffset;
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

    vector<vector<int>> ancestors;
    ancestors.resize(instance_.getTasksNum());
    for (pair<int, int> precedenceConstraint :
         instance_.getInputPrecedenceConstraints()) {
      ancestors[precedenceConstraint.second].push_back(
          precedenceConstraint.first);
    }

    set<int> setOfTasksToComplete;
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
    for (int ancestor : setOfTasksToComplete) {
      assert(!temporaryTaskPaths[ancestor].empty());
      earliestTimestep =
          max(earliestTimestep, temporaryTaskPaths[ancestor].endTime());
    }
    // vector<int> taskAncestors = instance_.getTaskDependencies()[task];
    // for (int ancestor : taskAncestors) {
    //   assert(!temporaryTaskPaths[ancestor].empty());
    //   earliestTimestep =
    //       max(earliestTimestep, temporaryTaskPaths[ancestor].endTime());
    // }

    // v2: Append the agent's internal precedence constraints to the input precedence constraints
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
  Utility bestUtility = serviceTimes.top();
  serviceTimes.pop();
  Utility secondBestUtility = serviceTimes.top();
  Regret regret(task, bestUtility.agent, bestUtility.taskPosition,
                secondBestUtility.value - bestUtility.value);
  lnsNeighborhood.regretMaxHeap.push(regret);
}

void LNS::computeRegretForTaskWithAgent(
    TaskRegretPacket regretPacket, vector<int>* taskAssignments,
    vector<Path>* taskPaths, vector<pair<int, int>>* precedenceConstraints,
    pairing_heap<Utility, compare<Utility::CompareNode>>* serviceTimes) {

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

    // // Compute the distance estimate it would take to finish the insertion
    // int distance = 0;
    // if (j > 0) {
    //   distance += instance_.getManhattanDistance(
    //       instance_.getTaskLocations((*taskAssignments)[j - 1]),
    //       instance_.getTaskLocations(regretPacket.task));
    // } else {
    //   distance += instance_.getManhattanDistance(
    //       solution_.agents[regretPacket.agent].pathPlanner->startLocation,
    //       instance_.getTaskLocations(regretPacket.task));
    // }

    // // If the computed distance estimated is longer than the original path size then why bother
    // if (distance > lnsNeighborhood.conflictedTasksPathSize[regretPacket.task]) {
    //   continue;
    // }

    regretPacket.taskPosition = j;
    vector<Path> temporaryTaskPaths = *taskPaths;
    vector<int> temporaryTaskAssignments = *taskAssignments;
    vector<pair<int, int>> temporaryPrecedenceConstraints =
        *precedenceConstraints;
    // Create a copy of the task assignments, paths and corresponding precedence constraints so that they dont get modified
    Utility utility =
        insertTask(regretPacket, &temporaryTaskPaths, &temporaryTaskAssignments,
                   &temporaryPrecedenceConstraints);
    serviceTimes->push(utility);
  }
}

// Need the task paths, assignments and precedence constraints as pointers so that we can reuse this code when commiting as we can make in-place changes to these data-structures
Utility LNS::insertTask(TaskRegretPacket regretPacket, vector<Path>* taskPaths,
                        vector<int>* taskAssignments,
                        vector<pair<int, int>>* precedenceConstraints,
                        bool commit) {

  double pathSizeChange = 0;
  int startTime = 0, previousTask = -1, nextTask = -1;

  // The task paths are all the task paths when we dont commit but if we commit they will be agent specific task paths
  vector<Path>& taskPathsRef = *taskPaths;
  vector<int>& taskAssignmentsRef = *taskAssignments;
  vector<pair<int, int>>& precedenceConstraintsRef = *precedenceConstraints;

  int agentTasksSize = (int)taskAssignmentsRef.size();

  // In this case we are inserting a task between two existing tasks
  if (regretPacket.taskPosition < agentTasksSize) {

    taskAssignmentsRef.insert(
        taskAssignmentsRef.begin() + regretPacket.taskPosition,
        regretPacket.task);

    nextTask = taskAssignmentsRef[regretPacket.taskPosition + 1];

    if (commit) {
      // Invalidate the path of the next task
      // Compute the path size of the next task before you remove it!
      pathSizeChange = (double)taskPathsRef[regretPacket.taskPosition].size();
      taskPathsRef[regretPacket.taskPosition] = Path();
    } else {
      taskPathsRef[regretPacket.task] = Path();
      pathSizeChange = (double)taskPathsRef[nextTask].size();
    }

    precedenceConstraintsRef.emplace_back(regretPacket.task, nextTask);

    // If we are NOT inserting at the start position then we need to take care of the previous task as well
    if (regretPacket.taskPosition != 0) {
      previousTask = taskAssignmentsRef[regretPacket.taskPosition - 1];
      if (!commit) {
        startTime = taskPathsRef[previousTask].endTime();
      } else {
        startTime = taskPathsRef[regretPacket.taskPosition - 1].endTime();
      }
      precedenceConstraintsRef.erase(
          std::remove_if(
              precedenceConstraintsRef.begin(), precedenceConstraintsRef.end(),
              [previousTask, nextTask](pair<int, int> x) {
                return x.first == previousTask && x.second == nextTask;
              }),
          precedenceConstraintsRef.end());
      precedenceConstraintsRef.emplace_back(previousTask, regretPacket.task);
    }

    if (commit) {
      // Insert an empty path between previous and next tasks
      // Only need to insert in the commit phase since we will be sending an updated task paths reference object with the new task placeholder before calling this function
      taskPathsRef.insert(taskPathsRef.begin() + regretPacket.taskPosition,
                          Path());
    }

  }
  // In this case we are inserting at the very end
  else if (regretPacket.taskPosition == agentTasksSize) {

    previousTask = taskAssignmentsRef[regretPacket.taskPosition - 1];
    if (commit) {
      startTime = taskPathsRef[regretPacket.taskPosition - 1].endTime();
    } else {
      startTime = taskPathsRef[previousTask].endTime();
    }

    taskAssignmentsRef.push_back(regretPacket.task);
    precedenceConstraintsRef.emplace_back(previousTask, regretPacket.task);

    if (commit) {
      taskPathsRef.emplace_back();
    }
  }

  vector<int> goalLocations = instance_.getTaskLocations(taskAssignmentsRef);
  ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
  if (!commit) {
    MultiLabelSpaceTimeAStar localPlanner =
        MultiLabelSpaceTimeAStar(instance_, regretPacket.agent);
    localPlanner.setGoalLocations(goalLocations);
    localPlanner.computeHeuristics();

    buildConstraintTable(constraintTable, regretPacket.task,
                         goalLocations[regretPacket.taskPosition],
                         &taskPathsRef, &precedenceConstraintsRef);
    taskPathsRef[regretPacket.task] = localPlanner.findPathSegment(
        constraintTable, startTime, regretPacket.taskPosition, 0);

    if (nextTask != -1) {
      startTime = taskPathsRef[regretPacket.task].endTime();
      // TODO: Should this also include logic for the patched tasks?
      // The task paths reference does not have ancestor information about next task, so we need to add those in
      vector<int> planningOrder = instance_.getInputPlanningOrder();
      set<int> agentsNewlyAddedTasksBelongTo;
      int nextTaskIndex =
          find(planningOrder.begin(), planningOrder.end(), nextTask) -
          planningOrder.begin();
      for (int i = 0; i < nextTaskIndex; i++) {
        int nextTaskAncestor = planningOrder[i];
        // The ancestor of the next task is not assigned. It can either be in the previous solution if and only if the ancestor was in the conflict set but not committed it. Otherwise it must be in the solution.
        if (taskPathsRef[nextTaskAncestor].empty()) {
          if (lnsNeighborhood.commitedTasks.count(nextTaskAncestor) > 0 &&
              !lnsNeighborhood.commitedTasks[nextTaskAncestor]) {
            int nextTaskAncestorAgent =
                previousSolution_.taskAgentMap[nextTaskAncestor];
            assert(nextTaskAncestorAgent != UNASSIGNED);
            // Same logic to compute the offset in case we invalidate multiple tasks on the same agent's queue
            int nextTaskLocalIndexOffset = 0;
            for (int localTask : previousSolution_.agents[nextTaskAncestorAgent]
                                     .taskAssignments) {
              if (localTask == nextTaskAncestor) {
                break;
              }
              if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                      lnsNeighborhood.immutableConflictedTasks.end() &&
                  localTask != nextTaskAncestor) {
                nextTaskLocalIndexOffset++;
              }
            }
            int nextTaskAncestorLocalIndex =
                previousSolution_.getLocalTaskIndex(nextTaskAncestorAgent,
                                                    nextTaskAncestor) -
                nextTaskLocalIndexOffset;
            // TODO: Should we change the begin time of this task?
            taskPathsRef[nextTaskAncestor] =
                previousSolution_.agents[nextTaskAncestorAgent]
                    .taskPaths[nextTaskAncestorLocalIndex];
            agentsNewlyAddedTasksBelongTo.insert(nextTaskAncestorAgent);
          } else {
            int nextTaskAncestorAgent =
                solution_.taskAgentMap[nextTaskAncestor];
            assert(nextTaskAncestorAgent != UNASSIGNED);
            if (agentsNewlyAddedTasksBelongTo.find(nextTaskAncestorAgent) !=
                    agentsNewlyAddedTasksBelongTo.end() &&
                lnsNeighborhood.patchedTasks.find(nextTaskAncestor) !=
                    lnsNeighborhood.patchedTasks.end()) {
              // Same logic to compute the offset in case we invalidate multiple tasks on the same agent's queue
              int nextTaskLocalIndexOffset = 0;
              for (int localTask :
                   previousSolution_.agents[nextTaskAncestorAgent]
                       .taskAssignments) {
                if (localTask == nextTaskAncestor) {
                  break;
                }
                if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                        lnsNeighborhood.immutableConflictedTasks.end() &&
                    localTask != nextTaskAncestor) {
                  nextTaskLocalIndexOffset++;
                }
              }
              int nextTaskAncestorLocalIndex =
                  previousSolution_.getLocalTaskIndex(nextTaskAncestorAgent,
                                                      nextTaskAncestor) -
                  nextTaskLocalIndexOffset;
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

      buildConstraintTable(constraintTable, nextTask,
                           goalLocations[regretPacket.taskPosition + 1],
                           &taskPathsRef, &precedenceConstraintsRef);
      taskPathsRef[nextTask] = localPlanner.findPathSegment(
          constraintTable, startTime, regretPacket.taskPosition + 1, 0);
    }
  } else {

    solution_.agents[regretPacket.agent].pathPlanner->setGoalLocations(
        goalLocations);
    solution_.agents[regretPacket.agent].pathPlanner->computeHeuristics();

    buildConstraintTable(constraintTable, regretPacket.task);
    taskPathsRef[regretPacket.taskPosition] =
        solution_.agents[regretPacket.agent].pathPlanner->findPathSegment(
            constraintTable, startTime, regretPacket.taskPosition, 0);
    solution_.agents[regretPacket.agent].insertIntraAgentPrecedenceConstraint(
        regretPacket.task, regretPacket.taskPosition);
    solution_.taskAgentMap[regretPacket.task] = regretPacket.agent;

    if (nextTask != -1) {
      startTime = taskPathsRef[regretPacket.taskPosition].endTime();

      for (int ancestorNextTask : instance_.getInputPlanningOrder()) {
        if (ancestorNextTask == nextTask) {
          break;
        }
        // Need an extra condition to ensure that we dont trigger this code block for the best regret task as it would not have been "committed" yet
        if (lnsNeighborhood.commitedTasks.count(ancestorNextTask) > 0 &&
            !lnsNeighborhood.commitedTasks[ancestorNextTask] &&
            ancestorNextTask != regretPacket.task) {
          // This case relates to the tasks in the conflict set that need to be addressed before the next task task can be committed
          int ancestorNextTaskAgent =
              previousSolution_.taskAgentMap[ancestorNextTask];
          assert(ancestorNextTaskAgent != UNASSIGNED);
          int ancestorNextTaskPositionOffset = 0;
          for (int localTask : previousSolution_.agents[ancestorNextTaskAgent]
                                   .taskAssignments) {
            if (localTask == ancestorNextTask) {
              break;
            }
            if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                    lnsNeighborhood.immutableConflictedTasks.end() &&
                localTask != ancestorNextTask) {
              ancestorNextTaskPositionOffset++;
            }
          }
          int ancestorNextTaskPosition =
              previousSolution_.getLocalTaskIndex(ancestorNextTaskAgent,
                                                  ancestorNextTask) -
              ancestorNextTaskPositionOffset;
          Path ancestorPath = previousSolution_.agents[ancestorNextTaskAgent]
                                  .taskPaths[ancestorNextTaskPosition];
          solution_.agents[ancestorNextTaskAgent]
              .pathPlanner->goalLocations.insert(
                  solution_.agents[ancestorNextTaskAgent]
                          .pathPlanner->goalLocations.begin() +
                      ancestorNextTaskPosition,
                  instance_.getTaskLocations(ancestorNextTask));
          solution_.agents[ancestorNextTaskAgent]
              .pathPlanner->computeHeuristics();
          solution_.agents[ancestorNextTaskAgent].taskAssignments.insert(
              solution_.agents[ancestorNextTaskAgent].taskAssignments.begin() +
                  ancestorNextTaskPosition,
              ancestorNextTask);
          solution_.agents[ancestorNextTaskAgent]
              .insertIntraAgentPrecedenceConstraint(ancestorNextTask,
                                                    ancestorNextTaskPosition);
          solution_.agents[ancestorNextTaskAgent].taskPaths.insert(
              solution_.agents[ancestorNextTaskAgent].taskPaths.begin() +
                  ancestorNextTaskPosition,
              ancestorPath);
          solution_.taskAgentMap[ancestorNextTask] = ancestorNextTaskAgent;

          // The ancestor of the next task that was in the conflict set has now been committed using its old path, hence we need to mark it as resolved now
          lnsNeighborhood.conflictedTasks.erase(ancestorNextTask);
          // lnsNeighborhood.conflictedTasksPathSize.erase(ancestorNextTask);
          lnsNeighborhood.commitedTasks[ancestorNextTask] = true;

          // Now if the next task on this agent's task queue was patched during the prepare iteration function phase then we need to remove it from that object and reuse its old path!
          if (ancestorNextTask != (int)solution_.agents[ancestorNextTaskAgent]
                                      .taskAssignments.back() &&
              lnsNeighborhood.patchedTasks.find(
                  solution_.agents[ancestorNextTaskAgent]
                      .taskAssignments[ancestorNextTaskPosition + 1]) !=
                  lnsNeighborhood.patchedTasks.end()) {
            int localNextTask =
                solution_.agents[ancestorNextTaskAgent]
                    .taskAssignments[ancestorNextTaskPosition + 1];
            int localNextTaskOldPositionOffset = 0;
            for (int localTask : previousSolution_.agents[ancestorNextTaskAgent]
                                     .taskAssignments) {
              if (localTask == localNextTask) {
                break;
              }
              if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                      lnsNeighborhood.immutableConflictedTasks.end() &&
                  localTask != localNextTask) {
                localNextTaskOldPositionOffset++;
              }
            }
            int localNextTaskOldPosition =
                previousSolution_.getLocalTaskIndex(ancestorNextTaskAgent,
                                                    localNextTask) -
                localNextTaskOldPositionOffset;
            // Dont want to mess with the begin time of this task because if some other ancestor task would get in between these two then the begin time would be wrong if edited here
            solution_.agents[ancestorNextTaskAgent]
                .taskPaths[ancestorNextTaskPosition + 1] =
                previousSolution_.agents[ancestorNextTaskAgent]
                    .taskPaths[localNextTaskOldPosition];
            lnsNeighborhood.patchedTasks.erase(localNextTask);
          }
          // Fix the begin times based on whatever new paths were added
          for (int k = ancestorNextTaskPosition + 1;
               k < (int)solution_.agents[ancestorNextTaskAgent]
                       .taskAssignments.size();
               k++) {
            solution_.agents[ancestorNextTaskAgent].taskPaths[k].beginTime =
                solution_.agents[ancestorNextTaskAgent]
                    .taskPaths[k - 1]
                    .endTime();
          }
        }
      }

      buildConstraintTable(constraintTable, nextTask);
      int nextTaskPosition =
          find(taskAssignmentsRef.begin(), taskAssignmentsRef.end(), nextTask) -
          taskAssignmentsRef.begin();
      assert(nextTaskPosition - 1 >= 0);
      startTime = taskPathsRef[nextTaskPosition - 1].endTime();
      taskPathsRef[nextTaskPosition] =
          solution_.agents[regretPacket.agent].pathPlanner->findPathSegment(
              constraintTable, startTime, nextTaskPosition, 0);
    }

    for (int k = regretPacket.taskPosition + 1;
         k < (int)taskAssignmentsRef.size(); k++) {
      taskPathsRef[k].beginTime = taskPathsRef[k - 1].endTime();
    }
  }

  if (!commit) {
    double value =
        -pathSizeChange + (double)taskPathsRef[regretPacket.task].size();
    if (nextTask != -1) {
      value += (double)taskPathsRef[nextTask].size();
    }

    Utility utility(regretPacket.agent, regretPacket.taskPosition, value);
    return utility;
  }
  return {};
}

void LNS::commitBestRegretTask(Regret bestRegret) {

  PLOGD << "Commiting for task " << bestRegret.task << " to agent "
        << bestRegret.agent << " with regret = " << bestRegret.value << endl;
  TaskRegretPacket bestRegretPacket = {
      bestRegret.task, bestRegret.agent, bestRegret.taskPosition, {}};
  for (int ancestorTask : instance_.getInputPlanningOrder()) {
    if (ancestorTask == bestRegret.task) {
      break;
    }
    if (lnsNeighborhood.commitedTasks.count(ancestorTask) > 0 &&
        !lnsNeighborhood.commitedTasks[ancestorTask]) {
      // This case relates to the tasks in the conflict set that need to be addressed before the best regret task can be committed
      int ancestorTaskAgent = previousSolution_.taskAgentMap[ancestorTask];
      assert(ancestorTaskAgent != UNASSIGNED);
      PLOGD
          << "Ancestor task assignment size: "
          << previousSolution_.agents[ancestorTaskAgent].taskAssignments.size()
          << "\n";
      int ancestorTaskPositionOffset = 0;
      for (int localTask :
           previousSolution_.agents[ancestorTaskAgent].taskAssignments) {
        if (localTask == ancestorTask) {
          break;
        }
        if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                lnsNeighborhood.immutableConflictedTasks.end() &&
            localTask != ancestorTask) {
          ancestorTaskPositionOffset++;
        }
      }
      int ancestorTaskPosition =
          previousSolution_.getLocalTaskIndex(ancestorTaskAgent, ancestorTask) -
          ancestorTaskPositionOffset;
      Path ancestorPath = previousSolution_.agents[ancestorTaskAgent]
                              .taskPaths[ancestorTaskPosition];
      solution_.agents[ancestorTaskAgent].pathPlanner->goalLocations.insert(
          solution_.agents[ancestorTaskAgent]
                  .pathPlanner->goalLocations.begin() +
              ancestorTaskPosition,
          instance_.getTaskLocations(ancestorTask));
      solution_.agents[ancestorTaskAgent].pathPlanner->computeHeuristics();
      solution_.agents[ancestorTaskAgent].taskAssignments.insert(
          solution_.agents[ancestorTaskAgent].taskAssignments.begin() +
              ancestorTaskPosition,
          ancestorTask);
      solution_.agents[ancestorTaskAgent].insertIntraAgentPrecedenceConstraint(
          ancestorTask, ancestorTaskPosition);
      solution_.agents[ancestorTaskAgent].taskPaths.insert(
          solution_.agents[ancestorTaskAgent].taskPaths.begin() +
              ancestorTaskPosition,
          ancestorPath);
      solution_.taskAgentMap[ancestorTask] = ancestorTaskAgent;

      // The ancestor of the task that was in the conflict set has now been committed using its old path, hence we need to mark it as resolved now
      lnsNeighborhood.conflictedTasks.erase(ancestorTask);
      // lnsNeighborhood.conflictedTasksPathSize.erase(ancestorTask);
      lnsNeighborhood.commitedTasks[ancestorTask] = true;

      // Now if the next task on this agent's task queue was patched during the prepare iteration function phase then we need to remove it from that object and reuse its old path!
      if (ancestorTask !=
              (int)solution_.agents[ancestorTaskAgent].taskAssignments.back() &&
          lnsNeighborhood.patchedTasks.find(
              solution_.agents[ancestorTaskAgent]
                  .taskAssignments[ancestorTaskPosition + 1]) !=
              lnsNeighborhood.patchedTasks.end()) {
        int localNextTask = solution_.agents[ancestorTaskAgent]
                                .taskAssignments[ancestorTaskPosition + 1];
        int localNextTaskOldPositionOffset = 0;
        for (int localTask :
             previousSolution_.agents[ancestorTaskAgent].taskAssignments) {
          if (localTask == localNextTask) {
            break;
          }
          if (lnsNeighborhood.immutableConflictedTasks.find(localTask) !=
                  lnsNeighborhood.immutableConflictedTasks.end() &&
              localTask != localNextTask) {
            localNextTaskOldPositionOffset++;
          }
        }
        int localNextTaskOldPosition = previousSolution_.getLocalTaskIndex(
                                           ancestorTaskAgent, localNextTask) -
                                       localNextTaskOldPositionOffset;
        // Dont want to mess with the begin time of this task because if some other ancestor task would get in between these two then the begin time would be wrong if edited here
        solution_.agents[ancestorTaskAgent]
            .taskPaths[ancestorTaskPosition + 1] =
            previousSolution_.agents[ancestorTaskAgent]
                .taskPaths[localNextTaskOldPosition];
        lnsNeighborhood.patchedTasks.erase(localNextTask);
      }
      // Fix the begin times based on whatever new paths were added
      for (int k = ancestorTaskPosition + 1;
           k < (int)solution_.agents[ancestorTaskAgent].taskAssignments.size();
           k++) {
        solution_.agents[ancestorTaskAgent].taskPaths[k].beginTime =
            solution_.agents[ancestorTaskAgent].taskPaths[k - 1].endTime();
      }
    }
  }

  // Once the setup is complete for this best regret task then we can compute the current precedence constraints
  vector<pair<int, int>> precedenceConstraints =
      instance_.getInputPrecedenceConstraints();
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    precedenceConstraints.insert(
        precedenceConstraints.end(),
        solution_.agents[agent].intraPrecedenceConstraints.begin(),
        solution_.agents[agent].intraPrecedenceConstraints.end());
  }

  // At this point any previously empty paths must be resolved and not we can use the insert task function to commit to the actual best regret task
  // TODO: Can it happen that some ancestor gets inserted before this task on the same agent's task queue?
  insertTask(bestRegretPacket,
             &solution_.agents[bestRegretPacket.agent].taskPaths,
             &solution_.agents[bestRegretPacket.agent].taskAssignments,
             &precedenceConstraints, true);

  lnsNeighborhood.conflictedTasks.erase(bestRegret.task);
  // lnsNeighborhood.conflictedTasksPathSize.erase(bestRegret.task);
  lnsNeighborhood.commitedTasks[bestRegret.task] = true;
}

void LNS::buildConstraintTable(ConstraintTable& constraintTable, int task,
                               int taskLocation, vector<Path>* taskPaths,
                               vector<pair<int, int>>* precedenceConstraints) {
  constraintTable.goalLocation = taskLocation;

  // Could have used the instance's precendence constraint ordering but this one would be smaller than that
  // TODO: Should we not use the ancestors computed using this method?
  vector<vector<int>> ancestors;
  ancestors.resize(instance_.getTasksNum());
  // for (pair<int, int> precedenceConstraint : (*precedenceConstraints)) {
  //   ancestors[precedenceConstraint.second].push_back(
  //       precedenceConstraint.first);
  // }
  for (pair<int, int> precedenceConstraint :
       instance_.getInputPrecedenceConstraints()) {
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

  // Identify which of the tasks that need to be done before the target task was/is the final task of its assigned agent
  unordered_map<int, int> lastTasks;
  for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
    // Loop through these tasks to ensure that the once it finishes the last tasks map is fully updated
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
  // Only care about tasks that are not in conflict right now
  vector<pair<int, int>> precedenceConstraints =
      instance_.getInputPrecedenceConstraints();
  // TODO: Can we really ignore these precedence constraints??
  // for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
  //   precedenceConstraints.insert(
  //       precedenceConstraints.end(),
  //       solution_.agents[agent].intraPrecedenceConstraints.begin(),
  //       solution_.agents[agent].intraPrecedenceConstraints.end());
  // }

  vector<vector<int>> ancestors;
  ancestors.resize(instance_.getTasksNum());
  for (pair<int, int> precedenceConstraint : precedenceConstraints) {
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
