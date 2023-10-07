#include "lns.hpp"
#include <numeric>
#include "utils.hpp"

bool
isSamePath(const Path& p1, const Path& p2)
{
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

LNS::LNS(int numOfIterations, const Instance& instance, int neighborSize, double timeLimit)
  : numOfIterations_(numOfIterations)
  , neighborSize_(neighborSize)
  , instance_(instance)
  , solution_(instance)
  , previousSolution_(instance)
  , timeLimit_(timeLimit)
{
    plannerStartTime_ = Time::now();
}

bool LNS::buildGreedySolution() {

    // Assign tasks
    greedyTaskAssignment(&instance_, &solution_);
    for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        vector<int> taskLocations = instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
        solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
        solution_.agents[agent].taskPaths.resize(solution_.getAgentGlobalTasks(agent).size(), Path());
        solution_.agents[agent].pathPlanner->computeHeuristics();
    }

    // Compute the precedence constraints based on current task assignments
    // Intra-agent precedence constraints
    for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        for (int task = 1; task < (int)solution_.getAgentGlobalTasks(agent).size(); task++) {
            solution_.insertPrecedenceConstraint(solution_.taskAssignments[agent][task - 1],
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
    bool success = topologicalSort(&instance_, &solution_.precedenceConstraints, planningOrder);
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
            PLOGE << "No path exists for agent " << agent << " and task " << task << endl;
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

bool LNS::run()
{

    bool success = buildGreedySolution();
    if (!success) {
        return success;
    }

    printPaths();

    initialSolutionRuntime_ = ((fsec)(Time::now() - plannerStartTime_)).count();
    iterationStats.emplace_back(initialSolutionRuntime_,
                                 "greedy",
                                 instance_.getAgentNum(),
                                 instance_.getTasksNum(),
                                 solution_.sumOfCosts);
    runtime = initialSolutionRuntime_;

    PLOGD << "Initial solution cost = " << solution_.sumOfCosts << ", Runtime = " << initialSolutionRuntime_ << endl;

    // LNS loop
    while (runtime < timeLimit_ && (int)iterationStats.size() <= numOfIterations_) {

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

        PLOGD << "Number of conflicts in old solution: " << previousSolution_.neighbor.conflictedTasks.size() << endl;
        PLOGD << "Number of conflicts in new solution: " << conflictedTasks.size() << endl;

        if (previousSolution_.neighbor.conflictedTasks.size() < conflictedTasks.size()) {
            // Reject this solution
            for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
                delete solution_.agents[agent].pathPlanner;
            }
            solution_ = previousSolution_;
            PLOGD << "Rejecting this solution!\n";
        } else if (previousSolution_.neighbor.conflictedTasks.size() == conflictedTasks.size()) {
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
          << "\n\tRuntime = " << runtime << "\n\tIterations = " << iterationStats.size()
          << "\n\tSolution Cost = " << sumOfCosts
          << "\n\tNumber of failures = " << numOfFailures << endl;

    return true;
}

void
LNS::prepareNextIteration()
{
    // Remove the conflicted tasks from the agents paths and recompile their paths
    PLOGI << "Preparing the solution object for next iteration\n";

    for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        solution_.agents[agent].pathPlanner = new MultiLabelSpaceTimeAStar(instance_, agent);
        vector<int> taskLocations = instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
        solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
        solution_.agents[agent].pathPlanner->computeHeuristics();
    }

    vector<int> tasksToFix; // If t_id is deleted then t_id + 1 task needs to be fixed
    unordered_map<int, int> affectedAgents, globalToLocalTaskId;
    for (int ct : solution_.neighbor.conflictedTasks) {

        int agent = solution_.getAgentWithTask(ct),
            ctPosition = solution_.getLocalTaskIndex(agent, ct),
            pathSize = (int)solution_.paths[ct].size(); // path_size is used for heuristic estimate
        PLOGD << "Conflicting task: " << ct << ", Agent: " << agent << endl;

        // If the conflicted task was not the last local task of this agent then t_id + 1 exists
        if (ctPosition != (int)solution_.getAgentGlobalTasks(agent).size() - 1) {
            int nextTask = solution_.getAgentGlobalTasks(agent, ctPosition + 1);
            if (solution_.neighbor.conflictedTasks.find(nextTask) == solution_.neighbor.conflictedTasks.end()) {
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
    assert(topologicalSort(&instance_, &solution_.precedenceConstraints, planningOrder));

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

        vector<int> taskLocations = instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent));
        solution_.agents[agent].pathPlanner->setGoalLocations(taskLocations);
        solution_.agents[agent].pathPlanner->computeHeuristics();
    }

    // Find the paths for the tasks whose previous tasks were removed
    for (int task : planningOrder) {
        if (std::find(tasksToFix.begin(), tasksToFix.end(), task) != tasksToFix.end()) {

            PLOGD << "Going to find path for next task: " << task << endl;

            int startTime = 0, agent = solution_.getAgentWithTask(task);
            int taskPosition = solution_.getLocalTaskIndex(agent, task);

            if (taskPosition != 0) {
                startTime = solution_.agents[agent].taskPaths[taskPosition - 1].endTime();
            }
            assert(taskPosition <= (int)solution_.getAgentGlobalTasks(agent).size() - 1);

            ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
            buildConstraintTable(constraintTable, task);
            solution_.paths[task] = solution_.agents[agent].pathPlanner->findPathSegment(
              constraintTable, startTime, taskPosition, 0);
            solution_.agents[agent].taskPaths[taskPosition] = solution_.paths[task];

            // Once the path was found fix the begin times for subsequent tasks of the agent
            for (int k = taskPosition + 1; k < (int)solution_.getAgentGlobalTasks(agent).size(); k++) {
                solution_.agents[agent].taskPaths[k].beginTime = solution_.agents[agent].taskPaths[k - 1].endTime();
            }
        }
    }

    vector<int> agentsToCompute;
    for (pair<int, int> taskAgent : affectedAgents) {
        agentsToCompute.push_back(taskAgent.second);
    }
    solution_.joinPaths(agentsToCompute);
}

void
LNS::computeRegret()
{
    solution_.neighbor.regretMaxHeap.clear();
    for (int task : solution_.neighbor.conflictedTasks) {
        computeRegretForTask(task);
    }
}

void
LNS::computeRegretForTask(int task)
{
    pairing_heap<Utility, compare<Utility::CompareNode>> serviceTimes;

    // Find the precedence constraints involving the task or any other task that is not in the conflicting set
    vector<pair<int, int>> precedenceConstraints;
    for (pair<int, int> pc : solution_.precedenceConstraints) {
        if (pc.first == task || pc.second == task ||
            (std::find_if(solution_.neighbor.conflictedTasks.begin(),
                          solution_.neighbor.conflictedTasks.end(),
                          [pc](int task) { return pc.first == task || pc.second == task; })) ==
              solution_.neighbor.conflictedTasks.end()) {
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
        ancestors[precedenceConstraint.second].push_back(precedenceConstraint.first);
    }

    stack<int> q({ task });
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
                PLOGD << "Going to update earliest timestep from " << earliestTimestep << endl;
                earliestTimestep = solution_.agents[agent].path.timeStamps[taskIdx];
                PLOGD << "New earliest timestep " << earliestTimestep << endl;
                PLOGD << "Came from " << agent << ", " << current << " at " << taskIdx << endl;
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
            PLOGD << "Going to update latest timestep from " << latestTimestep << endl;
            latestTimestep = solution_.agents[agent].path.timeStamps[taskIdx];
            PLOGD << "New latest timestep " << latestTimestep << endl;
            PLOGD << "Came from " << agent << ", " << succ << " at " << taskIdx << endl;
        }
    }

    for (int agent = 0; agent < instance_.getAgentNum(); agent++) {
        computeRegretForTaskWithAgent(
          task, agent, earliestTimestep, latestTimestep, &precedenceConstraints, &serviceTimes);
    }

    Utility bestUtility = serviceTimes.top();
    serviceTimes.pop();
    Utility secondBestUtility = serviceTimes.top();
    Regret regret(task,
                  bestUtility.agent,
                  bestUtility.taskPosition,
                  secondBestUtility.value - bestUtility.value);
    solution_.neighbor.regretMaxHeap.push(regret);
}

void
LNS::computeRegretForTaskWithAgent(
  int task,
  int agent,
  int earliestTimestep,
  int latestTimestep,
  vector<pair<int, int>>* precedenceConstraints,
  pairing_heap<Utility, compare<Utility::CompareNode>>* serviceTimes)
{

    // Compute the first position along the agent's task assignments where we can insert this task
    int firstValidPosition = 0, lastValidPosition = solution_.getAgentGlobalTasks(agent).size() + 1;
    vector<int> agentTasks = solution_.getAgentGlobalTasks(agent);
    for (int j = (int)solution_.getAgentGlobalTasks(agent).size() - 1; j >= 0; j--) {
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
    assert(lastValidPosition <= (int)solution_.getAgentGlobalTasks(agent).size() + 1);

    for (int j = firstValidPosition; j < lastValidPosition; j++) {

        // Compute the distance estimate it would take to finish the insertion
        int distance = 0;
        if (j > 0 && j < (int)solution_.getAgentGlobalTasks(agent).size()) {
            distance += instance_.getManhattanDistance(
              instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent, j - 1)),
              instance_.getTaskLocations(task));
            distance += instance_.getManhattanDistance(
              instance_.getTaskLocations(task),
              instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent, j)));
        } else if (j == 0) {
            distance += instance_.getManhattanDistance(
              solution_.agents[agent].pathPlanner->startLocation, instance_.getTaskLocations(task));
        } else {
            distance += instance_.getManhattanDistance(
              instance_.getTaskLocations(solution_.getAgentGlobalTasks(agent, j - 1)),
              instance_.getTaskLocations(task));
        }

        // If the computed distance estimated is longer than the original path size then why bother
        if (distance > solution_.neighbor.conflictedTasksPathSize[task]) {
            continue;
        }

        vector<Path> taskPaths = solution_.paths;
        vector<vector<int>> taskAssignments = solution_.taskAssignments;
        vector<pair<int, int>> precConstraints = *precedenceConstraints;
        Utility utility =
          insertTask(task, agent, j, &taskPaths, &taskAssignments, &precConstraints);
        serviceTimes->push(utility);
    }
}

Utility LNS::insertTask(int task,
                int agent,
                int taskPosition,
                vector<Path>* taskPaths,
                vector<vector<int>>* taskAssignments,
                vector<pair<int, int>>* precedenceConstraints,
                bool commit)
{

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

        taskPathsRef[nextTask] = Path(); // This path will change

        taskAssignmentsRef[agent].insert(taskAssignmentsRef[agent].begin() + taskPosition,
                                           task);
        precConstraintsRef.erase(std::remove_if(precConstraintsRef.begin(),
                                                  precConstraintsRef.end(),
                                                  [previousTask, nextTask](pair<int, int> x) {
                                                      return x.first == previousTask &&
                                                             x.second == nextTask;
                                                  }),
                                   precConstraintsRef.end());
        precConstraintsRef.emplace_back(previousTask, task);
        precConstraintsRef.emplace_back(task, nextTask);

        if (commit) {
            solution_.agents[agent].taskPaths[taskPosition] = Path();
            solution_.agents[agent].taskPaths.insert(
              solution_.agents[agent].taskPaths.begin() + taskPosition, Path());
        }

    } else if (taskPosition == 0) { // In this case we are inserting at the very start

        nextTask = taskAssignmentsRef[agent][taskPosition];
        pathSizeChange = (double)taskPathsRef[nextTask].size();

        taskPathsRef[nextTask] = Path();

        taskAssignmentsRef[agent].insert(taskAssignmentsRef[agent].begin() + taskPosition,
                                           task);
        precConstraintsRef.emplace_back(task, nextTask);

        if (commit) {
            solution_.agents[agent].taskPaths[taskPosition] = Path();
            solution_.agents[agent].taskPaths.insert(
              solution_.agents[agent].taskPaths.begin() + taskPosition, Path());
        }

    } else if (taskPosition == agentTasksSize) { // In this case we are inserting at the very end

        previousTask = taskAssignmentsRef[agent][taskPosition - 1];
        startTime = taskPathsRef[previousTask].endTime();

        taskAssignmentsRef[agent].push_back(task);

        precConstraintsRef.emplace_back(previousTask, task);

        if (commit) {
            solution_.agents[agent].taskPaths.emplace_back();
        }
    }

    vector<int> goalLocations = instance_.getTaskLocations(taskAssignmentsRef[agent]);
    ConstraintTable constraintTable(instance_.numOfCols, instance_.mapSize);
    if (!commit) {
        MultiLabelSpaceTimeAStar localPlanner = MultiLabelSpaceTimeAStar(instance_, agent);
        localPlanner.setGoalLocations(goalLocations);
        localPlanner.computeHeuristics();

        buildConstraintTable(constraintTable,
                               task,
                               instance_.getTaskLocations(task),
                               taskPaths,
                               taskAssignments,
                               precedenceConstraints);
        taskPathsRef[task] =
          localPlanner.findPathSegment(constraintTable, startTime, taskPosition, 0);

        if (nextTask != -1) {
            startTime = taskPathsRef[task].endTime();

            buildConstraintTable(constraintTable,
                                   nextTask,
                                   instance_.getTaskLocations()[nextTask],
                                   taskPaths,
                                   taskAssignments,
                                   precedenceConstraints);
            taskPathsRef[nextTask] =
              localPlanner.findPathSegment(constraintTable, startTime, taskPosition + 1, 0);
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
            taskPathsRef[nextTask] = solution_.agents[agent].pathPlanner->findPathSegment(
              constraintTable, startTime, taskPosition + 1, 0);
            solution_.agents[agent].taskPaths[taskPosition + 1] = taskPathsRef[nextTask];
        }

        for (int k = taskPosition + 1; k < (int)solution_.getAgentGlobalTasks(agent).size(); k++) {
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

void
LNS::commitBestRegretTask(Regret bestRegret)
{

    PLOGD << "Commiting for task " << bestRegret.task << " to agent " << bestRegret.agent
          << " with regret = " << bestRegret.value << endl;
    insertTask(bestRegret.task,
               bestRegret.agent,
               best_regret.task_position,
               &solution.paths,
               &solution.task_assignments,
               &solution.precedence_constraints,
               true);
    solution.neighbor.conflicted_tasks.erase(best_regret.task);
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table,
                            int task,
                            int task_location,
                            vector<Path>* paths,
                            vector<vector<int>>* task_assignments,
                            vector<pair<int, int>>* precedence_constraints)
{
    constraint_table.goal_location = task_location;

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : (*precedence_constraints))
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
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
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        assert(!(*paths)[id].empty());
        int task_position = -1, task_set_size = -1;
        for (int i = 0; i < instance.getAgentNum(); i++) {
            for (int j = 0; j < (int)(*task_assignments)[i].size(); j++) {
                if ((*task_assignments)[i][j] == id) {
                    task_position = j;
                    task_set_size = (int)(*task_assignments)[i].size();
                    break;
                }
            }
        }
        bool wait_at_goal = task_position == task_set_size - 1;
        constraint_table.addPath((*paths)[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!(*paths)[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, (*paths)[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table, int task)
{
    constraint_table.goal_location = instance.getTaskLocations(task);

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());

    // only care about tasks that are not in conflict right now
    for (pair<int, int> precedence_constraint : solution.precedence_constraints) {
        if (std::find_if(solution.neighbor.conflicted_tasks.begin(),
                         solution.neighbor.conflicted_tasks.end(),
                         [precedence_constraint](int task) {
                             return precedence_constraint.first == task ||
                                    precedence_constraint.second == task;
                         }) != solution.neighbor.conflicted_tasks.end())
            continue;
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);
    }

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
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
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        int agent = solution.getAgentWithTask(id);
        int task_position = solution.getLocalTaskIndex(agent, id);
        bool wait_at_goal = task_position == (int)solution.getAssignedTaskSize(agent) - 1;
        constraint_table.addPath(solution.paths[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!solution.paths[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, solution.paths[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

bool
LNS::validateSolution(set<int>* conflicted_tasks)
{

    bool result = true;

    // Check that the precedence constraints are not violated
    for (pair<int, int> precedence_constraint : solution.precedence_constraints) {

        // only care about tasks that are not in conflict
        if (std::find_if(solution.neighbor.conflicted_tasks.begin(),
                         solution.neighbor.conflicted_tasks.end(),
                         [precedence_constraint](int task) {
                             return precedence_constraint.first == task ||
                                    precedence_constraint.second == task;
                         }) != solution.neighbor.conflicted_tasks.end())
            continue;

        int agent_a = solution.getAgentWithTask(precedence_constraint.first),
            agent_b = solution.getAgentWithTask(precedence_constraint.second);
        int task_position_a = solution.getLocalTaskIndex(agent_a, precedence_constraint.first),
            task_position_b = solution.getLocalTaskIndex(agent_b, precedence_constraint.second);

        if (solution.agents[agent_a].path.timestamps[task_position_a] >=
            solution.agents[agent_b].path.timestamps[task_position_b]) {
            PLOGE << "Temporal conflict between agent " << agent_a << " doing task "
                  << precedence_constraint.first << " and agent " << agent_b << " doing task "
                  << precedence_constraint.second << endl;
            result = false;
            if (conflicted_tasks == nullptr)
                return false;
            else {
                conflicted_tasks->insert(precedence_constraint.first);
                conflicted_tasks->insert(precedence_constraint.second);
            }
        }
    }

    for (int agent_i = 0; agent_i < instance.getAgentNum(); agent_i++)
        for (int agent_j = 0; agent_j < instance.getAgentNum(); agent_j++) {
            if (agent_i == agent_j)
                continue;
            size_t min_path_length =
              solution.agents[agent_i].path.size() < solution.agents[agent_j].path.size()
                ? solution.agents[agent_i].path.size()
                : solution.agents[agent_j].path.size();
            for (int timestep = 0; timestep < (int)min_path_length; timestep++) {
                int location_agent_i = solution.agents[agent_i].path.at(timestep).location;
                int location_agent_j = solution.agents[agent_j].path.at(timestep).location;

                // Check that any two agents are not at the same location at the same timestep
                if (location_agent_i == location_agent_j) {
                    pair<int, int> coord = instance.getCoordinate(location_agent_i);
                    PLOGE << "Agents " << agent_i << " and " << agent_j
                          << " collide with each other at (" << coord.first << ", " << coord.second
                          << ") at timestep " << timestep << endl;
                    result = false;
                    if (conflicted_tasks == nullptr)
                        return false;
                    else {
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_i);
                             task_idx++)
                            if (solution.agents[agent_i].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_i, task_idx));
                                break;
                            }
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_j);
                             task_idx++)
                            if (solution.agents[agent_j].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_j, task_idx));
                                break;
                            }
                    }
                }
                // Check that any two agents are not following the same edge in the opposite
                // direction at the same timestep
                else if (timestep < (int)min_path_length - 1 &&
                         location_agent_i ==
                           solution.agents[agent_j].path.at(timestep + 1).location &&
                         location_agent_j ==
                           solution.agents[agent_i].path.at(timestep + 1).location) {
                    pair<int, int> coord_i = instance.getCoordinate(location_agent_i),
                                   coord_j = instance.getCoordinate(location_agent_j);
                    PLOGE << "Agents " << agent_i << " and " << agent_j
                          << " collide with each other at (" << coord_i.first << ", "
                          << coord_i.second << ") --> (" << coord_j.first << ", " << coord_j.second
                          << ") at timestep " << timestep << endl;
                    result = false;
                    if (conflicted_tasks == nullptr)
                        return false;
                    else {
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_i);
                             task_idx++)
                            if (solution.agents[agent_i].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_i, task_idx));
                                break;
                            }
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_j);
                             task_idx++)
                            if (solution.agents[agent_j].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_j, task_idx));
                                break;
                            }
                    }
                }
            }

            // Check that any two agents are not at the same location at the same timestep where
            // one agent might be waiting already
            if (solution.agents[agent_i].path.size() != solution.agents[agent_j].path.size()) {
                int smaller_path_agent =
                  solution.agents[agent_i].path.size() < solution.agents[agent_j].path.size()
                    ? agent_i
                    : agent_j;
                int larger_path_agent =
                  solution.agents[agent_i].path.size() < solution.agents[agent_j].path.size()
                    ? agent_j
                    : agent_i;
                int last_location_of_smaller_path_agent =
                  solution.agents[smaller_path_agent].path.back().location;
                for (int timestep = (int)min_path_length;
                     timestep < (int)solution.agents[larger_path_agent].path.size();
                     timestep++) {
                    int location_of_larger_path_agent =
                      solution.agents[larger_path_agent].path.at(timestep).location;
                    if (last_location_of_smaller_path_agent == location_of_larger_path_agent) {
                        pair<int, int> coord =
                          instance.getCoordinate(last_location_of_smaller_path_agent);
                        PLOGE << "Agents " << agent_i << " and " << agent_j
                              << " collide with each other at (" << coord.first << ", "
                              << coord.second << ") at timestep " << timestep << endl;
                        result = false;
                        if (conflicted_tasks == nullptr)
                            return false;
                        else {
                            if (solution.agents[agent_i]
                                  .path.timestamps[solution.getAssignedTaskSize(agent_i) - 1] <
                                timestep) {
                                conflicted_tasks->insert(solution.getAgentGlobalTasks(
                                  agent_i, solution.getAssignedTaskSize(agent_i) - 1));
                            } else {
                                for (int task_idx = 0;
                                     task_idx < solution.getAssignedTaskSize(agent_i);
                                     task_idx++)
                                    if (solution.agents[agent_i].path.timestamps[task_idx] >
                                        timestep) {
                                        conflicted_tasks->insert(
                                          solution.getAgentGlobalTasks(agent_i, task_idx));
                                        break;
                                    }
                            }
                            if (solution.agents[agent_j]
                                  .path.timestamps[solution.getAssignedTaskSize(agent_j) - 1] <
                                timestep) {
                                conflicted_tasks->insert(solution.getAgentGlobalTasks(
                                  agent_j, solution.getAssignedTaskSize(agent_j) - 1));
                            } else {
                                for (int task_idx = 0;
                                     task_idx < solution.getAssignedTaskSize(agent_j);
                                     task_idx++)
                                    if (solution.agents[agent_j].path.timestamps[task_idx] >
                                        timestep) {
                                        conflicted_tasks->insert(
                                          solution.getAgentGlobalTasks(agent_j, task_idx));
                                        break;
                                    }
                            }
                        }
                    }
                }
            }
        }
    return result;
}

void
LNS::printPaths() const
{
    for (int i = 0; i < instance.getAgentNum(); i++) {
        cout << "Agent " << i << " (cost = " << solution.agents[i].path.size() - 1 << "): ";
        cout << "\n\tPaths:\n\t";
        for (int t = 0; t < (int)solution.agents[i].path.size(); t++) {
            pair<int, int> coord = instance.getCoordinate(solution.agents[i].path.at(t).location);
            cout << "(" << coord.first << ", " << coord.second << ")@" << t;
            if (solution.agents[i].path.at(t).is_goal)
                cout << "*";
            if (i != (int)solution.agents[i].path.size() - 1)
                cout << " -> ";
        }
        cout << endl;
        cout << "\tTimestamps:\n\t";
        for (int j = 0; j < (int)solution.getAssignedTaskSize(i); j++) {
            pair<int, int> goal_coord =
              instance.getCoordinate(solution.agents[i].path_planner->goal_locations[j]);
            cout << "(" << goal_coord.first << ", " << goal_coord.second << ")@"
                 << solution.agents[i].path.timestamps[j];
            if (j != solution.getAssignedTaskSize(i) - 1)
                cout << " -> ";
        }
        cout << endl;
        cout << "\tTasks:\n\t";
        for (int j = 0; j < solution.getAssignedTaskSize(i); j++) {
            cout << solution.getAgentGlobalTasks(i)[j];
            if (j != solution.getAssignedTaskSize(i) - 1)
                cout << " -> ";
        }
        cout << endl;
    }
}
