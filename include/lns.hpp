#pragma once

#include <plog/Log.h>
#include <limits>
#include "common.hpp"
#include "constrainttable.hpp"
#include "mlastar.hpp"

struct Agent {
  int id;
  Path path;
  vector<Path> taskPaths;
  vector<int> taskAssignments;
  vector<pair<int, int>> intraPrecedenceConstraints;
  std::shared_ptr<SingleAgentSolver> pathPlanner = nullptr;

  Agent(const Agent&) = default;
  Agent(Agent&&) = delete;
  Agent& operator=(const Agent& other) {
    if (this == &other) {
      return *this;
    }
    this->id = other.id;
    this->path = other.path;
    this->taskPaths = other.taskPaths;
    this->taskAssignments = other.taskAssignments;
    this->intraPrecedenceConstraints = other.intraPrecedenceConstraints;

    // Copy the path planner details
    this->pathPlanner->numExpanded = other.pathPlanner->numExpanded;
    this->pathPlanner->numGenerated = other.pathPlanner->numGenerated;

    this->pathPlanner->heuristic = other.pathPlanner->heuristic;
    this->pathPlanner->goalLocations = other.pathPlanner->goalLocations;
    this->pathPlanner->heuristicLandmarks =
        other.pathPlanner->heuristicLandmarks;
    return *this;
  }
  Agent& operator=(Agent&&) = delete;
  Agent(const Instance& instance, int id) : id(id) {
    pathPlanner = std::make_shared<MultiLabelSpaceTimeAStar>(instance, id);
  }
  ~Agent() = default;

  int getLocalTaskIndex(int globalTask) const {
    assert(std::find(taskAssignments.begin(), taskAssignments.end(),
                     globalTask) != taskAssignments.end());
    for (int i = 0; i < (int)taskAssignments.size(); i++) {
      if (taskAssignments[i] == globalTask) {
        return i;
      }
    }
    assert(false);
  }

  inline void insertPrecedenceConstraint(int taskA, int taskB) {
    assert(std::find(taskAssignments.begin(), taskAssignments.end(), taskA) !=
           taskAssignments.end());
    assert(std::find(taskAssignments.begin(), taskAssignments.end(), taskB) !=
           taskAssignments.end());
    intraPrecedenceConstraints.emplace_back(taskA, taskB);
  }

  // This function inserts a precedence constraint when adding a new task into the agent task queue. This would involve removing the existing precedence constraint between the task before it and after it and then adding two new precedence constraints
  void insertIntraAgentPrecedenceConstraint(int task, int taskPosition) {
    int previousTask = -1, nextTask = -1;
    if (taskPosition != 0) {
      previousTask = taskAssignments[taskPosition - 1];
    }
    if (taskPosition != (int)taskAssignments.size() - 1) {
      nextTask = taskAssignments[taskPosition + 1];
    }
    intraPrecedenceConstraints.erase(
        std::remove_if(intraPrecedenceConstraints.begin(),
                       intraPrecedenceConstraints.end(),
                       [previousTask, nextTask](pair<int, int> x) {
                         return (
                             (x.first == previousTask && x.second == nextTask));
                       }),
        intraPrecedenceConstraints.end());
    if (previousTask != -1) {
      intraPrecedenceConstraints.insert(
          intraPrecedenceConstraints.begin() + taskPosition - 1,
          make_pair(previousTask, task));
    }
    if (nextTask != -1) {
      intraPrecedenceConstraints.insert(
          intraPrecedenceConstraints.begin() + taskPosition,
          make_pair(task, nextTask));
    }
  }

  void clearIntraAgentPrecedenceConstraint(int task) {
    assert(std::find(taskAssignments.begin(), taskAssignments.end(), task) !=
           taskAssignments.end());
    int taskPosition = getLocalTaskIndex(task);
    int previousTask = -1, nextTask = -1;
    if (taskPosition != 0) {
      previousTask = taskAssignments[taskPosition - 1];
    }
    if (taskPosition != (int)taskAssignments.size() - 1) {
      nextTask = taskAssignments[taskPosition + 1];
    }

    intraPrecedenceConstraints.erase(
        std::remove_if(intraPrecedenceConstraints.begin(),
                       intraPrecedenceConstraints.end(),
                       [task, previousTask, nextTask](pair<int, int> x) {
                         return (
                             (x.first == previousTask && x.second == task) ||
                             (x.first == task && x.second == nextTask));
                       }),
        intraPrecedenceConstraints.end());

    if (previousTask != -1 && nextTask != -1) {
      insertPrecedenceConstraint(previousTask, nextTask);
    }
  }
};

// We need min-heap for the utility since we want to quickly access the tasks which take the minimum time to complete
struct Utility {
  int agent, taskPosition;
  int pathLength, agentTasksLen;
  double value;

  Utility() {
    agent = -1;
    taskPosition = -1;
    pathLength = -1;
    agentTasksLen = -1;
    value = std::numeric_limits<double>::max();
  }

  Utility(int agent, int taskPosition, int pathLength, int agentTasksLen,
          double value)
      : agent(agent),
        taskPosition(taskPosition),
        pathLength(pathLength),
        agentTasksLen(agentTasksLen),
        value(value) {}

  struct CompareUtilities {
    bool operator()(const Utility& lhs, const Utility& rhs) const {
      if (lhs.value != rhs.value) {
        return lhs.value > rhs.value;
      }
      // Now that the regret values are same we move to compare the path lengths and prefer the regret with smaller path
      if (lhs.pathLength != rhs.pathLength) {
        return lhs.pathLength > rhs.pathLength;
      }
      // If even the path lengths are same then we will move to using the agent tasks queue lengths
      return lhs.agentTasksLen >= rhs.agentTasksLen;
    }
  };
};

// We need max-heap for the regret since we want to quickly access the task whose regret would be maximum
struct Regret {
  int task, agent, taskPosition;
  int pathLength, agentTasksLen;
  double value;

  Regret(int task, int agent, int taskPosition, int pathLength,
         int agentTasksLen, double value)
      : task(task),
        agent(agent),
        taskPosition(taskPosition),
        pathLength(pathLength),
        agentTasksLen(agentTasksLen),
        value(value) {}

  struct CompareRegrets {
    bool operator()(const Regret& lhs, const Regret& rhs) const {
      if (lhs.value != rhs.value) {
        return lhs.value < rhs.value;
      }
      // Now that the regret values are same we move to compare the path lengths and prefer the regret with smaller path
      if (lhs.pathLength != rhs.pathLength) {
        return lhs.pathLength > rhs.pathLength;
      }
      // If even the path lengths are same then we will move to using the agent tasks queue lengths
      return lhs.agentTasksLen >= rhs.agentTasksLen;
    }
  };
};

struct TaskRegretPacket {
  int task, agent, taskPosition, earliestTimestep;
};

struct Conflicts {
  int task, agent, taskPosition;
  Conflicts(int task, int agent, int taskPosition) {
    this->task = task;
    this->agent = agent;
    this->taskPosition = taskPosition;
  }
  bool operator<(const Conflicts& right) const {
    return this->task < right.task;
  }
};

struct Neighbor {
  int additionalTasksAdded;
  set<int> patchedTasks;
  map<int, bool> commitedTasks;
  map<int, int> removedTasksPathSize;
  set<Conflicts> removedTasks, immutableRemovedTasks;
  pairing_heap<Regret, compare<Regret::CompareRegrets>> regretMaxHeap;
  map<int, pairing_heap<Utility, compare<Utility::CompareUtilities>>>
      serviceTimesHeapMap;
};

struct FeasibleSolution {
 public:
  int sumOfCosts{}, numOfCols{};
  vector<Path> agentPaths;

  inline int getRowCoordinate(int id) const { return id / numOfCols; }
  inline int getColCoordinate(int id) const { return id % numOfCols; }
  inline pair<int, int> getCoordinate(int id) const {
    return make_pair(getRowCoordinate(id), getColCoordinate(id));
  }

  string toString() {
    string result =
        "Feasible Solution\n\tSum Of Costs = " + std::to_string(sumOfCosts) +
        "\n";
    for (int agent = 0; agent < (int)agentPaths.size(); agent++) {
      result += "Agent " + std::to_string(agent) +
                " (cost = " + std::to_string(agentPaths[agent].endTime()) +
                "): \n\tPaths:\n\t";
      for (int t = 0; t < (int)agentPaths[agent].path.size(); t++) {
        pair<int, int> coord =
            getCoordinate(agentPaths[agent].path.at(t).location);
        result += "(" + std::to_string(coord.first) + ", " +
                  std::to_string(coord.second) + ")@" + std::to_string(t);
        if (agentPaths[agent].path.at(t).isGoal) {
          result += "*";
        }
        if (agent != (int)agentPaths[agent].path.size() - 1) {
          result += " -> ";
        }
      }
      result += "\n";
    }
    return result;
  }
};

class Solution {
 public:
  int sumOfCosts{};
  vector<Agent> agents;
  map<int, int> taskAgentMap;  // (key, value) - (global task, agent)
  int numOfAgents, numOfTasks;

  Solution(const Solution&) = default;
  Solution(Solution&&) = delete;
  Solution& operator=(Solution&&) = delete;
  ~Solution() = default;

  Solution(const Instance& instance) {
    numOfTasks = instance.getTasksNum();
    numOfAgents = instance.getAgentNum();
    agents.reserve(numOfAgents);
    for (int i = 0; i < numOfAgents; i++) {
      agents.emplace_back(instance, i);
    }
    for (int i = 0; i < numOfTasks; i++) {
      taskAgentMap.insert(make_pair(i, UNASSIGNED));
    }
  }

  Solution& operator=(const Solution& other);

  int getAgentWithTask(int globalTask) { return taskAgentMap[globalTask]; }

  int getLocalTaskIndex(int agent, int globalTask) const {
    return agents[agent].getLocalTaskIndex(globalTask);
  }

  inline vector<int> getAgentGlobalTasks(int agent) const {
    return agents[agent].taskAssignments;
  }
  inline int getAgentGlobalTasks(int agent, int taskIndex) const {
    return agents[agent].taskAssignments[taskIndex];
  }

  inline void assignTaskToAgent(int agent, int task) {
    taskAgentMap[task] = agent;
    agents[agent].taskAssignments.push_back(task);
  }

  inline void assignTaskToAgent(int agent, int task, int taskPosition) {
    assert(taskPosition >= 0 &&
           taskPosition <= (int)agents[agent].taskAssignments.size());
    agents[agent].taskAssignments.insert(
        agents[agent].taskAssignments.begin() + taskPosition, task);
  }

  // Do we need this function?
  void joinPaths(const vector<int>& agentsToCompute) {
    for (int agent : agentsToCompute) {

      assert(getAgentGlobalTasks(agent).size() ==
             agents[agent].pathPlanner->goalLocations.size());
      assert(getAgentGlobalTasks(agent).size() ==
             agents[agent].taskPaths.size());

      agents[agent].path = Path();
      for (int i = 0; i < (int)getAgentGlobalTasks(agent).size(); i++) {
        if (i == 0) {
          agents[agent].path.path.push_back(agents[agent].taskPaths[i].front());
        }
        assert((int)agents[agent].path.size() - 1 ==
               agents[agent].taskPaths[i].beginTime);
        for (int j = 1; j < (int)agents[agent].taskPaths[i].size(); j++) {
          agents[agent].path.path.push_back(agents[agent].taskPaths[i].at(j));
        }
        agents[agent].path.timeStamps.push_back(agents[agent].path.size() - 1);
      }
    }
  }
};

struct ALNS {

  // Parameter values copied from 'https://d-nb.info/1072464683/34'
  int alnsCounter = 0, alnsCounterThreshold = 100, numDestroyHeuristics = 3,
      recentDestroyHeuristic = -1;
  double r1 = 65, r2 = 45, r3 = 25;
  double delta1 = r1 + r2 + r3, delta2 = r2 + r3, delta3 = r3;
  double reactionFactor = 0.35;
  vector<double> weights, used, success;

  ALNS() {

    // Initialize the vectors
    for (int i = 0; i < numDestroyHeuristics; i++) {
      weights.push_back(1);
      used.push_back(0);
      success.push_back(0);
    }
  }
};

class LNS {
 private:
  int numOfIterations_;

 protected:
  int neighborSize_;
  const Instance& instance_;
  FeasibleSolution incumbentSolution_;
  Solution solution_, previousSolution_;
  double timeLimit_, initialSolutionRuntime_ = 0;
  high_resolution_clock::time_point plannerStartTime_;

 public:
  double runtime = 0, temperature = 100, coolingCoefficient = 0.99975,
         heatingCoefficient = 1.00025, tolerance = 5;
  Neighbor lnsNeighborhood;
  vector<Path> initialPaths;
  string initialSolutionStrategy, destroyHeuristic, acceptanceCriteria;
  list<IterationStats> iterationStats;
  int numOfFailures = 0, sumOfCosts = 0;
  ALNS adaptiveLNS;

  LNS(int numOfIterations, const Instance& instance, int neighborSize,
      double timeLimit, string initialStrategy, string destroyHeuristic,
      string acceptanceCriteria);

  inline Instance getInstance() { return instance_; }

  bool run();

  bool buildGreedySolution();
  bool buildGreedySolutionWithMAPFPC(const string& variant);

  void prepareNextIteration();
  void markResolved(int globalTask);
  // Patches the task paths of an agent such that the begin times and end times match up
  void patchAgentTaskPaths(int agent, int taskPosition);

  void printPaths() const;
  bool validateSolution(set<Conflicts>* conflictedTasks = nullptr);

  void buildConstraintTable(ConstraintTable& constraintTable, int task);

  void buildConstraintTable(ConstraintTable& constraintTable, int task,
                            int taskLocation, vector<Path>* taskPaths,
                            vector<pair<int, int>>* precedenceConstraints);

  int extractOldLocalTaskIndex(int task, vector<int> taskQueue);
  set<int> reachableSet(int source, vector<vector<int>> edgeList);

  void computeRegret(bool firstIteration);
  void computeRegretForTask(int task, bool firstIteration);
  void computeRegretForTaskWithAgent(
      TaskRegretPacket regretPacket, vector<int>* taskAssignments,
      vector<Path>* taskPaths, vector<pair<int, int>>* precedenceConstraints,
      pairing_heap<Utility, compare<Utility::CompareUtilities>>* serviceTimes);

  void commitBestRegretTask(Regret bestRegret);
  void commitAncestorTaskOf(int globalTask,
                            std::optional<pair<bool, int>> committingNextTask);

  Utility insertTask(TaskRegretPacket regretPacket, vector<Path>* taskPaths,
                     vector<int>* taskAssignments,
                     vector<pair<int, int>>* precedenceConstraints);
  void insertBestRegretTask(TaskRegretPacket bestRegretPacket);

  Solution getSolution() { return solution_; }

  bool extractFeasibleSolution();
  FeasibleSolution getFeasibleSolution() { return incumbentSolution_; }

  void randomRemoval(std::optional<set<Conflicts>> potentialNeighborhood);
  void worstRemoval(std::optional<set<Conflicts>> potentialNeighborhood);
  void conflictRemoval(std::optional<set<Conflicts>> potentialNeighborhood);
  void shawRemoval(std::optional<set<Conflicts>> potentialNeighborhood);
  void alnsRemoval(std::optional<set<Conflicts>> potentialNeighborhood);

  bool simulatedAnnealing();
  bool thresholdAcceptance();
  bool oldBachelorsAcceptance();
  bool greatDelugeAlgorithm();

  void printAgents() const {
    for (int i = 0; i < instance_.getAgentNum(); i++) {
      pair<int, int> startLoc =
          instance_.getCoordinate(instance_.getStartLocations()[i]);
      PLOGI << "Agent " << i << " : S = (" << startLoc.first << ", "
            << startLoc.second << ") ;\nGoals : \n";
      for (int j = 0; j < (int)solution_.agents[i].taskAssignments.size();
           j++) {
        pair<int, int> goalLoc =
            instance_.getCoordinate(solution_.agents[i].taskAssignments[j]);
        PLOGI << "\t" << j << " : (" << goalLoc.first << " , " << goalLoc.second
              << ")\n";
      }
    }
  }
};
