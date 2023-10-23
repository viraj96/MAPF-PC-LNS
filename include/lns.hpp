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
  SingleAgentSolver* pathPlanner = nullptr;

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
    pathPlanner = new MultiLabelSpaceTimeAStar(instance, id);
  }
  ~Agent() { delete pathPlanner; }

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
  double value;

  Utility() {
    agent = -1;
    taskPosition = -1;
    value = std::numeric_limits<double>::max();
  }

  Utility(int agent, int taskPosition, double value)
      : agent(agent), taskPosition(taskPosition), value(value) {}

  struct CompareNode {
    bool operator()(const Utility& lhs, const Utility& rhs) const {
      return lhs.value >= rhs.value;
    }
  };
};

// We need max-heap for the regret since we want to quickly access the task whose regret would be maximum
struct Regret {
  int task, agent, taskPosition;
  double value;

  Regret(int task, int agent, int taskPosition, double value)
      : task(task), agent(agent), taskPosition(taskPosition), value(value) {}

  struct CompareNode {
    bool operator()(const Regret& lhs, const Regret& rhs) const {
      return lhs.value <= rhs.value;
    }
  };
};

struct TaskRegretPacket {
  int task, agent, taskPosition, earliestTimestep;
};

struct Neighbor {
  set<int> conflictedTasks, patchedTasks;
  map<int, bool> commitedTasks;
  map<int, int> conflictedTasksPathSize;
  pairing_heap<Regret, compare<Regret::CompareNode>> regretMaxHeap;
  map<int, pairing_heap<Utility, compare<Utility::CompareNode>>>
      serviceTimesHeapMap;
};

class Solution {
 public:
  int sumOfCosts{};
  vector<Agent> agents;
  map<int, int> taskAgentMap;  // (key, value) - (global task, agent)
  int numOfAgents, numOfTasks;

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

  Solution& operator=(const Solution& other) {
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

  void joinPaths(const vector<int>& agentsToCompute) {
    for (int agent : agentsToCompute) {

      assert(getAgentGlobalTasks(agent).size() ==
             agents[agent].pathPlanner->goalLocations.size());
      assert(getAgentGlobalTasks(agent).size() ==
             agents[agent].taskPaths.size());

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

class LNS {
 private:
  int numOfIterations_;

 protected:
  int neighborSize_;
  const Instance& instance_;
  Solution solution_, previousSolution_;
  double timeLimit_, initialSolutionRuntime_ = 0;
  high_resolution_clock::time_point plannerStartTime_;

 public:
  double runtime = 0, saTemperature = 100, saCoolingCoefficient = 0.955;
  Neighbor lnsNeighborhood;
  vector<Path> initialPaths;
  list<IterationStats> iterationStats;
  int numOfFailures = 0, sumOfCosts = 0;

  LNS(int numOfIterations, const Instance& instance, int neighborSize,
      double timeLimit);

  inline Instance getInstance() { return instance_; }

  bool run();
  bool buildGreedySolution();
  void prepareNextIteration();
  void printPaths() const;
  bool validateSolution(set<int>* conflictedTasks = nullptr);
  void buildConstraintTable(ConstraintTable& constraintTable, int task);

  void buildConstraintTable(ConstraintTable& constraintTable, int task,
                            int taskLocation, vector<Path>* taskPaths,
                            vector<pair<int, int>>* precedenceConstraints);

  void computeRegret(bool firstIteration);
  void computeRegretForTask(int task, bool firstIteration);
  void commitBestRegretTask(Regret bestRegret);
  void computeRegretForTaskWithAgent(
      TaskRegretPacket regretPacket, vector<int>* taskAssignments,
      vector<Path>* taskPaths, vector<pair<int, int>>* precedenceConstraints,
      pairing_heap<Utility, compare<Utility::CompareNode>>* serviceTimes);
  Utility insertTask(TaskRegretPacket regretPacket, vector<Path>* taskPaths,
                     vector<int>* taskAssignments,
                     vector<pair<int, int>>* precedenceConstraints,
                     bool commit = false);

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
