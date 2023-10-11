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
  SingleAgentSolver* pathPlanner = nullptr;

  Agent(const Instance& instance, int id) : id(id) {
    pathPlanner = new MultiLabelSpaceTimeAStar(instance, id);
  }
  ~Agent() { delete pathPlanner; }
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

struct Neighbor {
  set<int> conflictedTasks;
  map<int, int> conflictedTasksPathSize;
  pairing_heap<Regret, compare<Regret::CompareNode>> regretMaxHeap;
};

class Solution {
 public:
  int sumOfCosts{};
  Neighbor neighbor;
  vector<Path> paths;
  vector<Agent> agents;
  int numOfAgents, numOfTasks;
  vector<vector<int>> taskAssignments;
  vector<pair<int, int>> precedenceConstraints;

  Solution(const Instance& instance) {
    numOfTasks = instance.getTasksNum();
    numOfAgents = instance.getAgentNum();
    taskAssignments.resize(numOfAgents);

    agents.reserve(numOfAgents);
    for (int i = 0; i < numOfAgents; i++) {
      agents.emplace_back(instance, i);
    }
  }

  int getAgentWithTask(int globalTask) const {
    for (int i = 0; i < numOfAgents; i++) {
      for (int j = 0; j < (int)taskAssignments[i].size(); j++) {
        if (taskAssignments[i][j] == globalTask) {
          return i;
        }
      }
    }
    assert(false);
  }

  int getLocalTaskIndex(int agent, int globalTask) const {
    for (int i = 0; i < (int)taskAssignments[agent].size(); i++) {
      if (taskAssignments[agent][i] == globalTask) {
        return i;
      }
    }
    assert(false);
  }

  inline vector<int> getAgentGlobalTasks(int agent) const {
    return taskAssignments[agent];
  }
  inline int getAgentGlobalTasks(int agent, int taskIndex) const {
    return taskAssignments[agent][taskIndex];
  }

  inline void assignTaskToAgent(int agent, int task) {
    taskAssignments[agent].push_back(task);
  }

  inline void insertPrecedenceConstraint(int taskA, int taskB) {
    precedenceConstraints.emplace_back(taskA, taskB);
  }

  void clearIntraAgentPrecedenceConstraint(int task) {
    int agent = getAgentWithTask(task),
        taskPosition = getLocalTaskIndex(agent, task);
    int previousTask = -1, nextTask = -1;
    if (taskPosition != 0) {
      previousTask = taskAssignments[agent][taskPosition - 1];
    }
    if (taskPosition != (int)taskAssignments[agent].size() - 1) {
      nextTask = taskAssignments[agent][taskPosition + 1];
    }

    precedenceConstraints.erase(
        std::remove_if(
            precedenceConstraints.begin(), precedenceConstraints.end(),
            [task, previousTask, nextTask](pair<int, int> x) {
              return ((x.first == previousTask && x.second == task) ||
                      (x.first == task && x.second == nextTask));
            }),
        precedenceConstraints.end());

    if (previousTask != -1 && nextTask != -1) {
      insertPrecedenceConstraint(previousTask, nextTask);
    }
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
  double runtime = 0;
  vector<Path> initialPaths;
  list<IterationStats> iterationStats;
  int numOfFailures = 0, sumOfCosts = 0;

  LNS(int numOfIterations, const Instance& instance, int neighborSize,
      double timeLimit);

  inline Instance getInstance() { return instance_; }

  bool run();
  bool buildGreedySolution();
  void prepareNextIteration();
  unordered_set<int> prepareNextIterationv2();
  void printPaths() const;
  bool validateSolution(set<int>* conflictedTasks = nullptr);
  void buildConstraintTable(ConstraintTable& constraintTable, int task);

  void buildConstraintTable(ConstraintTable& constraintTable, int task,
                            int taskLocation, vector<Path>* paths,
                            vector<vector<int>>* taskAssignments,
                            vector<pair<int, int>>* precedenceConstraints);

  void computeRegret();
  void regretBasedReinsertion();
  void computeRegretForMetaTask(deque<int> metaTask);
  void computeRegretForTask(int task);
  void commitBestRegretTask(Regret bestRegret);
  void computeRegretForTaskWithAgent(
      int task, int agent, int earliestTimestep, int latestTimestep,
      vector<pair<int, int>>* precedenceConstraints,
      pairing_heap<Utility, compare<Utility::CompareNode>>* serviceTimes);
  Utility insertTask(int task, int agent, int taskPosition,
                     vector<Path>* taskPaths,
                     vector<vector<int>>* taskAssignments,
                     vector<pair<int, int>>* precedenceConstraints,
                     bool commit = false);

  void printAgents() const {
    for (int i = 0; i < instance_.getAgentNum(); i++) {
      pair<int, int> startLoc =
          instance_.getCoordinate(instance_.getStartLocations()[i]);
      PLOGI << "Agent " << i << " : S = (" << startLoc.first << ", "
            << startLoc.second << ") ;\nGoals : \n";
      for (int j = 0; j < (int)solution_.taskAssignments[i].size(); j++) {
        pair<int, int> goalLoc =
            instance_.getCoordinate(solution_.taskAssignments[i][j]);
        PLOGI << "\t" << j << " : (" << goalLoc.first << " , " << goalLoc.second
              << ")\n";
      }
    }
  }
};
