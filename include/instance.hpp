#pragma once

#include <plog/Log.h>
#include "common.hpp"

class Instance {

 protected:
  vector<bool> map_;
  string mapFname_;
  string agentTaskFname_;

  int numOfAgents_{}, numOfTasks_{};
  vector<int> taskLocations_, startLocations_;
  // Maps given task to all its predecessors as given in the input
  unordered_map<int, vector<int>> taskDependencies_;

  bool loadMap();
  bool loadAgentsAndTasks();
  void saveMap() const;
  void printMap() const;
  void saveAgents() const;

  bool isConnected(int start, int goal);
  friend class Solution;
  friend class SingleAgentSolver;

 public:
  int mapSize{}, numOfCols{}, numOfRows{};

  Instance() = default;
  Instance(const string& mapFname, const string& agentTaskFname,
           int numOfAgents = 0, int numOfTasks = 0);

  inline int getTaskLocations(int task) const { return taskLocations_[task]; }
  vector<int> getTaskLocations(vector<int> tasks) const {
    vector<int> taskLocs(tasks.size(), 0);
    for (int i = 0; i < (int)tasks.size(); i++) {
      taskLocs[i] = taskLocations_[tasks[i]];
    }
    return taskLocs;
  }
  list<int> getNeighbors(int current) const;
  inline bool isObstacle(int loc) const { return map_[loc]; }
  inline bool validMove(int curr, int next) const {
    if (next < 0 || next >= mapSize || map_[next]) {
      return false;
    }
    return getManhattanDistance(curr, next) < 2;
  };

  inline int linearizeCoordinate(int row, int col) const {
    return (this->numOfCols * row + col);
  }
  inline int getRowCoordinate(int id) const { return id / this->numOfCols; }
  inline int getColCoordinate(int id) const { return id % this->numOfCols; }
  inline pair<int, int> getCoordinate(int id) const {
    return make_pair(getRowCoordinate(id), getColCoordinate(id));
  }
  inline int getCols() const { return numOfCols; }
  inline int getAgentNum() const { return numOfAgents_; }
  inline int getTasksNum() const { return numOfTasks_; }
  inline vector<int> getTaskLocations() const { return taskLocations_; }
  inline vector<int> getStartLocations() const { return startLocations_; }
  inline unordered_map<int, vector<int>> getTaskDependencies() const {
    return taskDependencies_;
  }
  inline int getManhattanDistance(int loc1, int loc2) const {
    int loc1X = getRowCoordinate(loc1);
    int loc1Y = getColCoordinate(loc1);
    int loc2X = getRowCoordinate(loc2);
    int loc2Y = getColCoordinate(loc2);

    return abs(loc1X - loc2X) + abs(loc1Y - loc2Y);
  }
  inline int getManhattanDistance(const pair<int, int>& loc1,
                                  const pair<int, int>& loc2) const {
    return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
  }
  int getDefaultNumberOfTasks() const { return numOfTasks_; }
  int getDefaultNumberOfAgents() const { return numOfAgents_; }
};
