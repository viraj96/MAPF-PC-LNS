#pragma once

#include <utility>

#include "common.hpp"

#define NO_AGENT -1

class PathTable {
 public:
  int makespan = 0;
  string solverType;
  // This stores the collision-free paths, the value is the id of the agent
  vector<vector<int>> table;
  // This stores the goal locations of the paths: key is the location, while value is the timestep when the agent reaches the goal
  vector<int> goals;

  void reset() {
    auto mapSize = table.size();
    table.clear();
    table.resize(mapSize);
    goals.assign(mapSize, MAX_COST);
    makespan = 0;
  }

  void insertPath(int agentId, const Path& path, bool waitAtGoal = false,
                  bool firstTask = false, bool commit = false);
  std::variant<bool, pair<int, int>> validatePath(int agentId, const Path& path,
                                                  bool firstTask = false);
  void deletePath(int agentId, const Path& path, bool waitAtGoal = false,
                  bool firstTask = false, bool commit = false);
  void deleteAgent(int agentId, int goalLocation);
  bool constrained(int fromLocation, int toLocation, int toTime) const;

  void getAgents(set<int>& conflictingAgents, int location) const;
  void getAgents(set<int>& conflictingAgents, int neighborSize,
                 int location) const;
  void getConflictingAgents(set<int>& conflictingAgents, int fromLocation,
                            int toLocation, int toTime) const;

  int getHoldingTime(int location, int earliestTimestep) const;
  explicit PathTable(string solverType, int mapSize = 0)
      : solverType(std::move(solverType)),
        table(mapSize),
        goals(mapSize, MAX_COST) {}
};

class PathTableWithCollisions {
 private:
  vector<const Path*> paths_;

 public:
  int makespan = 0;
  // This stores the paths, the value is the id of the agent
  vector<vector<list<int>>> table;
  // This stores the goal locatons of the paths: key is the location, while value is the timestep when the agent reaches the goal
  vector<int> goals;

  void reset() {
    auto mapSize = table.size();
    table.clear();
    table.resize(mapSize);
    goals.assign(mapSize, MAX_COST);
    makespan = 0;
  }

  void insertPath(int agentId, const Path& path);
  void insertPath(int agentId);
  void deletePath(int agentId);

  const Path* getPath(int agentId) const { return paths_[agentId]; }

  // Return # of collisions when the agent is waiting at location starting from time forever
  int getFutureNumOfCollisions(int location, int time) const;
  int getNumOfCollisions(int fromLocation, int toLocation, int toTime) const;
  bool hasCollisions(int fromLocation, int toLocation, int toTime) const;
  bool hasEdgeCollisions(int fromLocation, int toLocation, int toTime) const;
  int getLastCollisionTimestep(int location) const;

  // Return the agent who reaches its target target location before timestep 'latest timestep'
  int getAgentWithTarget(int targetLocation, int latestTimestep) const;

  void clear();
  explicit PathTableWithCollisions(int mapSize = 0, int numOfAgents = 0)
      : paths_(numOfAgents, nullptr),
        table(mapSize),
        goals(mapSize, MAX_COST) {}
};
