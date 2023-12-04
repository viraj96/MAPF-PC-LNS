#pragma once

#include <plog/Log.h>
#include "common.hpp"
#include "pathtable.hpp"

class ConstraintTable {

 protected:
  friend class ReservationTable;

  int constraintTableMaxTimeStep_ = 0, conflictAvoidanceTableMaxTimeStep_ = 0;
  unordered_map<size_t, size_t> landmarks_;  // (key, value) - (time, location)
  unordered_map<size_t, list<pair<int, int>>>
      constraintTable_;  // (key, value) - (location, occupied time intervals)
  vector<vector<bool>> conflictAvoidanceTable_;
  vector<int> conflictAvoidanceTableGoals_;

  void insertLandmark(size_t location, int timestep);
  list<pair<int, int>> decodeBarrier(int barrier1, int barrier2, int t) const;
  inline size_t getEdgeIndex(size_t from, size_t to) const {
    return (1 + from) * mapSize + to;
  }

 public:
  size_t numCol{}, mapSize{};
  int lengthMin = 0, lengthMax = MAX_TIMESTEP, goalLocation = -1;
  // Latest recorded timestep in the occupied interval table. Cannot be the
  // MAX_TIMESTEP
  int latestTimestep = 0;

  const PathTable* pathTableForConstraints = nullptr;
  const PathTableWithCollisions* pathTableForConflictAvoidance = nullptr;

  ConstraintTable() = default;
  ConstraintTable(
      size_t numCol, size_t mapSize,
      const PathTable* pathTableForConstraints = nullptr,
      const PathTableWithCollisions* pathTableForConflictAvoidance = nullptr)
      : numCol(numCol),
        mapSize(mapSize),
        pathTableForConstraints(pathTableForConstraints),
        pathTableForConflictAvoidance(pathTableForConflictAvoidance) {}
  ConstraintTable(const ConstraintTable& other);

  int getHoldingTime();
  int getLastCollisionTimestep(int location) const;
  int getFutureNumOfCollisions(int location, int time) const;
  bool hasEdgeConflict(size_t currentId, size_t nextId, int nextTimestep) const;

  bool constrained(size_t location, int timestep) const;
  bool constrained(size_t currentLocation, size_t nextLocation,
                   int nextTimestep) const;

  void copy(const ConstraintTable& other);
  void insert2CT(size_t location, int tMin, int tMax);
  void insert2CT(size_t from, size_t to, int tMin, int tMax);

  void addPath(const Path& path, bool waitAtGoal);
  unordered_map<size_t, size_t> getLandmarks() const { return landmarks_; }
};
