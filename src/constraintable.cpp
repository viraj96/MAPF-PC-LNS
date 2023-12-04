#include "constrainttable.hpp"

void ConstraintTable::copy(const ConstraintTable& other) {

  lengthMin = other.lengthMin;
  lengthMax = other.lengthMax;

  numCol = other.numCol;
  mapSize = other.mapSize;

  constraintTable_ = other.constraintTable_;
  constraintTableMaxTimeStep_ = other.constraintTableMaxTimeStep_;

  conflictAvoidanceTable_ = other.conflictAvoidanceTable_;
  conflictAvoidanceTableGoals_ = other.conflictAvoidanceTableGoals_;
  conflictAvoidanceTableMaxTimeStep_ = other.conflictAvoidanceTableMaxTimeStep_;

  landmarks_ = other.landmarks_;

  pathTableForConstraints = other.pathTableForConstraints;
  pathTableForConflictAvoidance = other.pathTableForConflictAvoidance;
}

ConstraintTable::ConstraintTable(const ConstraintTable& other) {
  copy(other);
}

int ConstraintTable::getHoldingTime() {
  int holdingTime = lengthMin;
  if (pathTableForConstraints != nullptr) {
    holdingTime =
        pathTableForConstraints->getHoldingTime(goalLocation, lengthMin);
  }
  auto it = constraintTable_.find(goalLocation);
  if (it != constraintTable_.end()) {
    for (pair<int, int> timeRange : it->second) {
      holdingTime = max(holdingTime, timeRange.second);
    }
  }
  for (pair<size_t, size_t> landmark : landmarks_) {
    if ((int)landmark.second != goalLocation) {
      holdingTime = max(holdingTime, (int)landmark.first + 1);
    }
  }
  return holdingTime;
}

int ConstraintTable::getLastCollisionTimestep(int location) const {
  int result = -1;
  if (pathTableForConflictAvoidance != nullptr) {
    result = pathTableForConflictAvoidance->getLastCollisionTimestep(location);
  }
  if (!conflictAvoidanceTable_.empty()) {
    for (int t = (int)conflictAvoidanceTable_[location].size(); t > result;
         t--) {
      if (conflictAvoidanceTable_[location][t]) {
        return t;
      }
    }
  }
  return result;
}

int ConstraintTable::getFutureNumOfCollisions(int location, int time) const {
  int result = 0;
  if (pathTableForConflictAvoidance != nullptr) {
    result =
        pathTableForConflictAvoidance->getFutureNumOfCollisions(location, time);
  }
  if (!conflictAvoidanceTable_.empty()) {
    for (int timestep = time + 1;
         timestep < (int)conflictAvoidanceTable_[location].size(); timestep++) {
      result += (int)conflictAvoidanceTable_[location][timestep];
    }
  }
  return result;
}

bool ConstraintTable::hasEdgeConflict(size_t currentId, size_t nextId,
                                      int nextTimestep) const {
  assert(currentId != nextId);
  if (pathTableForConflictAvoidance != nullptr &&
      pathTableForConflictAvoidance->hasEdgeCollisions(currentId, nextId,
                                                       nextTimestep)) {
    return true;
  }
  return !conflictAvoidanceTable_.empty() && currentId != nextId &&
         (int)conflictAvoidanceTable_[nextId].size() >= nextTimestep &&
         (int)conflictAvoidanceTable_[currentId].size() > nextTimestep &&
         conflictAvoidanceTable_[nextId][nextTimestep] &&
         conflictAvoidanceTable_[currentId][nextTimestep];
}

bool ConstraintTable::constrained(size_t location, int timestep) const {
  assert((int)location >= 0);
  if (location < mapSize) {

    if (pathTableForConstraints != nullptr &&
        pathTableForConstraints->constrained(location, location, timestep)) {
      return true;
    }

    const auto& it = landmarks_.find(timestep);
    if (it != landmarks_.end() && it->second != location) {
      return true;  // Violate the positive vertex constraint
    }
  }

  const auto& it = constraintTable_.find(location);
  if (it == constraintTable_.end()) {
    return false;
  }
  for (const auto& constraint : it->second) {
    if (constraint.first <= timestep && timestep < constraint.second) {
      return true;
    }
  }
  return false;
}

bool ConstraintTable::constrained(size_t currentLocation, size_t nextLocation,
                                  int nextTimestep) const {
  return (pathTableForConstraints != nullptr &&
          pathTableForConstraints->constrained(currentLocation, nextLocation,
                                               nextTimestep)) ||
         constrained(getEdgeIndex(currentLocation, nextLocation), nextTimestep);
}

void ConstraintTable::insert2CT(size_t location, int tMin, int tMax) {
  assert((int)location >= 0);
  constraintTable_[location].emplace_back(tMin, tMax);
  if (tMax < MAX_TIMESTEP && tMax > latestTimestep) {
    latestTimestep = tMax;
  } else if (tMax == MAX_TIMESTEP && tMin > latestTimestep) {
    latestTimestep = tMin;
  }
}

void ConstraintTable::insert2CT(size_t from, size_t to, int tMin, int tMax) {
  insert2CT(getEdgeIndex(from, to), tMin, tMax);
}

void ConstraintTable::addPath(const Path& path, bool waitAtGoal) {
  int offset = path.beginTime;
  for (int i = 0; i < (int)path.size() - 1; i++) {
    int timestep = i + offset;
    insert2CT(path[i].location, timestep, timestep + 1);
    insert2CT(path[i + 1].location, path[i].location, timestep + 1,
              timestep + 2);
  }
  int last = (int)path.size() - 1;
  int timestep = last + offset;
  if (waitAtGoal) {
    insert2CT(path[last].location, timestep, MAX_TIMESTEP);
  } else {
    insert2CT(path[last].location, timestep, timestep + 1);
  }
}
