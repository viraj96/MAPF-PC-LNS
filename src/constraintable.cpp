#include "constrainttable.hpp"

int ConstraintTable::getHoldingTime() {
  int holdingTime = lengthMin;
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

bool ConstraintTable::constrained(size_t location, int timestep) const {
  assert((int)location >= 0);
  if (location < mapSize) {
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
  return constrained(getEdgeIndex(currentLocation, nextLocation), nextTimestep);
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
