#include "reservationtable.hpp"
#include "constrainttable.hpp"

void ReservationTable::insert2SIT(int location, int tMin, int tMax) {

  assert(tMin >= 0 && tMin < tMax and !safeIntervalTable_[location].empty());

  for (auto it = safeIntervalTable_[location].begin();
       it != safeIntervalTable_[location].end();) {
    if (tMin >= get<1>(*it)) {
      ++it;
    } else if (tMax <= get<0>(*it)) {
      break;
    } else if (get<0>(*it) < tMin && get<1>(*it) <= tMax) {
      (*it) = make_tuple(get<0>(*it), tMin, get<2>(*it));
      ++it;
    } else if (tMin <= get<0>(*it) && tMax < get<1>(*it)) {
      (*it) = make_tuple(tMax, get<1>(*it), get<2>(*it));
      break;
    } else if (get<0>(*it) < tMin && tMax < get<1>(*it)) {
      safeIntervalTable_[location].insert(
          it, make_tuple(get<0>(*it), tMin, get<2>(*it)));
      (*it) = make_tuple(tMax, get<1>(*it), get<2>(*it));
      break;
    } else {
      it = safeIntervalTable_[location].erase(it);
    }
  }
}

void ReservationTable::insertSoftConstraint2SIT(int location, int tMin,
                                                int tMax) {
  assert(tMin >= 0 && tMin < tMax and !safeIntervalTable_[location].empty());
  for (auto it = safeIntervalTable_[location].begin();
       it != safeIntervalTable_[location].end(); ++it) {
    if (tMin >= get<1>(*it) || get<2>(*it)) {
      continue;
    }
    if (tMax <= get<0>(*it)) {
      break;
    }

    auto iMin = get<0>(*it);
    auto iMax = get<1>(*it);
    if (iMin < tMin && iMax <= tMax) {
      if (it != safeIntervalTable_[location].end() &&
          std::next(it) != safeIntervalTable_[location].end() &&
          (location != goalLocation_ || iMax != constraintTable.lengthMin) &&
          iMax == get<0>(*std::next(it)) && get<2>(*std::next(it))) {
        // We can merge the current interval with the next one
        (*it) = make_tuple(iMin, tMin, false);
        ++it;
        (*it) = make_tuple(tMin, get<1>(*it), true);
      } else {
        safeIntervalTable_[location].insert(it, make_tuple(iMin, tMin, false));
        (*it) = make_tuple(tMin, iMax, true);
      }

    } else if (tMin <= iMin && tMax < iMax) {
      if (it != safeIntervalTable_[location].begin() &&
          (location != goalLocation_ || iMin != constraintTable.lengthMin) &&
          iMin == get<1>(*std::prev(it)) && get<2>(*std::prev(it))) {
        // We can merge the current interval with the previous one
        (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), tMax, true);
      } else {
        safeIntervalTable_[location].insert(it, make_tuple(iMin, tMax, true));
      }
      (*it) = make_tuple(tMax, iMax, false);
    } else if (iMin < tMin && tMax < iMax) {
      safeIntervalTable_[location].insert(it, make_tuple(iMin, tMin, false));
      safeIntervalTable_[location].insert(it, make_tuple(tMin, tMax, true));
      (*it) = make_tuple(tMax, iMax, false);
    } else {
      if (it != safeIntervalTable_[location].begin() &&
          (location != goalLocation_ || iMin != constraintTable.lengthMin) &&
          iMin == get<1>(*std::prev(it)) && get<2>(*std::prev(it))) {
        // We can merge the current interval with the previous one
        if (it != safeIntervalTable_[location].end() &&
            std::next(it) != safeIntervalTable_[location].end() &&
            (location != goalLocation_ || iMax != constraintTable.lengthMin) &&
            iMax == get<0>(*std::next(it)) && get<2>(*std::next(it))) {
          // We can merge the current interval with the next one
          (*std::prev(it)) =
              make_tuple(get<0>(*std::prev(it)), get<1>(*std::next(it)), true);
          safeIntervalTable_[location].erase(std::next(it));
          it = safeIntervalTable_[location].erase(it);
        } else {
          (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), iMax, true);
          it = safeIntervalTable_[location].erase(it);
        }
        --it;
      } else {
        if (it != safeIntervalTable_[location].end() &&
            std::next(it) != safeIntervalTable_[location].end() &&
            (location != goalLocation_ || iMax != constraintTable.lengthMin) &&
            iMax == get<0>(*std::next(it)) && get<2>(*std::next(it))) {
          // We can merge the current interval with the next one
          (*it) = make_tuple(iMin, get<1>(*std::next(it)), true);
          safeIntervalTable_[location].erase(std::next(it));
        } else {
          (*it) = make_tuple(iMin, iMax, true);
        }
      }
    }
  }
}

// Update SIT at the given location
void ReservationTable::updateSIT(int location) {

  assert(safeIntervalTable_[location].empty());

  // Length constraints for the goal location
  // We need to divide the same intervals into 2 parts [0, length_min) and [length_min, length_max + 1)
  if (location == goalLocation_) {
    if (constraintTable.lengthMin > constraintTable.lengthMax) {
      // The location is blocked for the entire time horizon
      safeIntervalTable_[location].emplace_back(0, 0, false);
      return;
    }
    if (0 < constraintTable.lengthMin) {
      safeIntervalTable_[location].emplace_back(0, constraintTable.lengthMin,
                                                false);
    }
    assert(constraintTable.lengthMin >= 0);
    safeIntervalTable_[location].emplace_back(
        constraintTable.lengthMin,
        min(constraintTable.lengthMax + 1, MAX_TIMESTEP), false);
  } else {
    safeIntervalTable_[location].emplace_back(
        0, min(constraintTable.lengthMax, MAX_TIMESTEP - 1) + 1, false);
  }

  // Path table
  if (constraintTable.pathTableForConstraints != nullptr and
      !constraintTable.pathTableForConstraints->table.empty()) {
    if (location < (int)constraintTable.mapSize) {
      // Vertex conflict
      for (int t = 0;
           t <
           (int)constraintTable.pathTableForConstraints->table[location].size();
           t++) {
        if (constraintTable.pathTableForConstraints->table[location][t] !=
            NO_AGENT) {
          insert2SIT(location, t, t + 1);
        }
      }
      if (constraintTable.pathTableForConstraints->goals[location] <
          MAX_TIMESTEP) {
        // Target conflict
        insert2SIT(location,
                   constraintTable.pathTableForConstraints->goals[location],
                   MAX_TIMESTEP + 1);
      }
    } else {
      // Edge conflict
      int from = location / constraintTable.mapSize - 1;
      int to = location % constraintTable.mapSize;
      if (from != to) {
        int tMax = (int)min(
            constraintTable.pathTableForConstraints->table[from].size(),
            constraintTable.pathTableForConstraints->table[to].size() + 1);
        for (int t = 1; t < tMax; t++) {
          if (constraintTable.pathTableForConstraints->table[to][t - 1] !=
                  NO_AGENT &&
              constraintTable.pathTableForConstraints->table[to][t - 1] ==
                  constraintTable.pathTableForConstraints->table[from][t]) {
            insert2SIT(location, t, t + 1);
          }
        }
      }
    }
  }

  // Negative constraints
  const auto& it = constraintTable.constraintTable_.find(location);
  if (it != constraintTable.constraintTable_.end()) {
    for (auto timeRange : it->second) {
      insert2SIT(location, timeRange.first, timeRange.second);
    }
  }

  // Positive constraints
  if (location < (int)constraintTable.mapSize) {
    for (auto landmark : constraintTable.landmarks_) {
      if ((int)landmark.second != location) {
        insert2SIT(location, landmark.first, landmark.first + 1);
      }
    }
  }

  // Soft path table
  if (constraintTable.pathTableForConflictAvoidance != nullptr and
      !constraintTable.pathTableForConflictAvoidance->table.empty()) {
    if (location < (int)constraintTable.mapSize) {
      // Vertex conflict
      for (int t = 0; t < (int)constraintTable.pathTableForConflictAvoidance
                              ->table[location]
                              .size();
           t++) {
        if (!constraintTable.pathTableForConflictAvoidance->table[location][t]
                 .empty()) {
          insertSoftConstraint2SIT(location, t, t + 1);
        }
      }
      if (constraintTable.pathTableForConflictAvoidance->goals[location] <
          MAX_TIMESTEP) {
        // Target conflict
        insertSoftConstraint2SIT(
            location,
            constraintTable.pathTableForConflictAvoidance->goals[location],
            MAX_TIMESTEP + 1);
      }
    } else {
      // Edge conflict
      auto from = location / constraintTable.mapSize - 1;
      auto to = location % constraintTable.mapSize;
      if (from != to) {
        int tMax = (int)min(
            constraintTable.pathTableForConflictAvoidance->table[from].size(),
            constraintTable.pathTableForConflictAvoidance->table[to].size() +
                1);
        for (int t = 1; t < tMax; t++) {
          bool found = false;
          for (auto a1 : constraintTable.pathTableForConflictAvoidance
                             ->table[to][t - 1]) {
            for (auto a2 : constraintTable.pathTableForConflictAvoidance
                               ->table[from][t]) {
              if (a1 == a2) {
                insertSoftConstraint2SIT(location, t, t + 1);
                found = true;
                break;
              }
              if (found) {
                break;
              }
            }
          }
        }
      }
    }
  }

  // Soft constraints
  if (!constraintTable.conflictAvoidanceTable_.empty()) {
    for (int t = 0;
         t < (int)constraintTable.conflictAvoidanceTable_[location].size();
         t++) {
      if (constraintTable.conflictAvoidanceTable_[location][t]) {
        insertSoftConstraint2SIT(location, t, t + 1);
      }
    }
    if (constraintTable.conflictAvoidanceTableGoals_[location] < MAX_TIMESTEP) {
      insertSoftConstraint2SIT(
          location, constraintTable.conflictAvoidanceTableGoals_[location],
          MAX_TIMESTEP + 1);
    }
  }
}

// Return <upper bound, low, high, vertex collision, edge collision>
list<tuple<int, int, int, bool, bool>> ReservationTable::getSafeIntervals(
    int fromLocation, int toLocation, int lowerBound, int upperBound) {

  list<tuple<int, int, int, bool, bool>> result;
  if (lowerBound >= upperBound) {
    return result;
  }

  if (safeIntervalTable_[toLocation].empty()) {
    updateSIT(toLocation);
  }

  for (auto interval : safeIntervalTable_[toLocation]) {

    if (lowerBound >= get<1>(interval)) {
      continue;
    }

    if (upperBound <= get<0>(interval)) {
      break;
    }

    // The interval overlaps with [lower_bound, upper_bound)
    auto t1 = getEarliestArrivalTime(fromLocation, toLocation,
                                     max(lowerBound, get<0>(interval)),
                                     min(upperBound, get<1>(interval)));

    if (t1 < 0) {
      // The interval is not reachable
      continue;
    }
    if (get<2>(interval)) {
      // The interval has collisions
      result.emplace_back(get<1>(interval), t1, get<1>(interval), true, false);
    } else {
      // The interval does not have collisions so we need to check the move action has collisions or not
      auto t2 = getEarliestNoCollisionArrivalTime(fromLocation, toLocation,
                                                  interval, t1, upperBound);
      if (t1 == t2) {
        result.emplace_back(get<1>(interval), t1, get<1>(interval), false,
                            false);
      } else if (t2 < 0) {
        result.emplace_back(get<1>(interval), t1, get<1>(interval), false,
                            true);
      } else {
        result.emplace_back(get<1>(interval), t1, t2, false, true);
        result.emplace_back(get<1>(interval), t2, get<1>(interval), false,
                            false);
      }
    }
  }
  return result;
}

Interval ReservationTable::getFirstSafeInterval(size_t location,
                                                int startTime) {
  if (safeIntervalTable_[location].empty()) {
    updateSIT(location);
  }
  for (auto interval : safeIntervalTable_[location]) {
    // Find the interval where we can actually start the task
    // TODO: Should we use start time + 1 for any case or make a case specific for same aget vs different agent
    if (startTime + 1 >= get<0>(interval) && startTime + 1 < get<1>(interval)) {
      return interval;
    }
  }
  return {};
}

// Find a safe interval with tMin as given
bool ReservationTable::findSafeInterval(Interval& interval, size_t location,
                                        int tMin) {

  if (tMin >= min(constraintTable.lengthMax, MAX_TIMESTEP - 1) + 1) {
    return false;
  }

  if (safeIntervalTable_[location].empty()) {
    updateSIT(location);
  }

  for (auto& i : safeIntervalTable_[location]) {

    if ((int)get<0>(i) <= tMin && tMin < (int)get<1>(i)) {
      interval = Interval(tMin, get<1>(i), get<2>(i));
      return true;
    }

    if (tMin < (int)get<0>(i)) {
      break;
    }
  }
  return false;
}

int ReservationTable::getEarliestArrivalTime(int fromLocation, int toLocation,
                                             int lowerBound,
                                             int upperBound) const {
  for (int t = lowerBound; t < upperBound; t++) {
    if (!constraintTable.constrained(fromLocation, toLocation, t)) {
      return t;
    }
  }
  return -1;
}
int ReservationTable::getEarliestNoCollisionArrivalTime(
    int fromLocation, int toLocation, const Interval& interval, int lowerBound,
    int upperBound) const {
  for (int t = max(lowerBound, get<0>(interval));
       t < min(upperBound, get<1>(interval)); t++) {
    if (!constraintTable.hasEdgeConflict(fromLocation, toLocation, t)) {
      return t;
    }
  }
  return -1;
}
