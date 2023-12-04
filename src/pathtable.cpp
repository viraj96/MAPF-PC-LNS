#include "pathtable.hpp"
#include "common.hpp"

using Changes =
    std::tuple<int, int, int, int>;  // location, timestep, agent from, agent to

void PathTable::insertPath(int agentId, const Path& path, bool waitAtGoal,
                           bool firstTask) {

  if (path.empty() || solverType != "sipps") {
    return;
  }

  int offset = path.beginTime;
  for (int t = 0; t < (int)path.size(); t++) {
    int timestep = t + offset, location = path[t].location;
    if ((int)table[location].size() <= timestep) {
      table[location].resize(timestep + 1, NO_AGENT);
    }

    if (!firstTask && t == 0) {
      assert(table[location][timestep] == agentId);
    } else {
      assert(table[location][timestep] == NO_AGENT);
    }
    table[location][timestep] = agentId;
  }

  if (waitAtGoal) {
    assert(goals[path.back().location] == MAX_TIMESTEP);
    goals[path.back().location] = (int)path.endTime();
  }
  makespan = max(makespan, (int)path.endTime());
}

std::variant<bool, pair<int, int>> PathTable::validatePath(int agentId,
                                                           const Path& path,
                                                           bool firstTask) {

  if (path.empty() || solverType != "sipps") {
    return true;
  }

  vector<Changes> updates;
  pair<int, int> offendingResult;

  bool success = true;
  int offset = path.beginTime;
  for (int t = 0; t < (int)path.size(); t++) {
    int timestep = t + offset, location = path[t].location;
    if ((int)table[location].size() <= timestep) {
      table[location].resize(timestep + 1, NO_AGENT);
    }

    if (!firstTask && t == 0) {
      if (table[location][timestep] != agentId) {
        success = false;
        assert(table[location][timestep] != NO_AGENT);
        offendingResult = make_pair(table[location][timestep], timestep);
        break;
      }
    } else {
      if (table[location][timestep] != NO_AGENT) {
        success = false;
        offendingResult = make_pair(table[location][timestep], timestep);
        break;
      }
    }
    updates.emplace_back(location, timestep, table[location][timestep],
                         agentId);
    table[location][timestep] = agentId;
  }

  if (!success) {
    // Roll-back the changes made during this iteration
    for (auto& update : updates) {
      int location, timestep, agentFrom, agentTo;
      tie(location, timestep, agentFrom, agentTo) = update;
      table[location][timestep] = agentFrom;
    }
    return offendingResult;
  }
  return true;
}

void PathTable::deletePath(int agentId, const Path& path, bool waitAtGoal,
                           bool firstTask) {

  if (path.empty() || solverType != "sipps") {
    return;
  }

  int offset = path.beginTime, t = 0;
  if (!firstTask) {
    // If we are not the first task then our start location would be the same as the end location of the previous task so we cannot unassign that node!
    t += 1;
  }
  for (; t < (int)path.size(); t++) {
    int timestep = t + offset, location = path[t].location;
    assert((int)table[location].size() > timestep && table[location][timestep] == agentId);
    table[location][timestep] = NO_AGENT;
  }

  if (waitAtGoal) {
    goals[path.back().location] = MAX_TIMESTEP;
  }
  // Recompute makespan
  if (makespan == (int)path.endTime()) {
    makespan = 0;
    for (int time : goals) {
      if (time < MAX_TIMESTEP && time > makespan) {
        makespan = time;
      }
    }
  }
}

void PathTable::deleteAgent(int agentId, int goalLocation) {
  if (solverType != "sipps") {
    return;
  }
  for (auto& i : table) {
    for (int& j : i) {
      if (j == agentId) {
        j = NO_AGENT;
      }
    }
  }
  goals[goalLocation] = MAX_TIMESTEP;
}

bool PathTable::constrained(int fromLocation, int toLocation,
                            int toTime) const {

  if (!table.empty()) {
    if ((int)table[toLocation].size() > toTime &&
        table[toLocation][toTime] != NO_AGENT) {
      // Vertex conflict with agent table[to][to_time]
      return true;
    }
    if ((int)table[toLocation].size() >= toTime &&
        (int)table[fromLocation].size() > toTime &&
        !table[toLocation].empty() &&
        table[toLocation][toTime - 1] != NO_AGENT &&
        table[fromLocation][toTime] == table[toLocation][toTime - 1]) {
      // Edge conflict with agent table[to][to_time - 1]
      return true;
    }
  }

  if (!goals.empty()) {
    if (goals[toLocation] <= toTime) {
      // Target conflict
      return true;
    }
  }
  return false;
}

void PathTable::getConflictingAgents(set<int>& conflictingAgents,
                                     int fromLocation, int toLocation,
                                     int toTime) const {

  if (table.empty()) {
    return;
  }
  if ((int)table[toLocation].size() > toTime &&
      table[toLocation][toTime] != NO_AGENT) {
    // Vertex conflict
    conflictingAgents.insert(table[toLocation][toTime]);
  }
  if ((int)table[toLocation].size() >= toTime &&
      (int)table[fromLocation].size() > toTime &&
      table[toLocation][toTime - 1] != NO_AGENT &&
      table[fromLocation][toTime] == table[toLocation][toTime - 1]) {
    // Edge conflict
    conflictingAgents.insert(table[fromLocation][toTime]);
  }
  // TODO: collect target conflicts as well.
}

void PathTable::getAgents(set<int>& conflictingAgents, int location) const {
  if (location < 0) {
    return;
  }
  for (auto agent : table[location]) {
    if (agent >= 0) {
      conflictingAgents.insert(agent);
    }
  }
}

void PathTable::getAgents(set<int>& conflictingAgents, int neighborSize,
                          int location) const {

  if (location < 0 || table[location].empty()) {
    return;
  }

  int tMax = (int)table[location].size() - 1;
  while (table[location][tMax] == NO_AGENT && tMax > 0) {
    tMax--;
  }

  if (tMax == 0) {
    return;
  }

  int t0 = rand() % tMax;
  if (table[location][t0] != NO_AGENT) {
    conflictingAgents.insert(table[location][t0]);
  }

  int delta = 1;
  while (t0 - delta >= 0 || t0 + delta <= tMax) {

    if (t0 - delta >= 0 && table[location][t0 - delta] != NO_AGENT) {
      conflictingAgents.insert(table[location][t0 - delta]);
      if ((int)conflictingAgents.size() == neighborSize) {
        return;
      }
    }

    if (t0 + delta <= tMax && table[location][t0 + delta] != NO_AGENT) {
      conflictingAgents.insert(table[location][t0 + delta]);
      if ((int)conflictingAgents.size() == neighborSize) {
        return;
      }
    }
    delta++;
  }
}

// Get the holding time after the earliest_timestep for a location
int PathTable::getHoldingTime(int location, int earliestTimestep = 0) const {

  if (table.empty() or (int) table[location].size() <= earliestTimestep) {
    return earliestTimestep;
  }

  int rest = (int)table[location].size();
  while (rest > earliestTimestep and table[location][rest - 1] == NO_AGENT) {
    rest--;
  }

  return rest;
}

void PathTableWithCollisions::insertPath(int agentId, const Path& path) {

  paths_[agentId] = &path;

  if (path.empty()) {
    return;
  }
  for (int t = 0; t < (int)path.size(); t++) {
    if ((int)table[path[t].location].size() <= t) {
      table[path[t].location].resize(t + 1);
    }
    table[path[t].location][t].push_back(agentId);
  }

  assert(goals[path.back().location] == MAX_TIMESTEP);
  goals[path.back().location] = (int)path.size() - 1;
  makespan = max(makespan, (int)path.size() - 1);
}

void PathTableWithCollisions::insertPath(int agentId) {
  assert(paths_[agentId] != nullptr);
  insertPath(agentId, *paths_[agentId]);
}

void PathTableWithCollisions::deletePath(int agentId) {

  const Path& path = *paths_[agentId];

  if (path.empty()) {
    return;
  }

  for (int t = 0; t < (int)path.size(); t++) {
    assert((int)table[path[t].location].size() > t &&
           std::find(table[path[t].location][t].begin(),
                     table[path[t].location][t].end(),
                     agentId) != table[path[t].location][t].end());
    table[path[t].location][t].remove(agentId);
  }

  goals[path.back().location] = MAX_TIMESTEP;

  // Recompute makespan
  if (makespan == (int)path.size() - 1) {
    makespan = 0;
    for (int time : goals) {
      if (time < MAX_TIMESTEP && time > makespan) {
        makespan = time;
      }
    }
  }
}

int PathTableWithCollisions::getFutureNumOfCollisions(int location,
                                                      int time) const {
  assert(goals[location] == MAX_TIMESTEP);

  int rest = 0;
  if (!table.empty() && (int)table[location].size() > time) {
    for (int t = time + 1; t < (int)table[location].size(); t++) {
      rest += (int)table[location][t].size();  // vertex conflict
    }
  }

  return rest;
}

int PathTableWithCollisions::getNumOfCollisions(int fromLocation,
                                                int toLocation,
                                                int toTime) const {

  int rest = 0;

  if (!table.empty()) {

    if ((int)table[toLocation].size() > toTime) {
      // Vertex conflict
      rest += (int)table[toLocation][toTime].size();
    }
    if (fromLocation != toLocation && (int)table[toLocation].size() >= toTime &&
        (int)table[fromLocation].size() > toTime) {
      for (auto a1 : table[toLocation][toTime - 1]) {
        for (auto a2 : table[fromLocation][toTime]) {
          if (a1 == a2) {
            // Edge conflict
            rest++;
          }
        }
      }
    }
  }

  if (!goals.empty()) {
    if (goals[toLocation] < toTime) {
      // target conflict
      rest++;
    }
  }
  return rest;
}

bool PathTableWithCollisions::hasCollisions(int fromLocation, int toLocation,
                                            int toTime) const {

  if (!table.empty()) {
    if ((int)table[toLocation].size() > toTime and
        !table[toLocation][toTime].empty()) {
      // Vertex conflict
      return true;
    }
    if (fromLocation != toLocation && (int)table[toLocation].size() >= toTime &&
        (int)table[fromLocation].size() > toTime) {
      for (auto a1 : table[toLocation][toTime - 1]) {
        for (auto a2 : table[fromLocation][toTime]) {
          if (a1 == a2) {
            // Edge conflict
            return true;
          }
        }
      }
    }
  }

  if (!goals.empty()) {
    if (goals[toLocation] < toTime) {
      // Target conflict
      return true;
    }
  }

  return false;
}

bool PathTableWithCollisions::hasEdgeCollisions(int fromLocation,
                                                int toLocation,
                                                int toTime) const {

  if (!table.empty() && fromLocation != toLocation &&
      (int)table[toLocation].size() >= toTime &&
      (int)table[fromLocation].size() > toTime) {
    for (auto a1 : table[toLocation][toTime - 1]) {
      for (auto a2 : table[fromLocation][toTime]) {
        if (a1 == a2) {
          // edge conflict
          return true;
        }
      }
    }
  }

  return false;
}

int PathTableWithCollisions::getAgentWithTarget(int targetLocation,
                                                int latestTimestep) const {
  if (table.empty() || goals.empty() ||
      goals[targetLocation] > latestTimestep) {
    return -1;
  }

  // Look at all agents at the goal time
  for (auto id : table[targetLocation][goals[targetLocation]]) {
    if (paths_[id]->back().location == targetLocation) {
      // If agent id's goal is to, then this is the agent we want
      return id;
    }
  }

  // This should never happen
  assert(false);
  return -1;
}

int PathTableWithCollisions::getLastCollisionTimestep(int location) const {

  if (table.empty()) {
    return -1;
  }
  for (int t = (int)table[location].size() - 1; t >= 0; t--) {
    if (!table[location][t].empty()) {
      return t;
    }
  }

  return -1;
}

void PathTableWithCollisions::clear() {
  table.clear();
  goals.clear();
  paths_.clear();
}
