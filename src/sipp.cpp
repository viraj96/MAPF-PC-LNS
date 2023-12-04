#include "sipp.hpp"

void SIPP::updatePath(const LLNode* goal, Path& path) {

  int offset = path.beginTime;
  path.path.resize(goal->timestep + 1 - offset);

  const auto* curr = goal;
  // Non-root node
  while (curr->parent != nullptr) {
    const auto* prev = curr->parent;
    int t = prev->timestep + 1 - offset;

    while (t < curr->timestep - offset) {
      // Wait at prev location
      path[t].location = prev->location;
      t++;
    }

    // Move to curr location
    path[curr->timestep - offset].location = curr->location;
    curr = prev;
  }

  assert(curr->timestep - offset == 0);
  path[0].location = curr->location;
}

// Find path by A*
// Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
Path SIPP::findPathSegment(ConstraintTable& constraintTable, int startTime,
                           int stage, int lb) {

  reset();

  int location = startLocation;
  if (stage != 0) {
    location = goalLocations[stage - 1];
  }

  ReservationTable reservationTable(constraintTable);

  Path path;
  path.beginTime = startTime;

  Interval interval =
      reservationTable.getFirstSafeInterval(location, startTime);
  if (interval == Interval{}) {
    return path;
  }

  // You only need to hold if you are the last task
  int holdingTime = constraintTable.lengthMin;
  if (stage == (int)goalLocations.size() - 1) {
    holdingTime = constraintTable.getHoldingTime();
  }

  int lastTargetCollisionTime =
      constraintTable.getLastCollisionTimestep(constraintTable.goalLocation);

  // Generate start and add it to the OPEN & FOCAL list
  auto heuristicVal = max(max(getHeuristic(stage, startLocation), holdingTime),
                          lastTargetCollisionTime + 1);
  auto start =
      new SIPPNode(nullptr, location, 0, heuristicVal, startTime,
                   static_cast<int>(get<2>(interval)), stage, get<1>(interval),
                   get<1>(interval), get<2>(interval));
  pushNodeToFocal(start);

  while (!focalList_.empty()) {

    SIPPNode* current = focalList_.top();
    focalList_.pop();

    current->inOpenlist = false;
    numExpanded_++;

    assert(current->location >= 0);

    // Check if the popped node is a goal
    if (current->location == goalLocations[stage]) {
      // current->timestep >= holdingTime) {
      updatePath(current, path);
      break;
    }

    // If we arrive at the goal location and the agent can hold the goal location afterward and not wait at the goal location
    if (current->location == constraintTable.goalLocation &&
        !current->waitAtGoal && current->timestep >= holdingTime) {

      int futureCollisions = constraintTable.getFutureNumOfCollisions(
          current->location, current->timestep);
      if (futureCollisions == 0) {
        updatePath(current, path);
        break;
      }

      // Generate a goal node
      auto goal = new SIPPNode(*current);
      goal->hVal = 0;
      goal->numOfConflicts += futureCollisions;

      // Try to retrieve it from the hash table
      if (dominanceCheck(goal)) {
        pushNodeToFocal(goal);
      } else {
        delete goal;
      }
    }

    // Move to neighboring locations
    for (int nextLocation : instance.getNeighbors(current->location)) {

      for (auto& inter : reservationTable.getSafeIntervals(
               current->location, nextLocation, current->timestep + 1,
               current->highExpansion + 1)) {

        int nextHighGeneration, nextTimestep, nextHighExpansion;
        bool nextCollisionV, nextCollisionE;
        tie(nextHighGeneration, nextTimestep, nextHighExpansion, nextCollisionV,
            nextCollisionE) = inter;

        if (nextTimestep + getHeuristic(stage, nextLocation) >
            constraintTable.lengthMax) {
          break;
        }

        int nextCollisions =
            current->numOfConflicts + (int)nextCollisionV + (int)nextCollisionE;
        int nextHVal =
            max(getHeuristic(stage, nextLocation),
                (int)((nextCollisions > 0 ? holdingTime : current->getFVal()) -
                      nextTimestep));

        // Generate (maybe temporary) node
        auto next =
            new SIPPNode(current, nextLocation, nextTimestep, nextHVal,
                         nextTimestep, nextCollisions, stage,
                         nextHighGeneration, nextHighExpansion, nextCollisionV);

        // Try to retrieve it from the hash table
        if (dominanceCheck(next)) {
          pushNodeToFocal(next);
        } else {
          delete next;
        }
      }
    }  // End for loop that generates successors

    // Wait at the current location
    if (current->highExpansion == current->highGeneration &&
        reservationTable.findSafeInterval(interval, current->location,
                                          current->highExpansion) &&
        get<0>(interval) + current->hVal <=
            reservationTable.constraintTable.lengthMax) {

      int nextTimestep = get<0>(interval);
      int nextHVal =
          max(getHeuristic(current->stage, current->location),
              (int)((get<2>(interval) ? holdingTime : current->getFVal()) -
                    nextTimestep));
      auto nextCollisions = current->numOfConflicts + (int)get<2>(interval);
      auto next =
          new SIPPNode(current, current->location, nextTimestep, nextHVal,
                       nextTimestep, nextCollisions, current->stage,
                       get<1>(interval), get<1>(interval), get<2>(interval));
      next->waitAtGoal = (current->location == constraintTable.goalLocation);
      if (dominanceCheck(next)) {
        pushNodeToFocal(next);
      } else {
        delete next;
      }
    }
  }  // End while loop
  releaseNodes();
  return path;
}

inline void SIPP::pushNodeToFocal(SIPPNode* node) {
  numGenerated_++;
  std::shared_ptr<SIPPNode> nodePtr = std::make_shared<SIPPNode>(*node);
  allNodesTable_[nodePtr].push_back(nodePtr);
  node->inOpenlist = true;
  // We only use focal list; no open list is used
  node->focalHandle = focalList_.push(node);
}

void SIPP::releaseNodes() {

  focalList_.clear();
  allNodesTable_.clear();
  uselessNodes_.clear();
}

// Return true iff we the new node is not dominated by any old node
bool SIPP::dominanceCheck(SIPPNode* newNode) {

  std::shared_ptr<SIPPNode> newNodePtr = std::make_shared<SIPPNode>(*newNode);
  auto ptr = allNodesTable_.find(newNodePtr);
  if (ptr == allNodesTable_.end()) {
    return true;
  }

  for (auto& oldNode : ptr->second) {

    if (oldNode->timestep <= newNode->timestep &&
        oldNode->numOfConflicts <= newNode->numOfConflicts) {
      // The new node is dominated by the old node
      return false;
    }

    if (oldNode->timestep >= newNode->timestep &&
        oldNode->numOfConflicts >= newNode->numOfConflicts) {
      // The old node is dominated by the new node
      if (oldNode->inOpenlist) {  // The old node has not been expanded yet
        // Delete the old node
        focalList_.erase(oldNode->focalHandle);
      } else {
        // The old node has been expanded already
        numReopened_++;  // Re-expand it
      }
      uselessNodes_.push_back(oldNode);
      ptr->second.remove(oldNode);
      // This is because we later will increase this variable when we insert the new node into lists.
      numGenerated_--;
      return true;
    }

    if (oldNode->timestep < newNode->highExpansion &&
        newNode->timestep < oldNode->highExpansion) {
      // Intervals overlap i.e we need to split the node to make them disjoint
      if (oldNode->timestep <= newNode->timestep) {
        assert(oldNode->numOfConflicts > newNode->numOfConflicts);
        oldNode->highExpansion = newNode->timestep;
      } else {
        // i.e. old_node->timestep > new_node->timestep
        assert(oldNode->numOfConflicts <= newNode->numOfConflicts);
        newNode->highExpansion = oldNode->timestep;
      }
    }
  }
  return true;
}
