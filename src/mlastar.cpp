#include "mlastar.hpp"

void MultiLabelSpaceTimeAStar::releaseNodes() {
  openList_.clear();
  focalList_.clear();
  for (MultiLabelAStarNode* node : allNodesTable_) {
    delete node;
  }
  allNodesTable_.clear();
}

inline void MultiLabelSpaceTimeAStar::pushNode(MultiLabelAStarNode* node) {
  numGenerated++;
  node->inOpenlist = true;
  node->openHandle = openList_.push(node);
  if (node->getFVal() <= lowerBound_) {
    node->focalHandle = focalList_.push(node);
  }
}

inline MultiLabelAStarNode* MultiLabelSpaceTimeAStar::popNode() {
  numExpanded++;
  MultiLabelAStarNode* node = focalList_.top();
  focalList_.pop();
  node->inOpenlist = false;
  openList_.erase(node->openHandle);
  return node;
}

void MultiLabelSpaceTimeAStar::updateFocalList() {
  MultiLabelAStarNode* openHead = openList_.top();
  // Focal list is always supposed to be the set of nodes in open list whose f-value does not exceed the minimum f-value of a node in open list
  if (openHead->getFVal() > minFVal_) {
    int newMinFVal = (int)openHead->getFVal();
    int newLowerBound = max(lowerBound_, newMinFVal);
    for (MultiLabelAStarNode* node : openList_) {
      if (node->getFVal() > lowerBound_ && node->getFVal() <= newLowerBound) {
        node->focalHandle = focalList_.push(node);
      }
    }
    minFVal_ = newMinFVal;
    lowerBound_ = newLowerBound;
  }
}

void MultiLabelSpaceTimeAStar::updatePath(const LLNode* goal, Path& path) {
  path.path.resize(goal->gVal + 1);
  path.timeStamps.resize(goalLocations.size(), 0);
  path.timeStamps.back() = goal->gVal;

  const LLNode* current = goal;
  while (current != nullptr) {
    path[current->gVal].location = current->location;
    if (current->parent != nullptr &&
        current->stage != current->parent->stage) {
      path.timeStamps[current->parent->stage] = current->gVal;
      path[current->gVal].isGoal = true;
    } else {
      path[current->gVal].isGoal = false;
    }
    current = current->parent;
  }
}

Path MultiLabelSpaceTimeAStar::findPathSegment(ConstraintTable& constraintTable,
                                               int startTime, int stage,
                                               int lb) {
  int location = startLocation;
  if (stage != 0) {
    location = goalLocations[stage - 1];
  }

  Path path;
  path.beginTime = startTime;
  auto* start = new MultiLabelAStarNode(
      nullptr, location, 0, getHeuristic(stage, location), startTime, 0, stage);

  // Ensure that the constraint table is built before we call this
  numGenerated++;
  start->inOpenlist = true;
  allNodesTable_.insert(start);
  minFVal_ = (int)start->getFVal();

  start->openHandle = openList_.push(start);
  start->focalHandle = focalList_.push(start);
  start->secondaryKeys.push_back(-start->gVal);

  int holdingTime = constraintTable.lengthMin;
  if (stage == (int)goalLocations.size() - 1) {
    holdingTime = constraintTable.getHoldingTime();
  }
  lowerBound_ = max(holdingTime - startTime, max(minFVal_, lb));

  while (!openList_.empty()) {
    updateFocalList();
    MultiLabelAStarNode* current = popNode();

    if (current->location == goalLocations[stage] &&
        current->timestep >= holdingTime) {
      updatePath(current, path);
      break;
    }

    if (current->timestep >= constraintTable.lengthMax) {
      continue;  // Why is this needed? Probably because this node is not "good" and needs to be discarded
    }

    list<int> successors = instance.getNeighbors(current->location);

    // We can stay at the same location for the next timestep
    successors.emplace_back(current->location);
    for (int successor : successors) {
      int nextTimestep = current->timestep + 1;

      if (max(constraintTable.size, constraintTable.latestTimestep) + 1 <
          current->timestep) {
        if (successor == current->location) {
          continue;
        }
        nextTimestep--;  // What does this do?
      }

      if (constraintTable.constrained(successor, nextTimestep) ||
          constraintTable.constrained(current->location, successor,
                                      nextTimestep)) {
        continue;
      }

      // Setting the stage
      unsigned int stage = current->stage;
      vector<int> timestamps = current->timeStamps;

      int successorGVal = current->gVal + 1;
      int successorHVal =
          max(getHeuristic(stage, successor), holdingTime - nextTimestep);
      int successorInternalConflicts = current->numOfConflicts;
      auto* next = new MultiLabelAStarNode(current, successor, successorGVal,
                                           successorHVal, nextTimestep,
                                           successorInternalConflicts, stage);
      next->timeStamps = timestamps;
      next->secondaryKeys.push_back(-successorGVal);
      next->distanceToNext = heuristic[stage][successor];

      if (next->stage == goalLocations.size() - 1 &&
          successor == goalLocations.back() &&
          current->location == goalLocations.back()) {
        next->waitAtGoal = true;
      }

      // Try to retrieve it from the hash table
      auto it = allNodesTable_.find(next);
      if (it == allNodesTable_.end()) {
        pushNode(next);
        allNodesTable_.insert(next);
        continue;
      }

      // If we found existing entry then we need to update it but only if its in the open list
      if ((*it)->getFVal() > next->getFVal() ||
          ((*it)->getFVal() == next->getFVal() &&
           LLNode::FocalCompareNode()((*it), next))) {
        if (!(*it)->inOpenlist) {
          (*it)->copy(*next);
          pushNode(*it);
        } else {
          bool addToFocal = false, updateInFocal = false, updateOpen = false;
          // New node can be in focal list
          if ((successorGVal + successorHVal) <= lowerBound_) {
            if ((*it)->getFVal() > lowerBound_) {
              addToFocal = true;  // Old node could not be in focal list
            } else {
              updateInFocal =
                  true;  // Old node could be in focal list so need to update
            }
          }
          if ((*it)->getFVal() > successorGVal + successorHVal) {
            updateOpen =
                true;  // This node would not have been popped yet from the open list
          }
          (*it)->copy(*next);
          if (updateOpen) {
            openList_.increase((*it)->openHandle);
          }
          if (addToFocal) {
            (*it)->focalHandle = focalList_.push(*it);
          }
          if (updateInFocal) {
            focalList_.update((*it)->focalHandle);
          }
        }
      }
      delete next;
    }
  }
  releaseNodes();
  return path;
}
