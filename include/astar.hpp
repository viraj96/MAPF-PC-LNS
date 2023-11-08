#pragma once

#include <plog/Log.h>
#include "common.hpp"
#include "constrainttable.hpp"
#include "instance.hpp"

#include <utility>

class LLNode {

 public:
  vector<int> timeStamps;
  vector<int> secondaryKeys;

  LLNode* parent = nullptr;
  int location{}, gVal{}, hVal = 0, timestep = 0, numOfConflicts = 0;
  bool inOpenlist = false, waitAtGoal = false;
  unsigned int stage = 0, distanceToNext = 0;

  struct OpenCompareNode {
    bool operator()(const LLNode* lhs, const LLNode* rhs) const {
      if (lhs->gVal + lhs->hVal == rhs->gVal + rhs->hVal) {
        if (lhs->hVal == rhs->hVal) {
          return rand() % 2 == 0;
        }
        return lhs->hVal >= rhs->hVal;
      }
      return lhs->gVal + lhs->hVal >= rhs->gVal + rhs->hVal;
    }
  };

  struct CompareTimestamps {
    int operator()(const LLNode* lhs, const LLNode* rhs) const {
      for (int i = 0;
           i < min((int)lhs->timeStamps.size(), (int)rhs->timeStamps.size());
           i++) {
        if (lhs->timeStamps[i] != rhs->timeStamps[i]) {
          return lhs->timeStamps[i] > rhs->timeStamps[i] ? 1 : -1;
        }
      }

      int lhsLastVal, rhsLastVal;
      if (lhs->timeStamps.size() > rhs->timeStamps.size()) {
        lhsLastVal = lhs->timeStamps[rhs->timeStamps.size()];
        rhsLastVal = rhs->gVal + rhs->distanceToNext;
      } else if (lhs->timeStamps.size() < rhs->timeStamps.size()) {
        lhsLastVal = lhs->gVal + lhs->distanceToNext;
        rhsLastVal = rhs->timeStamps[lhs->timeStamps.size()];
      } else {
        lhsLastVal = lhs->gVal + lhs->distanceToNext;
        rhsLastVal = rhs->gVal + rhs->distanceToNext;
      }

      if (lhsLastVal != rhsLastVal) {
        return lhsLastVal > rhsLastVal ? 1 : -1;
      }

      return 0;
    }
  };

  struct FocalCompareNode {
    bool operator()(const LLNode* lhs, const LLNode* rhs) const {
      for (int i = 0; i < min((int)lhs->secondaryKeys.size(),
                              (int)rhs->secondaryKeys.size());
           i++) {
        if (lhs->secondaryKeys[i] != rhs->secondaryKeys[i]) {
          return lhs->secondaryKeys[i] > rhs->secondaryKeys[i];
        }
      }
      int timeStampsComparison = CompareTimestamps()(lhs, rhs);
      if (timeStampsComparison != 0) {
        return timeStampsComparison == 1;
      }
      if (lhs->numOfConflicts == rhs->numOfConflicts) {
        if (lhs->gVal + lhs->hVal == rhs->gVal + rhs->hVal) {
          if (lhs->hVal == rhs->hVal) {
            return rand() % 2 == 0;
          }
          return lhs->hVal >= rhs->hVal;
        }
        return lhs->gVal + lhs->hVal >= rhs->gVal + rhs->hVal;
      }
      return lhs->numOfConflicts >= rhs->numOfConflicts;
    }
  };

  LLNode() = default;
  LLNode(LLNode* parent, int location, int gVal, int hVal, int timestep,
         int numOfConflicts, unsigned int stage)
      : parent(parent),
        location(location),
        gVal(gVal),
        hVal(hVal),
        timestep(timestep),
        numOfConflicts(numOfConflicts),
        stage(stage) {}
  LLNode(const LLNode& old) { copy(old); }

  inline double getFVal() const { return gVal + hVal; }

  void copy(const LLNode& old) {
    location = old.location;
    gVal = old.gVal;
    hVal = old.hVal;
    parent = old.parent;
    timestep = old.timestep;
    numOfConflicts = old.numOfConflicts;
    waitAtGoal = old.waitAtGoal;
  }
};

class SingleAgentSolver {
 public:
  uint64_t numExpanded = 0, numGenerated = 0;

  bool useTimeStamps = true;

  const Instance& instance;

  int startLocation;
  vector<int> goalLocations;
  vector<int> heuristicLandmarks;

  vector<vector<int>> heuristic;

  void computeHeuristics();
  int getHeuristic(int stage, int location) const {
    return heuristic[stage][location] + heuristicLandmarks[stage];
  }
  int computeHeuristic(int from, int to) const {
    return instance.getManhattanDistance(from, to);
  }
  inline void setGoalLocations(vector<int> goals) {
    goalLocations = std::move(goals);
  }
  
  int getGlobalTaskFromLocation(int taskLocation) const {
    vector<int> taskLocations = instance.getTaskLocations();
    auto it = std::find(taskLocations.begin(),
                        taskLocations.end(), taskLocation);
    assert(it != taskLocations.end());
    return it - taskLocations.begin();
  }

  virtual string getName() const = 0;
  virtual Path findPathSegment(ConstraintTable& constraintTable, int startTime,
                               int stage, int lowerBound) = 0;
  list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }

  SingleAgentSolver(const Instance& instance, int agent)
      : instance(instance),
        startLocation(instance.startLocations_[agent]),
        goalLocations(instance.taskLocations_) {
    computeHeuristics();
  }
  virtual ~SingleAgentSolver() = default;
};
