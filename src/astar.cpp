#include "astar.hpp"

void SingleAgentSolver::computeHeuristics() {

  heuristic.clear();
  heuristicLandmarks.clear();
  heuristic.resize(goalLocations.size());
  heuristicLandmarks.resize(goalLocations.size(), 0);

  for (int i = 0; i < (int)goalLocations.size(); i++) {
    int globalTask = getGlobalTaskFromLocation(goalLocations[i]);
    if (globalTask == UNDEFINED) {
      auto it = find(instance.startLocations_.begin(), instance.startLocations_.end(), startLocation);
      assert(it != instance.startLocations_.end());
      int agentId = it - instance.startLocations_.begin(); 
      heuristic[goalLocations.size() - 1] = instance.startLocationHeuristics_[agentId];
    }
    else {
      heuristic[i] = instance.heuristics_[globalTask];
    }
  }

  for (int i = (int)goalLocations.size() - 2; i >= 0; i--) {
    heuristicLandmarks[i] =
        heuristicLandmarks[i + 1] + heuristic[i + 1][goalLocations[i]];
  }
}
