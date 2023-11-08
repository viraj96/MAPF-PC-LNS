#include "astar.hpp"

void SingleAgentSolver::computeHeuristics() {

  heuristic.clear();
  heuristicLandmarks.clear();
  heuristic.resize(goalLocations.size());
  heuristicLandmarks.resize(goalLocations.size(), 0);

  for (int i = 0; i < (int)goalLocations.size(); i++) {
    int globalTask = getGlobalTaskFromLocation(goalLocations[i]);
    heuristic[i] = instance.heuristics_[globalTask];
  }

  for (int i = (int)goalLocations.size() - 2; i >= 0; i--) {
    heuristicLandmarks[i] =
        heuristicLandmarks[i + 1] + heuristic[i + 1][goalLocations[i]];
  }
}
