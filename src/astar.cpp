#include "astar.hpp"

void SingleAgentSolver::computeHeuristics() {
  struct Node {
    int location, value;
    Node(int location, int value) : location(location), value(value) {}

    struct CompareNode {
      bool operator()(const Node& lhs, const Node& rhs) const {
        return lhs.value >= rhs.value;
      }
    };
  };

  heuristic.clear();
  heuristicLandmarks.clear();
  heuristic.resize(goalLocations.size());
  heuristicLandmarks.resize(goalLocations.size(), 0);

  for (int i = 0; i < (int)goalLocations.size(); i++) {
    heuristic[i].resize(instance.mapSize, MAX_TIMESTEP);
    pairing_heap<Node, compare<Node::CompareNode>> heap;

    // h-val of the goal is always 0
    Node root(goalLocations[i], 0);
    heuristic[i][goalLocations[i]] = 0;

    heap.push(root);

    while (!heap.empty()) {
      Node current = heap.top();
      heap.pop();
      for (int nextLocation : instance.getNeighbors(current.location)) {
        if (heuristic[i][nextLocation] > current.value + 1) {
          heuristic[i][nextLocation] = current.value + 1;
          Node next(nextLocation, heuristic[i][nextLocation]);
          heap.push(next);
        }
      }
    }
  }

  for (int i = (int)goalLocations.size() - 2; i >= 0; i--) {
    heuristicLandmarks[i] =
        heuristicLandmarks[i + 1] + heuristic[i + 1][goalLocations[i]];
  }
}
