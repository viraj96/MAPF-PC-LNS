#include "astar.hpp"

void
SingleAgentSolver::compute_heuristics()
{
    struct Node
    {
        int location, value;
        Node(int location, int value)
          : location(location)
          , value(value)
        {}

        struct compare_node
        {
            bool operator()(const Node& lhs, const Node& rhs) const
            {
                return lhs.value >= rhs.value;
            }
        };
    };

    heuristic.clear();
    heuristic_landmarks.clear();
    heuristic.resize(goal_locations.size());
    heuristic_landmarks.resize(goal_locations.size(), 0);

    for (int i = 0; i < (int)goal_locations.size(); i++) {
        heuristic[i].resize(instance.map_size, MAX_TIMESTEP);
        pairing_heap<Node, compare<Node::compare_node>> heap;

        // h-val of the goal is always 0
        Node root(goal_locations[i], 0);
        heuristic[i][goal_locations[i]] = 0;

        heap.push(root);

        while (!heap.empty()) {
            Node current = heap.top();
            heap.pop();
            for (int next_location : instance.getNeighbors(current.location))
                if (heuristic[i][next_location] > current.value + 1) {
                    heuristic[i][next_location] = current.value + 1;
                    Node next(next_location, heuristic[i][next_location]);
                    heap.push(next);
                }
        }
    }

    for (int i = (int)goal_locations.size() - 2; i >= 0; i--)
        heuristic_landmarks[i] = heuristic_landmarks[i + 1] + heuristic[i + 1][goal_locations[i]];
}
