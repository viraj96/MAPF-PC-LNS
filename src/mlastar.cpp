#include "mlastar.hpp"

inline MultiLabelAStarNode*
MultiLabelSpaceTimeAStar::popNode()
{
    num_expanded++;
    MultiLabelAStarNode* node = focal_list.top();
    focal_list.pop();
    node->in_openlist = false;
    open_list.erase(node->open_handle);
    return node;
}

void
MultiLabelSpaceTimeAStar::updateFocalList()
{
    MultiLabelAStarNode* open_head = open_list.top();
    if (open_head->getFVal() > min_f_val) {
        int new_min_f_val = (int)open_head->getFVal();
        int new_lower_bound = max(lower_bound, new_min_f_val);
        for (MultiLabelAStarNode* node : open_list)
            if (node->getFVal() > lower_bound && node->getFVal() <= new_lower_bound)
                node->focal_handle = focal_list.push(node);
        min_f_val = new_min_f_val;
        lower_bound = new_lower_bound;
    }
}

void
MultiLabelSpaceTimeAStar::updatePath(const LLNode* goal, Path& path)
{
    path.path.resize(goal->g_val + 1);
    path.timestamps.resize(goal_locations.size(), 0);
    path.timestamps.back() = goal->g_val;

    const LLNode* current = goal;
    while (current != nullptr) {
        path[current->g_val].location = current->location;
        if (current->parent != nullptr && current->stage != current->parent->stage) {
            path.timestamps[current->parent->stage] = current->g_val;
            path[current->g_val].is_goal = true;
        } else
            path[current->g_val].is_goal = false;
        current = current->parent;
    }
}

Path
MultiLabelSpaceTimeAStar::findPathSegment(ConstraintTable& constraint_table,
                                          int start_time,
                                          int stage,
                                          int lb)
{
    int location = start_location;
    if (stage != 0)
        location = goal_locations[stage - 1];

    Path path;
    path.begin_time = start_time;
    MultiLabelAStarNode* start = new MultiLabelAStarNode(
      nullptr, location, 0, get_heuristic(stage, location), start_time, 0, stage);

    // ensure that the constraint table is built before we call this
    num_generated++;
    start->in_openlist = true;
    allNodes_table.insert(start);
    min_f_val = (int)start->getFVal();
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->secondary_keys.push_back(-start->g_val);
    int holding_time = constraint_table.length_min;
    if (stage == goal_locations.size() - 1)
        constraint_table.getHoldingTime();
    lower_bound = max(holding_time - start_time, max(min_f_val, lb));

    while (!open_list.empty()) {
        updateFocalList();
        MultiLabelAStarNode* current = popNode();

        if (current->location == goal_locations.back() &&
            current->stage == goal_locations.size() - 1 && !current->wait_at_goal &&
            current->timestep >= holding_time) {
            updatePath(current, path);
            break;
        }

        if (current->timestep >= constraint_table.length_max)
            continue; // Why is this needed?
    }
}
