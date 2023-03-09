#include "mlastar.hpp"

void
MultiLabelSpaceTimeAStar::releaseNodes()
{
    open_list.clear();
    focal_list.clear();
    for (MultiLabelAStarNode* node : allNodes_table)
        delete node;
    allNodes_table.clear();
}

inline void
MultiLabelSpaceTimeAStar::pushNode(MultiLabelAStarNode* node)
{
    num_generated++;
    node->in_openlist = true;
    node->open_handle = open_list.push(node);
    if (node->getFVal() <= lower_bound)
        node->focal_handle = focal_list.push(node);
}

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
    if (stage == (int)goal_locations.size() - 1)
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

        list<int> successors = instance.getNeighbors(current->location);
        // we can stay at the same location for the next timestep
        successors.emplace_back(current->location);
        for (int successor : successors) {
            int next_timestep = current->timestep + 1;

            // now everything is static so switch to space A* where we always use the same timestep
            if (max(constraint_table.size, constraint_table.latest_timestep) + 1 <
                current->timestep) {
                if (successor == current->location)
                    continue;
                next_timestep--;
            }

            if (constraint_table.constrained(successor, next_timestep) ||
                constraint_table.constrained(current->location, successor, next_timestep))
                continue;

            // setting the stage
            unsigned int stage = current->stage;
            vector<int> timestamps = current->timestamps;

            int successor_g_val = current->g_val + 1;
            int successor_h_val = max(get_heuristic(stage, successor), holding_time - successor);
            int successor_internal_conflicts = current->num_of_conflicts;
            MultiLabelAStarNode* next = new MultiLabelAStarNode(current,
                                                                successor,
                                                                successor_g_val,
                                                                successor_h_val,
                                                                next_timestep,
                                                                successor_internal_conflicts,
                                                                stage);
            next->timestamps = timestamps;
            next->secondary_keys.push_back(-successor_g_val);
            next->distance_to_next = heuristic[stage][successor];

            if (next->stage == goal_locations.size() - 1 && successor == goal_locations.back() &&
                current->location == goal_locations.back())
                next->wait_at_goal = true;

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end()) {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }

            // if we found existing entry then we need to update it but only if its in the open list
            if ((*it)->getFVal() > next->getFVal() || ((*it)->getFVal() == next->getFVal() &&
                                                       LLNode::focal_compare_node()((*it), next))) {
                if (!(*it)->in_openlist) {
                    (*it)->copy(*next);
                    pushNode(*it);
                } else {
                    bool add_to_focal = false, update_in_focal = false, update_open = false;
                    // new node can be in focal list
                    if ((successor_g_val + successor_h_val) <= lower_bound) {
                        if ((*it)->getFVal() > lower_bound)
                            add_to_focal = true; // old node could not be in focal list
                        else
                            update_in_focal =
                              true; // old node could be in focal list so need to update
                    }
                    if ((*it)->getFVal() > successor_g_val + successor_h_val)
                        update_open = true;
                    (*it)->copy(*next);
                    if (update_open)
                        open_list.increase((*it)->open_handle);
                    if (add_to_focal)
                        (*it)->focal_handle = focal_list.push(*it);
                    if (update_in_focal)
                        focal_list.update((*it)->focal_handle);
                }
            }
            delete next;
        }
    }
    releaseNodes();
    return path;
}
