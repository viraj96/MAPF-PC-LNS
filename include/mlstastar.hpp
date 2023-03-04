#pragma once

#include "astar.hpp"
#include "common.hpp"
#include <plog/Log.h>

class MultiLabelAStarNode : public LLNode
{
  public:
    pairing_heap<MultiLabelAStarNode*, compare<LLNode::open_compare_node>>::handle_type open_handle;
    pairing_heap<MultiLabelAStarNode*, compare<LLNode::focal_compare_node>>::handle_type
      focal_handle;

    MultiLabelAStarNode()
      : LLNode()
    {}

    MultiLabelAStarNode(const MultiLabelAStarNode& old)
      : LLNode(old)
    {}

    MultiLabelAStarNode(LLNode* parent,
                        int location,
                        int g_val,
                        int h_val,
                        int timestep,
                        int num_of_conflicts,
                        unsigned int stage)
      : LLNode(parent, location, g_val, h_val, timestep, num_of_conflicts, stage)
    {}

    struct NodeHasher
    {
        size_t operator()(const MultiLabelAStarNode* node) const
        {
            size_t location_hash = hash<int>()(node->location);
            size_t stage_hash = hash<int>()(node->stage);
            size_t timestep_hash = hash<int>()(node->timestep);
            return (location_hash ^ (timestep_hash << 1) ^ (stage_hash << 1));
        }
    };

    struct compare_node
    {
        bool operator()(const MultiLabelAStarNode* lhs, const MultiLabelAStarNode* rhs) const
        {
            return (lhs == rhs) || (lhs && rhs && lhs->location == rhs->location &&
                                    lhs->timestep == rhs->timestep && lhs->stage == rhs->stage &&
                                    lhs->wait_at_goal == rhs->wait_at_goal);
        }
    };
};

class MultiLabelSpaceTimeAStar : public SingleAgentSolver
{
  private:
    pairing_heap<MultiLabelAStarNode*, compare<MultiLabelAStarNode::open_compare_node>> open_list;
    pairing_heap<MultiLabelAStarNode*, compare<MultiLabelAStarNode::focal_compare_node>> focal_list;

    int min_f_val, lower_bound;

    unordered_set<MultiLabelAStarNode*,
                  MultiLabelAStarNode::NodeHasher,
                  MultiLabelAStarNode::compare_node>
      allNodes_table;

    Path findShortestPath(ConstraintTable& constraint_table,
                          const pair<int, int> start_state,
                          int lower_bound);
    Path findPath(ConstraintTable& constraint_table,
                  const pair<int, int> start,
                  const pair<int, int> goal);

    void releaseNodes();
    void updateFocalList();
    inline MultiLabelAStarNode* popNode();
    inline void pushNode(MultiLabelAStarNode* node);
    void updatePath(const LLNode* goal, Path& path);
};
