#pragma once

#include "astar.hpp"
#include "common.hpp"
#include "constrainttable.hpp"
#include <plog/Log.h>

class MultiLabelAStarNode : public LLNode
{
  public:
    pairing_heap<MultiLabelAStarNode*, compare<LLNode::OpenCompareNode>>::handle_type openHandle;
    pairing_heap<MultiLabelAStarNode*, compare<LLNode::FocalCompareNode>>::handle_type focalHandle;

    MultiLabelAStarNode() = default;

    MultiLabelAStarNode(const MultiLabelAStarNode& old): LLNode(old) {}

    MultiLabelAStarNode(LLNode* parent,
                        int location,
                        int gVal,
                        int hVal,
                        int timestep,
                        int numOfConflicts,
                        unsigned int stage)
      : LLNode(parent, location, gVal, hVal, timestep, numOfConflicts, stage)
    {}

    ~MultiLabelAStarNode() = default;

    struct NodeHasher
    {
        size_t operator()(const MultiLabelAStarNode* node) const
        {
            size_t locationHash = hash<int>()(node->location);
            size_t stageHash = hash<int>()(node->stage);
            size_t timestepHash = hash<int>()(node->timestep);
            return (locationHash ^ (timestepHash << 1) ^ (stageHash << 1));
        }
    };

    struct CompareNode
    {
        bool operator()(const MultiLabelAStarNode* lhs, const MultiLabelAStarNode* rhs) const
        {
            return (lhs == rhs) || ((lhs != nullptr) && (rhs != nullptr) && lhs->location == rhs->location &&
                                    lhs->timestep == rhs->timestep && lhs->stage == rhs->stage &&
                                    lhs->waitAtGoal == rhs->waitAtGoal);
        }
    };
};

class MultiLabelSpaceTimeAStar : public SingleAgentSolver
{
  private:
    pairing_heap<MultiLabelAStarNode*, compare<MultiLabelAStarNode::OpenCompareNode>> openList_;
    pairing_heap<MultiLabelAStarNode*, compare<MultiLabelAStarNode::FocalCompareNode>> focalList_;

    int minFVal_{}, lowerBound_{};

    unordered_set<MultiLabelAStarNode*,
                  MultiLabelAStarNode::NodeHasher,
                  MultiLabelAStarNode::CompareNode>
      allNodesTable_;

    void releaseNodes();
    void updateFocalList();
    inline MultiLabelAStarNode* popNode();
    inline void pushNode(MultiLabelAStarNode* node);
    void updatePath(const LLNode* goal, Path& path);

  public:
    MultiLabelSpaceTimeAStar(const Instance& instance, int agent)
      : SingleAgentSolver(instance, agent)
    {}
    string getName() const override { return "MLAStar"; }
    Path findPathSegment(ConstraintTable& constraintTable, int startTime, int stage, int lb) override;
};
