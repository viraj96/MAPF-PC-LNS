#pragma once

#include <boost/functional/hash.hpp>
#include "astar.hpp"
#include "reservationtable.hpp"

class SIPPNode : public LLNode {
 public:
  pairing_heap<SIPPNode*, compare<SIPPNode::OpenCompareNode>>::handle_type
      openHandle;
  pairing_heap<SIPPNode*, compare<SIPPNode::FocalCompareNode>>::handle_type
      focalHandle;

  bool collisionV{};
  // The upper bound with respect to expansion
  int highExpansion{};
  // The upper bound with respect to generation
  int highGeneration{};
  unsigned int stage = 0;
  SIPPNode(SIPPNode* parent, int location, int gVal, int hVal, int timestep,
           int numOfConflicts, int stage, int highGeneration, int highExpansion,
           bool collisionV)
      : LLNode(parent, location, gVal, hVal, timestep, numOfConflicts, stage),
        collisionV(collisionV),
        highExpansion(highExpansion),
        highGeneration(highGeneration) {}
  ~SIPPNode() = default;

  // Copy everything except for handles
  void copy(const SIPPNode& other) {
    LLNode::copy(other);
    collisionV = other.collisionV;
    highExpansion = other.highExpansion;
    highGeneration = other.highGeneration;
  }

  // The following is used by for generating the hash value of a nodes
  struct NodeHasher {
    std::size_t operator()(const std::shared_ptr<SIPPNode>& n) const {
      size_t seed = 0;
      boost::hash_combine(seed, n->location);
      boost::hash_combine(seed, n->highGeneration);
      return seed;
    }
  };

  // The following is used for checking whether two nodes are equal
  // We say that two nodes, s1 and s2, are equal if both are non-NULL and agree on the id and timestep
  struct CompareNode {
    bool operator()(const std::shared_ptr<SIPPNode>& lhs,
                    const std::shared_ptr<SIPPNode>& rhs) const {
      return (lhs == rhs) ||
             ((lhs != nullptr) && (rhs != nullptr) &&
              lhs->location == rhs->location &&
              lhs->waitAtGoal == rhs->waitAtGoal && lhs->stage == rhs->stage &&
              lhs->highGeneration == rhs->highGeneration);
    }
  };
};

class SIPP : public SingleAgentSolver {
 private:
  pairing_heap<SIPPNode*, boost::heap::compare<LLNode::OpenCompareNode>>
      openList_;
  pairing_heap<SIPPNode*, boost::heap::compare<LLNode::FocalCompareNode>>
      focalList_;

  unordered_map<std::shared_ptr<SIPPNode>, list<std::shared_ptr<SIPPNode>>,
                SIPPNode::NodeHasher, SIPPNode::CompareNode>
      allNodesTable_;

  list<std::shared_ptr<SIPPNode>> uselessNodes_;

  void updatePath(const LLNode* goal, Path& path);

  void releaseNodes();
  bool dominanceCheck(SIPPNode* newNode);
  inline void pushNodeToFocal(SIPPNode* node);

 public:
  string getName() const override { return "SIPP"; }
  // Return a path that minimizes collisions, breaking ties by cost
  Path findPathSegment(ConstraintTable& constraintTable, int startTime,
                       int stage, int lb) override;

  SIPP(const Instance& instance, int agent)
      : SingleAgentSolver(instance, agent) {}
};
