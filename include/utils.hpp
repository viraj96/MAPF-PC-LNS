#pragma once

#include "common.hpp"
#include "instance.hpp"
#include "lns.hpp"

void greedyTaskAssignment(const Instance* instance, Solution* solution);

bool topologicalSort(const Instance* instance,
                     vector<pair<int, int>>* precedenceConstraints,
                     vector<int>& planningOrder);

set<Conflicts> extractNConflicts(int size, const set<Conflicts>& conflicts);

struct MovingMetrics {
  int size{}, oldestValue = 0;
  vector<double> conflictNum{}, conflictSquareNum{}, costNum{}, costSquareNum{};
  double lnsConflictWeight{}, lnsCostWeight{}, sumOfNumConflicts{},
      sumOfNumConflictsSquare{}, sumOfNumCosts{}, sumOfNumCostsSquare{};

  MovingMetrics(int size, double lnsConflictWeight, double lnsCostWeight,
                int numberOfConflicts, int sumOfCosts)
      : size(size),
        lnsConflictWeight(lnsConflictWeight),
        lnsCostWeight(lnsCostWeight) {
    conflictNum.resize(size);
    conflictNum[oldestValue] = numberOfConflicts;
    conflictSquareNum.resize(size);
    conflictSquareNum[oldestValue] = pow(numberOfConflicts, 2);
    costNum.resize(size);
    costNum[oldestValue] = sumOfCosts;
    costSquareNum.resize(size);
    costSquareNum[oldestValue] = pow(sumOfCosts, 2);
    sumOfNumConflicts =
        std::accumulate(begin(conflictNum), end(conflictNum), 0.0);
    sumOfNumConflictsSquare =
        std::accumulate(begin(conflictSquareNum), end(conflictSquareNum), 0.0);
    sumOfNumCosts = std::accumulate(begin(costNum), end(costNum), 0.0);
    sumOfNumCostsSquare =
        std::accumulate(begin(costSquareNum), end(costSquareNum), 0.0);
  }
  double computeMovingMetrics(int numberOfConflicts, int sumOfCosts);
};
