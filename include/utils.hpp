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
  double sumOfNumConflicts{}, sumOfNumConflictsSquare{}, sumOfNumCosts{},
      sumOfNumCostsSquare{}, lnsConflictWeight{}, lnsCostWeight{};

  MovingMetrics(int size, int lnsConflictWeight, int lnsCostWeight,
                int numberOfConflicts, int sumOfCosts)
      : size(size),
        lnsConflictWeight(lnsConflictWeight),
        lnsCostWeight(lnsCostWeight) {
    conflictNum.resize(size, numberOfConflicts);
    conflictSquareNum.resize(size, pow(numberOfConflicts, 2));
    costNum.resize(size, sumOfCosts);
    costSquareNum.resize(size, pow(sumOfCosts, 2));
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
