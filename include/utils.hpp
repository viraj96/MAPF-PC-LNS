#pragma once

#include "common.hpp"
#include "instance.hpp"
#include "lns.hpp"

void
greedyTaskAssignment(const Instance* instance, Solution* solution);

bool
topologicalSort(const Instance* instance,
                 vector<pair<int, int>>* precedenceConstraints,
                 vector<int>& planningOrder);
