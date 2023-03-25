#pragma once

#include "common.hpp"
#include "instance.hpp"
#include "lns.hpp"
#include "mlastar.hpp"

void
greedy_task_assignment(const Instance* instance, Solution* solution);

bool
topological_sort(const Instance* instance,
                 vector<pair<int, int>>* precedence_constraints,
                 vector<int>& planning_order);
