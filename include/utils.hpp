#pragma once

#include "common.hpp"
#include "instance.hpp"
#include "lns.hpp"
#include "mlastar.hpp"

void
greedy_task_assignment(Instance* instance);

bool
topological_sort(Instance* instance, vector<int>& planning_order);
