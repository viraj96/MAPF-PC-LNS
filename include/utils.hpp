#pragma once

#include "common.hpp"
#include "instance.hpp"
#include "lns.hpp"
#include "mlastar.hpp"

int
greedy_task_assignment(Instance* instance);

bool
topological_sort(LNS* lns_instance, vector<int>& planning_order);
