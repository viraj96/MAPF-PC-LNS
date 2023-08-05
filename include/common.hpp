#pragma once
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>  // std::setprecision
#include <iostream> // std::cout, std::fixed
#include <list>
#include <map>
#include <set>
#include <deque>
#include <queue> // adding for temporal order struct
#include <stack>
#include <tuple>
#include <vector>

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::pairing_heap;
using std::cerr;
using std::clock;
using std::cout;
using std::deque;
using std::distance;
using std::endl;
using std::get;
using std::hash;
using std::list;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::make_unique;
using std::map;
using std::max;
using std::min;
using std::ofstream;
using std::pair;
using std::reverse;
using std::set;
using std::shared_ptr;
using std::stack;
using std::string;
using std::tie;
using std::tuple;
using std::unique_ptr;
using std::vector;
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;
typedef std::priority_queue<pair<int, int>, vector<pair<int, int>>, std::greater<pair<int, int>>>
  pq;

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
    bool is_goal;
    int location = -1;
};

struct Path
{
    int begin_time = 0;
    int end_time() { return begin_time + (int)size() - 1; }

    vector<PathEntry> path;
    vector<int> timestamps;

    bool empty() const { return path.empty(); }
    size_t size() const { return path.size(); }
    PathEntry& back() { return path.back(); }
    PathEntry& front() { return path.front(); }
    const PathEntry& back() const { return path.back(); }
    const PathEntry& front() const { return path.front(); }
    const PathEntry& at(int idx) const { return path[idx]; }

    PathEntry& operator[](int idx) { return path[idx]; }
    const PathEntry& operator[](int idx) const { return path[idx]; }

    Path() {}
    Path(int size)
      : path(vector<PathEntry>(size))
    {}
};

std::ostream&
operator<<(std::ostream& os, const Path& path);
bool
isSamePath(const Path& p1, const Path& p2);

struct IterationStats
{
    double runtime;
    string algorithm;
    int num_of_agents, num_of_tasks, sum_of_costs, sum_of_costs_lower_bound,
      num_of_conflicting_pairs;
    IterationStats(double runtime,
                   string algorithm,
                   int num_of_agents,
                   int num_of_tasks,
                   int sum_of_costs,
                   int sum_of_costs_lower_bound = 0,
                   int num_of_conflicting_pairs = 0)
      : runtime(runtime)
      , algorithm(algorithm)
      , num_of_agents(num_of_agents)
      , num_of_tasks(num_of_tasks)
      , sum_of_costs(sum_of_costs)
      , sum_of_costs_lower_bound(sum_of_costs_lower_bound)
      , num_of_conflicting_pairs(num_of_conflicting_pairs)
    {}
};
