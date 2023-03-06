#pragma once

#include "common.hpp"
#include <plog/Log.h>

class Instance
{
  protected:
    vector<bool> map;
    string map_fname;
    string agent_task_fname;

    int num_of_agents, num_of_tasks;
    vector<vector<int>> task_assignments;
    vector<int> task_locations, start_locations;
    unordered_map<int, vector<int>> task_dependencies;

    bool loadMap();
    bool loadAgentsAndTasks();
    void saveMap() const;
    void printMap() const;
    void saveAgents() const;

    bool isConnected(int start, int goal);
    friend class SingleAgentSolver;

  public:
    int num_of_cols;
    int num_of_rows;
    int map_size;

    Instance() {}
    Instance(const string& map_fname,
             const string& agent_task_fname,
             int num_of_agents = 0,
             int num_of_tasks = 0);

    void printAgents() const;
    void assignTaskToAgent(int agent, int task);
    list<int> getNeighbors(int curr) const;
    inline bool isObstacle(int loc) const { return map[loc]; }
    inline bool validMove(int curr, int next) const
    {
        if (next < 0 || next >= map_size || map[next])
            return false;
        return getManhattanDistance(curr, next) < 2;
    };

    inline int linearizeCoordinate(int row, int col) const
    {
        return (this->num_of_cols * row + col);
    }
    inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
    inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
    inline pair<int, int> getCoordinate(int id) const
    {
        return make_pair(getRowCoordinate(id), getColCoordinate(id));
    }
    inline int getCols() const { return num_of_cols; }
    inline int getAgentNum() const { return num_of_agents; }
    inline int getTasksNum() const { return num_of_tasks; }
    inline vector<int> getTaskLocations() const { return task_locations; }
    inline vector<int> getStartLocations() const { return start_locations; }
    inline unordered_map<int, vector<int>> getTaskDependencies() const { return task_dependencies; }
    inline vector<vector<int>> getTaskAssignments() const { return task_assignments; }
    inline int getManhattanDistance(int loc1, int loc2) const
    {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);

        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2) const
    {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

    int getDegree(int loc) const
    {
        assert(loc >= 0 && loc < map_size && !map[loc]);
        int degree = 0;
        if (0 < loc - num_of_cols && !map[loc - num_of_cols])
            degree++;
        if (loc + num_of_cols < map_size && !map[loc + num_of_cols])
            degree++;
        if (loc % num_of_cols > 0 && !map[loc - 1])
            degree++;
        if (loc % num_of_cols < num_of_cols - 1 && !map[loc + 1])
            degree++;
        return degree;
    }

    int getDefaultNumberOfAgents() const { return num_of_agents; }
};
