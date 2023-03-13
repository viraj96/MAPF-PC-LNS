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
    vector<int> task_locations, start_locations;
    unordered_map<int, vector<int>> task_dependencies;

    // these values needs to change after goal reasssignments
    vector<vector<int>> task_assignments;
    vector<pair<int, int>> precedence_constraints;

    bool loadMap();
    bool loadAgentsAndTasks();
    void saveMap() const;
    void printMap() const;
    void saveAgents() const;

    bool isConnected(int start, int goal);
    friend class SingleAgentSolver;

  public:
    int map_size, num_of_cols, num_of_rows;

    // these values need to be updated after goal reassignments
    vector<int> id_base;
    vector<pair<int, int>> id_to_agent_task;
    int agent_task_to_id(pair<int, int> agent_task) const
    {
        int agent = agent_task.first, task = agent_task.second;
        return id_base[agent] + task;
    }

    Instance() {}
    Instance(const string& map_fname,
             const string& agent_task_fname,
             int num_of_agents = 0,
             int num_of_tasks = 0);

    void printAgents() const;
    int getAgentWithTask(int task) const;
    void assignTaskToAgent(int agent, int task);
    vector<int> getAgentTasks(int agent) const;
    vector<int> getTaskLocations(vector<int> tasks) const
    {
        vector<int> task_locs(tasks.size(), 0);
        for (int i = 0; i < (int)tasks.size(); i++)
            task_locs[i] = task_locations[tasks[i]];
        return task_locs;
    }
    int getLocalTaskIndex(int agent, int task) const
    {
        for (int i = 0; i < (int)task_assignments[agent].size(); i++) {
            if (task_assignments[agent][i] == task)
                return i;
        }
        return -1;
    }
    inline vector<pair<int, int>> getPrecedenceConstraints() const
    {
        return precedence_constraints;
    }
    inline void insertPrecedenceConstraint(int task_id_a, int task_id_b)
    {
        precedence_constraints.push_back(make_pair(task_id_a, task_id_b));
    }
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

    int getDefaultNumberOfTasks() const { return num_of_tasks; }
    int getDefaultNumberOfAgents() const { return num_of_agents; }
};
