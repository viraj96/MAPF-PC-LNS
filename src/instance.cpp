#include "instance.hpp"
#include <boost/tokenizer.hpp>

Instance::Instance(const string& map_fname,
                   const string& agent_task_fname,
                   int num_of_agents,
                   int num_of_tasks)
  : map_fname(map_fname)
  , agent_task_fname(agent_task_fname)
  , num_of_agents(num_of_agents)
  , num_of_tasks(num_of_tasks)
{
    bool succ = loadMap();
    if (!succ) {

        PLOGE << "Map file " << map_fname << " not found.\n";
        exit(-1);
    }

    succ = loadAgentsAndTasks();
    if (!succ) {

        PLOGE << "Agent and task file " << agent_task_fname << " not found.\n";
        exit(-1);
    }
}

void
Instance::assignTaskToAgent(int agent, int task)
{
    task_assignments[agent].push_back(task);
}

bool
Instance::loadMap()
{
    using namespace std;
    using namespace boost;

    ifstream file(map_fname.c_str());
    if (!file.is_open())
        return false;

    // using custom mapf benchmark for now. Once validated you want to move to the original mapf
    // bechmarks
    string line;
    char_separator<char> sep(",");
    tokenizer<char_separator<char>>::iterator begin;

    getline(file, line);
    tokenizer<char_separator<char>> tokenizer(line, sep);
    begin = tokenizer.begin();
    num_of_rows = atoi((*begin).c_str()); // read the number of rows
    begin++;
    num_of_cols = atoi((*begin).c_str()); // read the number of columns

    map_size = num_of_cols * num_of_rows;
    map.resize(map_size, false);
    for (int i = 0; i < num_of_rows; i++) {
        getline(file, line);
        for (int j = 0; j < num_of_cols; j++)
            map[linearizeCoordinate(i, j)] = (line[j] != '.');
    }
    file.close();
    return true;
}

bool
Instance::loadAgentsAndTasks()
{
    using namespace std;
    using namespace boost;

    ifstream file(agent_task_fname.c_str());
    if (!file.is_open())
        return false;

    string line;
    char_separator<char> sep(",");
    tokenizer<char_separator<char>>::iterator begin;

    getline(file, line);
    if (num_of_agents != atoi(line.c_str())) {
        PLOGE << "The number of robots passed in command line and the agent file do not match.\n";
        exit(-1);
    }

    if (num_of_agents == 0) {
        PLOGE << "The number of agents should be larger than 0.\n";
        exit(-1);
    }

    if (num_of_tasks == 0) {
        PLOGE << "The number of tasks should be larger than 0.\n";
        exit(-1);
    }

    // Reading the agent start locations
    start_locations.resize(num_of_agents);
    task_assignments.resize(num_of_agents);

    for (int i = 0; i < num_of_agents; i++) {
        getline(file, line);
        tokenizer<char_separator<char>> tokenizer(line, sep);
        begin = tokenizer.begin();
        int col = atoi((*begin).c_str());
        begin++;
        int row = atoi((*begin).c_str());
        start_locations[i] = linearizeCoordinate(row, col);
        assert(!isObstacle(start_locations[i]));
    }

    // Skipping the extra white lines
    while (!file.eof() && line[0] != 't')
        getline(file, line);

    getline(file, line);
    if (num_of_tasks != atoi(line.c_str())) {
        PLOGE
          << "The number of tasks passed in the command line and the agent file do not match.\n";
        exit(-1);
    }

    // Reading the task goal locations
    task_locations.resize(num_of_tasks);

    for (int i = 0; i < num_of_tasks; i++) {
        getline(file, line);
        tokenizer<char_separator<char>> tokenizer(line, sep);
        begin = tokenizer.begin();
        int col = atoi((*begin).c_str());
        begin++;
        int row = atoi((*begin).c_str());
        task_locations[i] = linearizeCoordinate(row, col);
        assert(!isObstacle(task_locations[i]));
    }

    // Skipping the extra white lines
    while (!file.eof() && line[0] != 't')
        getline(file, line);

    getline(file, line);
    int num_dependencies = atoi(line.c_str());
    vector<pair<int, int>> temporal_dependencies;

    for (int i = 0; i < num_dependencies; i++) {
        getline(file, line);
        tokenizer<char_separator<char>> tokenizer(line, sep);
        begin = tokenizer.begin();
        int predecessor = atoi((*begin).c_str());
        begin++;
        int successor = atoi((*begin).c_str());
        temporal_dependencies.push_back(make_pair(predecessor, successor));
    }

    for (pair<int, int> dependency : temporal_dependencies) {
        int i, j;
        tie(i, j) = dependency;
        task_dependencies[j].push_back(i);
    }

    PLOGD << "# Agents: " << num_of_agents << "\t # Tasks: " << num_of_tasks
          << "\t # Dependencies: " << num_dependencies << endl;

    file.close();
    return true;
}

list<int>
Instance::getNeighbors(int current) const
{
    list<int> neighbors;
    int candidates[4] = { current + 1, current - 1, current + num_of_cols, current - num_of_cols };
    for (int next : candidates)
        if (validMove(current, next))
            neighbors.emplace_back(next);
    return neighbors;
}
