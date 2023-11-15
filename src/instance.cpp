#include "instance.hpp"
#include <boost/token_functions.hpp>
#include <boost/tokenizer.hpp>
#include "utils.hpp"

Instance::Instance(const string& mapFname, const string& agentTaskFname,
                   int numOfAgents, int numOfTasks)
    : mapFname_(mapFname),
      agentTaskFname_(agentTaskFname),
      numOfAgents_(numOfAgents),
      numOfTasks_(numOfTasks) {
  bool succ = loadMap();
  if (!succ) {
    PLOGE << "Map file " << mapFname << " not found.\n";
    exit(-1);
  }

  ancestors_.resize(numOfTasks_);
  successors_.resize(numOfTasks_);
  succ = loadAgentsAndTasks();
  if (!succ) {
    PLOGE << "Agent and task file " << agentTaskFname << " not found.\n";
    exit(-1);
  }
  preComputeHeuristics();
}

bool Instance::loadMap() {
  using namespace std;
  using namespace boost;

  ifstream file(mapFname_.c_str());
  if (!file.is_open()) {
    return false;
  }

  // Using custom mapf benchmark for now.
  // Once validated you want to move to the original mapf bechmarks
  string line;
  tokenizer<char_separator<char>>::iterator begin;

  getline(file, line);

  if (line[0] == 't') {
    // Original MAPF benchmarks
    char_separator<char> sep(" ");
    getline(file, line);
    tokenizer<char_separator<char>> tokenizer(line, sep);
    begin = tokenizer.begin();
    begin++;
    numOfRows = atoi((*begin).c_str());  // Read the number of rows / height
    getline(file, line);
    tokenizer.assign(line, sep);
    begin = tokenizer.begin();
    begin++;
    numOfCols = atoi((*begin).c_str());  // Read the number of columns / width
    getline(file, line);                 // Skip the map
  } else {
    // Custom empty benchmark
    char_separator<char> sep(",");
    tokenizer<char_separator<char>> tokenizer(line, sep);
    begin = tokenizer.begin();
    numOfRows = atoi((*begin).c_str());  // Read the number of rows
    begin++;
    numOfCols = atoi((*begin).c_str());  // Read the number of columns
  }

  mapSize = numOfCols * numOfRows;
  map_.resize(mapSize, false);
  for (int i = 0; i < numOfRows; i++) {
    getline(file, line);
    for (int j = 0; j < numOfCols; j++) {
      map_[linearizeCoordinate(i, j)] = (line[j] != '.');
    }
  }
  file.close();
  return true;
}

bool Instance::loadAgentsAndTasks() {
  using namespace std;
  using namespace boost;

  ifstream file(agentTaskFname_.c_str());
  if (!file.is_open()) {
    return false;
  }

  string line;
  char_separator<char> sep(",");
  tokenizer<char_separator<char>>::iterator begin;

  getline(file, line);
  if (numOfAgents_ != atoi(line.c_str())) {
    PLOGE << "The number of robots passed in command line and the agent file "
             "do not match.\n";
    exit(-1);
  }

  if (numOfAgents_ == 0) {
    PLOGE << "The number of agents should be larger than 0.\n";
    exit(-1);
  }

  if (numOfTasks_ == 0) {
    PLOGE << "The number of tasks should be larger than 0.\n";
    exit(-1);
  }

  // Reading the agent start locations
  startLocations_.resize(numOfAgents_);

  for (int i = 0; i < numOfAgents_; i++) {
    getline(file, line);
    tokenizer<char_separator<char>> tokenizer(line, sep);
    begin = tokenizer.begin();
    int col = atoi((*begin).c_str());
    begin++;
    int row = atoi((*begin).c_str());
    startLocations_[i] = linearizeCoordinate(row, col);
    assert(!isObstacle(startLocations_[i]));
  }

  // Skipping the extra white lines
  while (!file.eof() && line[0] != 't') {
    getline(file, line);
  }

  getline(file, line);
  if (numOfTasks_ != atoi(line.c_str())) {
    PLOGE << "The number of tasks passed in the command line and the agent "
             "file do not match.\n";
    exit(-1);
  }

  // Reading the task goal locations
  taskLocations_.resize(numOfTasks_);

  for (int i = 0; i < numOfTasks_; i++) {
    getline(file, line);
    tokenizer<char_separator<char>> tokenizer(line, sep);
    begin = tokenizer.begin();
    int col = atoi((*begin).c_str());
    begin++;
    int row = atoi((*begin).c_str());
    taskLocations_[i] = linearizeCoordinate(row, col);
    assert(!isObstacle(taskLocations_[i]));
  }

  // Skipping the extra white lines
  while (!file.eof() && line[0] != 't') {
    getline(file, line);
  }

  getline(file, line);
  int numDependencies = atoi(line.c_str());
  vector<pair<int, int>> temporalDependencies;

  for (int i = 0; i < numDependencies; i++) {
    getline(file, line);
    tokenizer<char_separator<char>> tokenizer(line, sep);
    begin = tokenizer.begin();
    int predecessor = atoi((*begin).c_str());
    begin++;
    int successor = atoi((*begin).c_str());
    temporalDependencies.emplace_back(predecessor, successor);
  }

  for (pair<int, int> dependency : temporalDependencies) {
    int i, j;
    tie(i, j) = dependency;
    taskDependencies_[j].push_back(i);
    ancestors_[j].push_back(i);
    successors_[i].push_back(j);
    inputPrecedenceConstraints_.emplace_back(i, j);
  }

  PLOGD << "# Agents: " << numOfAgents_ << "\t # Tasks: " << numOfTasks_
        << "\t # Dependencies: " << numDependencies << endl;

  file.close();

  assert(
      topologicalSort(this, &inputPrecedenceConstraints_, inputPlanningOrder_));

  return true;
}

list<int> Instance::getNeighbors(int current) const {
  list<int> neighbors;
  int candidates[4] = {current + 1, current - 1, current + numOfCols,
                       current - numOfCols};
  for (int next : candidates) {
    if (validMove(current, next)) {
      neighbors.emplace_back(next);
    }
  }
  return neighbors;
}

void Instance::printMap() const {
  for (int i = 0; i < numOfRows; i++) {
    for (int j = 0; j < numOfCols; j++) {
      if (map_[linearizeCoordinate(i, j)]) {
        PLOGI << '@';
      } else
        PLOGI << '.';
    }
    PLOGI << endl;
  }
}
