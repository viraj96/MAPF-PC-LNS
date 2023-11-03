#include "costchecker.hpp"
#include <iomanip>

using namespace std;

void SaveToTxt::printStart() {
  cout << "Loading task assignment, locations and precedence constraint data"
       << endl;
}

void SaveToTxt::fileSave() {
  cout << "File is saved!" << endl;
  cout << "Name of file: " << this->outputFile << endl;
}

void SaveToTxt::runData(const Instance* inst, const Solution* sol) {
  // Open file
  auto now = std::chrono::system_clock::now();

  auto inTimeT = std::chrono::system_clock::to_time_t(now);
  std::stringstream datetime;
  datetime << put_time(std::localtime(&inTimeT), "%Y-%m-%d-%X");
  // DateTime
  string fileName = "report_" + datetime.str() + ".txt";
  std::ofstream myFile(fileName);
  this->outputFile = fileName;

  if (myFile.is_open()) {
    // Get number of agents
    int agentNum = inst->getAgentNum();
    myFile << agentNum << " # number of agents" << endl;
    myFile << "# Format:  num_of_goals sx sy g1x g1y g2x g2y ..." << endl;

    // Get global task ids for each agent
    for (int a = 0; a < agentNum; a++) {
      vector<int> globalTasks = sol->getAgentGlobalTasks(a);
      int numTasks = globalTasks.size();
      pair<int, int> startLocAgent =
          inst->getCoordinate(inst->getStartLocations()[a]);
      myFile << numTasks << "\t" << startLocAgent.second << "\t"
             << startLocAgent.first << "\t";
      for (int i = 0; i < numTasks; i++) {
        pair<int, int> taskLoc = inst->getCoordinate(
            sol->agents[a].pathPlanner->goalLocations.at(i));
        myFile << taskLoc.second << "\t" << taskLoc.first << "\t";
      }
      myFile << "" << endl;
    }

    // Write the precedence constraints
    myFile << "temporal cons:" << endl;

    vector<pair<int, int>> globalPc = inst->getInputPrecedenceConstraints();

    for (auto pc : globalPc) {
      int predecessor = pc.first, successor = pc.second;
      int predAgent = sol->taskAgentMap.at(predecessor),
          predLocalIndex = sol->getLocalTaskIndex(predAgent, predecessor);
      int succAgent = sol->taskAgentMap.at(successor),
          succLocalIndex = sol->getLocalTaskIndex(succAgent, successor);

      myFile << predAgent << "\t" << predLocalIndex << "\t" << succAgent << "\t"
             << succLocalIndex << endl;
    }
  } else {
    cout << "Failed to create file for some reason";
  }
}
