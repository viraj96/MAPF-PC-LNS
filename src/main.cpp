#include <plog/Log.h>
#include <plog/Severity.h>
#include "plog/Appenders/ColorConsoleAppender.h"
#include "plog/Formatters/TxtFormatter.h"
#include "plog/Initializers/ConsoleInitializer.h"

#include <boost/program_options.hpp>
#include "lns.hpp"
#include "common.hpp"
#include "costchecker.hpp"
#include "instance.hpp"
#include "utils.hpp"

int main(int argc, char** argv) {

  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(plog::error, &consoleAppender);

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()("help", "Produce help message");
  desc.add_options()("map,m", po::value<string>()->required(),
                     "Input file for map");
  desc.add_options()("agents,a", po::value<string>()->required(),
                     "Input file for agents");
  desc.add_options()("output,o", po::value<string>(),
                     "Output file for schedule");
  desc.add_options()("cutoffTime,t", po::value<double>()->default_value(7200),
                     "Cutoff time (seconds)");
  desc.add_options()("agentNum,k", po::value<int>()->default_value(0),
                     "Number of agents to plan for");
  desc.add_options()("taskNum,l", po::value<int>()->default_value(0),
                     "Number of tasks to plan for");
  desc.add_options()("neighborSize,n", po::value<int>()->default_value(8),
                     "Size of the neighborhood");
  desc.add_options()("maxIterations,i", po::value<int>()->default_value(0),
                     "Maximum number of iterations");
  desc.add_options()("severity,d", po::value<int>()->default_value(0),
                     "Debugging level");
  desc.add_options()("initialSolution,s", po::value<string>(),
                     "Strategy for the initial solution");
  desc.add_options()("genReport,g", po::value<bool>()->default_value(false),
                     "Whether to generate the report file that can be fed to "
                     "CBS-PC for verificattion");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help") != 0u) {
    plog::get()->setMaxSeverity(plog::debug);
    PLOGD << desc << endl;
    return 1;
  }

  po::notify(vm);
  plog::get()->setMaxSeverity(
      static_cast<plog::Severity>(vm["severity"].as<int>()));

  string initialSolutionStrategy = vm["initialSolution"].as<string>();
  if (initialSolutionStrategy != "greedy" &&
      initialSolutionStrategy.find("sota") == string::npos) {
    PLOGE << "Incorrect initial solution strategy provided. Please choose from "
             "'greedy', 'sota_cbs' or 'sota_pbs' options"
          << endl;
    return 1;
  }

  // Need to store the seed for debugging
  auto srandSeed = (int)time(nullptr);
  // srandSeed = 1699554871;
  srand(srandSeed);

  Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
                    vm["agentNum"].as<int>(), vm["taskNum"].as<int>());

  for (int i = 0; i < instance.getAgentNum(); i++) {
    pair<int, int> location =
        instance.getCoordinate(instance.getStartLocations()[i]);
    PLOGD << "Agent " << i << " starts at :(" << location.first << ", "
          << location.second << ")\n";
  }
  for (int i = 0; i < instance.getTasksNum(); i++) {
    pair<int, int> location =
        instance.getCoordinate(instance.getTaskLocations()[i]);
    PLOGD << "Task " << i << " starts at :(" << location.first << ", "
          << location.second << ")\n";
  }
  for (pair<int, vector<int>> dependencies : instance.getTaskDependencies()) {
    PLOGD << "Task  " << dependencies.first
          << " has the following dependent tasks\n";
    for (int task : dependencies.second) {
      PLOGD << "\t Task : " << task << "\n";
    }
  }

  LNS lnsInstance =
      LNS(vm["maxIterations"].as<int>(), instance, vm["neighborSize"].as<int>(),
          vm["cutoffTime"].as<double>(), initialSolutionStrategy);
  bool success = lnsInstance.run();

  FeasibleSolution anytimeSolution = lnsInstance.getFeasibleSolution();
  if (!anytimeSolution.agentPaths.empty()) {
    PLOGI << "Anytime solution found!\n";
    std::cout << anytimeSolution.toString() << std::endl;
  }
  else {
    PLOGE << "Anytime solution was not found!\n";
  }

  if (vm["genReport"].as<bool>()) {
    SaveToTxt saveFileForCBSPC;
    saveFileForCBSPC.printStart();
    Solution finalSolution = lnsInstance.getSolution();
    saveFileForCBSPC.runData(&instance, &finalSolution);
    saveFileForCBSPC.fileSave();
  }

  double firstFeasibleSolutionTime = 0;
  if (success) {
    for (const IterationStats& iter : lnsInstance.iterationStats) {
      if (iter.feasibleSolutionFound) {
        firstFeasibleSolutionTime = iter.runtime;
        break;
      }
    }
  }
  else {
    firstFeasibleSolutionTime = INT_MAX;
  }

  std::cout << "MAPF-PC-LNS: "
            << "\n\tRuntime = " << lnsInstance.runtime
            << "\n\tIterations = " << lnsInstance.iterationStats.size()
            << "\n\tFirst Feasible Solution Runtime = " << firstFeasibleSolutionTime
            << "\n\tSolution Cost = " << lnsInstance.getSolution().sumOfCosts
            << "\n\tNumber of failures = " << lnsInstance.numOfFailures
            << "\n\tSuccess = " << success << endl;
}
