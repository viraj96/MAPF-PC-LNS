#include "plog/Appenders/ColorConsoleAppender.h"
#include "plog/Formatters/TxtFormatter.h"
#include "plog/Initializers/ConsoleInitializer.h"
#include <plog/Log.h>

#include "common.hpp"
#include "instance.hpp"
#include "utils.hpp"
#include <boost/program_options.hpp>

int
main(int argc, char** argv)
{

    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::init(plog::error, &consoleAppender);

    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()("help", "Produce help message");
    desc.add_options()("map,m", po::value<string>()->required(), "Input file for map");
    desc.add_options()("agents,a", po::value<string>()->required(), "Input file for agents");
    desc.add_options()("output,o", po::value<string>(), "Output file for schedule");
    desc.add_options()(
      "cutoffTime,t", po::value<double>()->default_value(7200), "Cutoff time (seconds)");
    desc.add_options()(
      "agentNum,k", po::value<int>()->default_value(0), "Number of agents to plan for");
    desc.add_options()(
      "taskNum,l", po::value<int>()->default_value(0), "Number of tasks to plan for");
    desc.add_options()(
      "neighborSize,n", po::value<int>()->default_value(8), "Size of the neighborhood");
    desc.add_options()(
      "maxIterations,i", po::value<int>()->default_value(0), "Maximum number of iterations");
    desc.add_options()("severity,d", po::value<int>()->default_value(0), "Debugging level");
    // adding another flag for switching to combo solution ->string value (can be changed later to some other format)
    desc.add_options()("combo_sol,c", po::value<string>()->default_value("original"), "Switch to combination solution");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        plog::get()->setMaxSeverity(plog::debug);
        PLOGD << desc << endl;
        return 1;
    }

    po::notify(vm);
    plog::get()->setMaxSeverity(static_cast<plog::Severity>(vm["severity"].as<int>()));

    /* srand((int)time(0)); */
    // srand(5); // initial two conflict fault
    srand(7); // iniital large conflict fault
    // srand(8); // initial large conflict fault
    // srand((int)time(0));

    Instance instance(vm["map"].as<string>(),
                      vm["agents"].as<string>(),
                      vm["agentNum"].as<int>(),
                      vm["taskNum"].as<int>());

    for (int i = 0; i < instance.getAgentNum(); i++) {
         pair<int, int> location = instance.getCoordinate(instance.getStartLocations()[i]);
        PLOGD << "Agent " << i << " starts at :(" << location.first << ", " << location.second
              << ")\n";
    }
    for (int i = 0; i < instance.getTasksNum(); i++) {
        pair<int, int> location = instance.getCoordinate(instance.getTaskLocations()[i]);
        PLOGD << "Task " << i << " starts at :(" << location.first << ", " << location.second
              << ")\n";
    }
    for (pair<int, vector<int>> dependencies : instance.getTaskDependencies()) {
        PLOGD << "Task  " << dependencies.first << " has the following dependent tasks\n";
        for (int task : dependencies.second)
            PLOGD << "\t Task : " << task << "\n";
    }
    /* greedy_task_assignment(&instance); */

    LNS lns_instance = LNS(vm["maxIterations"].as<int>(),
                           instance,
                           vm["neighborSize"].as<int>(),
                           vm["cutoffTime"].as<double>(),
                           vm["combo_sol"].as<string>());
    bool success = lns_instance.run();
    PLOGI << "Success = " << success << endl;
}
