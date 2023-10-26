#include "cbs_pc_cost_checker.h"

void SavetoTxt::printStart()
{
    cout<<"Loading task assignment, locations and precedence constraint data"<<endl;
}

void SavetoTxt::filesave()
{
    cout<<"File is saved!" <<endl;
    cout<<"Name of file: " << this->output_file << endl;
}

void SavetoTxt::run_data(const Instance* inst, const Solution* sol)
{
    // open file
    auto now = std::chrono::system_clock::now();
    auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream datetime;
    datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
    // UTC
    // ofstream myFile1("report_" + std::to_string(UTC) + ".txt");
    // DateTime
    string file_name = "report_" + datetime.str() + ".txt";
    std::ofstream myFile(file_name);
    this->output_file = file_name;
    
    if(myFile.is_open())
    {
        // get number of agents
        int agent_num = inst->getAgentNum();
        myFile << agent_num << " # number of agents" <<endl;
        myFile << "# Format:  num_of_goals sx sy g1x g1y g2x g2y ..." << endl;

        // vector<int> agent_locs = inst->getStartLocations();

        // for(auto it = agent_locs.begin(); it != agent_locs.end(); it++)
        // {
        //     myFile << *it << endl;
        // }
        
        // get global task ids for each agent
        for(int a = 0; a < agent_num; a++)
        {
            vector<int> GlobalTasks = sol->getAgentGlobalTasks(a);
            int num_tasks = GlobalTasks.size();
            pair<int,int> start_loc_agent = inst->getCoordinate(sol->agents[a].path.at(0).location);
            myFile << num_tasks << "\t"<< start_loc_agent.second << "\t" << start_loc_agent.first <<"\t";
            for(int i = 0; i < num_tasks; i++)
            {
                pair<int,int> task_loc = inst->getCoordinate(sol->agents[a].pathPlanner->goalLocations.at(i));
                myFile << task_loc.second <<"\t" << task_loc.first << "\t";
            }
            myFile<<""<<endl;
        }

        // write the precedence constraints
        myFile<<"temporal cons:" <<endl;

        vector<pair<int,int>> global_pc = inst->getInputPrecedenceConstraints();
        int pc_size = global_pc.size();

        for(auto pc: global_pc)
        {
            int predecessor = pc.first;
            int successor = pc.second;
            int pred_agent = sol->taskAgentMap.at(predecessor);
            // PLOGD << pred_agent << " task = "  << predecessor;
            int pred_local = sol->getLocalTaskIndex(pred_agent, predecessor);
            //pair<int,int> pred_loc = inst->getCoordinate(sol->agents[pred_agent].pathPlanner->goalLocations.at(pred_local));
            int succ_agent = sol->taskAgentMap.at(successor);
            int succ_local = sol->getLocalTaskIndex(succ_agent, successor);
            //pair<int,int> succ_loc = inst->getCoordinate(sol->agents[succ_agent].pathPlanner->goalLocations.at(succ_local));

            //myFile << pred_loc.second <<"\t" << pred_loc.first <<"\t" << succ_loc.second << "\t" << succ_loc.first<<endl;
            myFile << pred_agent << "\t" << pred_local << "\t" << succ_agent << "\t" << succ_local << endl;
        }
    }
    else{
        cout <<"Failed to create file for some reason";
    } 
}