#include "lns.hpp"
#include "utils.hpp"

bool
isSamePath(const Path& p1, const Path& p2)
{
    if (p1.size() != p2.size())
        return false;
    for (int i = 0; i < (int)p1.size(); i++)
        if (p1.path[i].location != p2.path[i].location)
            return false;
    return true;
}

LNS::LNS(int num_of_iterations, const Instance& instance, int neighbor_size, double time_limit, const string& combo_flag) // added combo flag
  : num_of_iterations(num_of_iterations)
  , neighbor_size(neighbor_size)
  , instance(instance)
  , solution(instance)
  , previous_solution(instance)
  , time_limit(time_limit)
  , combo_flag(combo_flag) // adding combination flag
{
    planner_start_time = Time::now();
}

bool
LNS::run()
{
    // assign tasks
    greedy_task_assignment(&instance, &solution);
    for (int agent = 0; agent < instance.getAgentNum(); agent++) {
        vector<int> task_locations = instance.getTaskLocations(solution.getAgentGlobalTasks(agent));
        solution.agents[agent].path_planner->setGoalLocations(task_locations);
        solution.agents[agent].task_paths.resize(solution.getAssignedTaskSize(agent), Path());
        solution.agents[agent].path_planner->compute_heuristics();
    }

    // compute the precedence constraints based on current task assignments
    // intra agent precedence constraints
    for (int agent = 0; agent < instance.getAgentNum(); agent++)
        for (int task = 1; task < solution.getAssignedTaskSize(agent); task++)
            solution.insertPrecedenceConstraint(solution.task_assignments[agent][task - 1],
                                                solution.task_assignments[agent][task]);
    // inter agent precedence constraints - this should remain same across all
    for (int task = 0; task < instance.getTasksNum(); task++) {
        vector<int> previous_tasks = instance.getTaskDependencies()[task];
        for (int pt : previous_tasks)
            solution.insertPrecedenceConstraint(pt, task);
    }

    // find paths based on the task assignments
    // first we need to sort the tasks based on the precedence constraints
    vector<int> planning_order;
    bool success = topological_sort(&instance, &solution.precedence_constraints, planning_order);
    if (!success) {
        PLOGE << "Topological sorting failed\n";
        return success;
    }

    // following the topological order we find the paths for each task
    initial_paths.resize(instance.getTasksNum(), Path());
    solution.paths.resize(instance.getTasksNum(), Path());
    for (int id : planning_order) {

        int agent = solution.getAgentWithTask(id), task = id,
            task_position = solution.getLocalTaskIndex(agent, task), start_time = 0;
        if (task_position != 0) {
            int previous_task = solution.task_assignments[agent][task_position - 1];
            assert(!initial_paths[previous_task].empty());
            start_time = initial_paths[previous_task].end_time();
        }

        PLOGI << "Planning for agent " << agent << " and task " << task << endl;

        ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
        build_constraint_table(constraint_table, task);
        initial_paths[id] = solution.agents[agent].path_planner->findPathSegment(
          constraint_table, start_time, task_position, 0);
        if (initial_paths[id].empty()) {
            PLOGE << "No path exists for agent " << agent << " and task " << task << endl;
            return false;
        }

        solution.agents[agent].task_paths[task_position] = initial_paths[id];
        solution.paths[id] = initial_paths[id];
    }

    // join the individual task paths to form the agent's path
    vector<int> agents_to_compute(instance.getAgentNum());
    std::iota(agents_to_compute.begin(), agents_to_compute.end(), 0);
    solution.joinPaths(agents_to_compute);

    // gather the information
    int initial_sum_of_costs = 0;
    for (int agent = 0; agent < instance.getAgentNum(); agent++)
        initial_sum_of_costs += solution.agents[agent].path.end_time();
    sum_of_costs = initial_sum_of_costs;
    solution.sum_of_costs = sum_of_costs;

    printPaths();

    initial_solution_runtime = ((fsec)(Time::now() - planner_start_time)).count();
    iteration_stats.emplace_back(initial_solution_runtime,
                                 "greedy",
                                 instance.getAgentNum(),
                                 instance.getTasksNum(),
                                 initial_sum_of_costs);
    runtime = initial_solution_runtime;

    PLOGD << "Initial solution cost = " << initial_sum_of_costs
          << ", Runtime = " << initial_solution_runtime << endl;

    // LNS loop
    while (runtime < time_limit && (int)iteration_stats.size() <= num_of_iterations) {

        runtime = ((fsec)(Time::now() - planner_start_time)).count();

        // extract the set of conflicting tasks
        set<int> conflicted_tasks;
        bool valid = validateSolution(&conflicted_tasks);

        if (valid) {
            PLOGV << "Solution was found!\n";
            break;
        } else {

            PLOGE << "The initial solution was not valid!\n";
            solution.neighbor.conflicted_tasks = conflicted_tasks;
            previous_solution = solution;
            // need to add a flag here
            if (combo_flag == "original" || combo_flag == "online" || combo_flag == "single"){
                IslandprepareNextIteration(); // changed from prepNextIteration to support island logic
                PLOGW << "Printing paths after prep next step"; // delete this if not using
                printPaths(); // delete this if not using
            }

            // Compute regret for each of the tasks that are in the conflicting set
            // Pick the best one and repeat the whole process aboe
            if (combo_flag == "original")
            {
                /*
                IDEA 1: This is the original work from MAPF-LNS-PC repo
                */
                while (!solution.neighbor.conflicted_tasks.empty()) {
                    computeRegret();
                    Regret best_regret = solution.neighbor.regret_max_heap.top();
                    // Use the best regret task and insert it in its correct location
                    commitBestRegretTask(best_regret);
                }
            }
            else if (combo_flag == "online")
            {
                /*
                IDEA 2: This introduces the concept of grouping conflicted tasks together
                */


                //TODO: optimization tip MAKE ISLANDS during validation solution check


                // Step 1. Break up the set into mini islands
                // TODO: merge both island creation and order generation
                vector<set<int>> islands = solution.getConflictIslands(&instance);
                // Step 2. Find topological order for each island
                vector<pair<vector<int>, vector<TemporalOrder>>> islands_in_order; // save the order for use later
                for(set<int> s : islands)
                {
                    pair<vector<int>, vector<TemporalOrder>> islandPackage = solution.getIslandOrder(s, &instance);
                    islands_in_order.push_back(islandPackage);
                    PLOGW <<"Temporal Order in the island";
                    for (int elem: islandPackage.first)
                    {
                        PLOGW << elem;
                    }
                }
                PLOGW << "Number of islands found = " << islands.size();

                // Step 3. Compute the best regret for each island
                while (!solution.neighbor.conflicted_tasks.empty())
                {                    
                    // Setup for storing metatask regret for each island
                    vector<int> compare_regret_sums; // store each island's regret sum
                    vector<vector<pair<int,Regret>>> island_regrets;

                    // Loop over each island
                    for(pair<vector<int>, vector<TemporalOrder>> packet : islands_in_order)
                    {
                        // TODO: Have a way of doing combination for the tasks in order when the island keeps failing
                        
                        /* 
                        A. Setup a copy solution object that stores the "current solution" methods for 
                            continuous update of tasks within this island
                        */
                        CopySolution cp_soln(solution.paths, solution.agents, solution.task_assignments, solution.precedence_constraints, solution.neighbor.conflicted_tasks_path_size);
                        // NOTE: the change of solution.path to have same timestamps as agent.task_path has occured in prepNextIter

                        // Store the cancelled positions of a task for different agents and task_positions that didn't work out
                        unordered_map<int, vector<pair<int,int>>> cancelled_positions; // dictionary task -> vector [ (pair -> agent and task position) ]
                        // Setup the cancelled positions map
                        for(int t: packet.first)
                        {
                            vector<pair<int,int>> temp;
                            cancelled_positions.insert(make_pair(t, temp));
                        }

                        // Setup for island loop
                        vector<int> island = packet.first; // Get island tasks in order
                        vector<TemporalOrder> local =  packet.second; // Get the TemporalOrder object for each task

                        vector<pair<int, Regret>> save_regrets; // save each task and its regret
                        int IslandSums = 0;
                        bool move_next = false; // to help stay on the same island if regrets don't work out

                        /* 
                        B. Loop to check if the current island finds regrets for its tasks that have a lower combined distance
                            than they did before, if yes move on to next island, if no repeat the island by choosing some other regret 
                        */
                        while(!move_next)
                        {
                            /* 
                            B1. Loop over the island in its temporal order, that is, parents will be selected first
                            */
                            for (int t: island)
                            {
                                TemporalOrder* curr;
                                for (auto it = local.begin(); it != local.end(); it++) // get this task's struct
                                {
                                    int index = distance(local.begin(), it);
                                    TemporalOrder o = local[index];
                                    if (o.task == t)
                                    {
                                        curr = &(local[index]); // using the address of the current task to add changes to it
                                        break;
                                    }
                                }

                                /* B1.1 For current task in for loop check if it has any predecessors 
                                */

                                int earlyT = -100000; // variable that stores a garbage start time of parents
                                if (curr->predecessors.size() != 0) // this is looking for parents that exist
                                {
                                    for (int anc: curr->predecessors) // find the earliest time for it
                                    {
                                        for (auto o : local)
                                        {
                                            if (o.task == anc)
                                            {
                                                if (o.task_time > earlyT)
                                                {
                                                    earlyT = o.task_time; // max of the anc time is the earliest time that becomes constraint
                                                }
                                            }
                                        }
                                    }
                                }

                                /* 
                                B1.2 compute its regret based on the earilest time, if no parents then garbage value is passed
                                */ 

                                // changed compute regret function to take in parameters
                                OnlinecomputeRegret(t, earlyT, &cp_soln, cancelled_positions);
                                Regret best_regret = solution.neighbor.regret_max_heap.top();
                                IslandSums += best_regret.value; // store the task's best regret values
                                save_regrets.push_back(make_pair(t, best_regret)); // store the task's best regret
                                // save its time in its TemporalOrder struct, so that its children can call this time
                                curr->task_time = best_regret.endtime; // using begin_time  + task path size to compute this is in insertTask()
                                // Print the task chosen agent, chosen task position, its new end time
                                PLOGD << "Task = " << t <<" Agent = " << best_regret.agent << " Task Pos = " << best_regret.task_position << " End Time = " << curr->task_time;

                                /* 
                                B1.3 Update the CopySolution object now with the details above so that the next task is aware of the current
                                    task's position and time, precedence constraints
                                */

                                // need to insert it at the exact location
                                vector<int> * task_assign_vec = &(cp_soln.task_assign_refs[best_regret.agent]);
                                task_assign_vec->insert(task_assign_vec->begin() + best_regret.task_position, t); // updating task assignment with agent
                                // update the task_path_refs cp_soln with task time.
                                Path* path_ptr = &(cp_soln.task_paths_refs[t]);
                                *path_ptr = best_regret.saved_path_task;
                                vector<int> tasks_for_affected_agent = cp_soln.task_assign_refs[best_regret.agent];

                                // update the precedence constraints
                                vector<pair<int,int>>* prec_constr = &(cp_soln.precedence_refs);
                                if(best_regret.task_position > 0 && best_regret.task_position < (int)cp_soln.task_assign_refs[best_regret.task_position].size())
                                {
                                    int prev_task = tasks_for_affected_agent[best_regret.task_position-1];
                                    int next_task = tasks_for_affected_agent[best_regret.task_position+1];
                                    prec_constr->erase(std::remove_if(prec_constr->begin(),
                                                                            prec_constr->end(),
                                                                            [prev_task, next_task](pair<int, int> x) {
                                                                                return x.first == prev_task &&
                                                                                        x.second == next_task;
                                                                            }),
                                                            prec_constr->end());
                                    prec_constr->push_back(make_pair(prev_task, best_regret.task));
                                    prec_constr->push_back(make_pair(best_regret.task, next_task));

                                }
                                else if(best_regret.task_position == 0)
                                {
                                    int next_task = tasks_for_affected_agent[best_regret.task_position+1];
                                    prec_constr->push_back(make_pair(best_regret.task, next_task));
                                }
                                else if(best_regret.task_position == (int)cp_soln.task_assign_refs[best_regret.agent].size()-1) // because we already inserted task in task_assign
                                {
                                    int prev_task = tasks_for_affected_agent[best_regret.task_position-1];
                                    prec_constr->push_back(make_pair(prev_task, best_regret.task));
                                }

                                // clear agent path timestamps to fill in again
                                vector<int>* agent_ref_path_times = &(cp_soln.agent_refs[best_regret.agent].path.timestamps);
                                agent_ref_path_times->clear();

                                // fill in the agent path timestamps
                                for(int j = 0; j < (int)tasks_for_affected_agent.size(); j++)
                                {
                                    if(j > best_regret.task_position)
                                    {
                                        int prev_task = tasks_for_affected_agent[j-1];
                                        // PLOGI << "prev task when updating " << prev_task;
                                        int affected_task = tasks_for_affected_agent[j];
                                        // PLOGI << "affected task = " << affected_task;
                                        // PLOGI <<"prev task end time = " << cp_soln.task_paths_refs[prev_task].end_time();
                                        cp_soln.task_paths_refs[affected_task].begin_time = cp_soln.task_paths_refs[prev_task].end_time();
                                        // PLOGI <<"new end time of affected task = " << cp_soln.task_paths_refs[affected_task].end_time();
                                    }
                                    // fixing all paths
                                    agent_ref_path_times->push_back(cp_soln.task_paths_refs[tasks_for_affected_agent[j]].end_time());
                                }

                                vector<Path>* agent_ref_tp_vec = &(cp_soln.agent_refs[best_regret.agent].task_paths);
                                agent_ref_tp_vec->insert(agent_ref_tp_vec->begin() + best_regret.task_position, best_regret.saved_path_task);
                                // vector<int>* agent_time_vec = &(cp_soln.agent_refs[best_regret.agent].path.timestamps);
                                // agent_time_vec->insert(agent_time_vec->begin() + best_regret.task_position, cp_soln.task_paths_refs[best_regret.task].end_time()); 

                                // PLOGW << "Task path size after update in agent ref " <<cp_soln.agent_refs[best_regret.agent].task_paths.size();
                                PLOGI <<"Task assign for agent " << best_regret.agent << " is " << cp_soln.task_assign_refs[best_regret.agent].size();
                                PLOGI <<"Path size for task " << best_regret.task << " is " << cp_soln.task_paths_refs[best_regret.task].size();
                                // PLOGW <<"Previous task end time " << cp_soln.agent_refs[best_regret.agent].task_paths[best_regret.task_position - 1].end_time() ;
                                PLOGI <<"Task new begin time " << cp_soln.task_paths_refs[best_regret.task].begin_time << " and task new end time " << cp_soln.task_paths_refs[best_regret.task].end_time();
                                PLOGI << "Agent new task sequence end time " <<cp_soln.agent_refs[best_regret.agent].path.timestamps[cp_soln.agent_refs[best_regret.agent].path.timestamps.size()-1];
                                
                            }

                            /* 
                            B2. Compute distance change for the island and then invoke the same island again if fail...
                            */

                            // B2.1 check if all the regrets have been computed, that is the number of tasks in this island

                            int old_sum = 0;
                            for(int old: island)
                            {
                                for(auto n : cp_soln.conflicted_pathsize_ref)
                                {
                                    if(n.first == old)
                                    {
                                        old_sum += n.second; // to only include this island's distances;
                                    }
                                }
                            }
                            int new_sum = 0;
                            for(auto n: cp_soln.new_distances)
                            {
                                new_sum += n.second;
                            }
                            PLOGD <<"Island new distance sum = " << new_sum;
                            PLOGD <<"Island old distance sum = " << old_sum;

                            // B2.2 if things don't work, we need to reset cp_soln but not lose information about regrets

                            if(new_sum > old_sum)
                            {
                                // this island regret configuration did not work
                                // reset cp_soln
                                cp_soln = CopySolution(solution.paths, solution.agents, solution.task_assignments, solution.precedence_constraints, solution.neighbor.conflicted_tasks_path_size);
                                // add the regret to cancelled positions map
                                //TODO: need to somehow implement a better combination strategy to loop through regrets
                                for(pair<int, Regret> reg : save_regrets)
                                {
                                    if(island[0] == reg.first)
                                    {
                                        PLOGW << "Cancelled this set of configuration, going again!";
                                        cancelled_positions[island[0]].push_back(make_pair(reg.second.agent, reg.second.task_position));
                                    }
                                    
                                }
                                // clear island sum
                                IslandSums = 0;

                                // clear save regrets
                                save_regrets.clear();

                            }
                            else
                            {
                                move_next = true; // everything is fine with this island move to next island
                            }

                        } // while loop scope breaks if move_next = true
                        
                        /* 
                        C. Save the regrets and their values
                        */

                        compare_regret_sums.push_back(IslandSums);
                        island_regrets.push_back(save_regrets);

                    } // end of scope of for loop over islands


                    /* 
                    D. Compute the best island to commit
                    */

                    // find the island with the max regret sum
                    auto max_it = std::max_element(compare_regret_sums.begin(), compare_regret_sums.end());
                    int max_index = distance(compare_regret_sums.begin(), max_it);
                    PLOGI << "Max regret value = " << compare_regret_sums[max_index] << " For Island = " << max_index;
                    // find the corresponding island in original island vector
                    pair<vector<int>, vector<TemporalOrder>> packet = islands_in_order[max_index];
                    vector<int> island = packet.first;
                    vector<pair<int,Regret>> saved_regrets = island_regrets[max_index];

                    /*
                    E. Commit the best island tasks in order
                    */
                    for (int task: island)
                    {
                        for(pair<int,Regret> p: saved_regrets)
                        {
                            if (p.first == task)
                            {
                                // Use the best regret task and insert it in its correct location
                                commitBestRegretTask(p.second);
                                PLOGI << "Removing from conflict set task = " << task;
                                
                                solution.neighbor.conflicted_tasks.erase(task);
                                conflicted_tasks.erase(task);
                            }
                        }
                    }
                    PLOGW << "Size of solution neighbor CT = " << solution.neighbor.conflicted_tasks.size();
                    PLOGW << "Size of local CT = " << conflicted_tasks.size();


                    /*
                    F. Delete the island
                    */                 
                    islands_in_order.erase(islands_in_order.begin() + max_index);
                    
                } // online while loop scope end  -> checking for empty conflict set

            } // online flag end scope


            // join the individual paths that were found for each agent
            for (int i = 0; i < instance.getAgentNum(); i++)
                solution.agents[i].path = Path();

            vector<int> agents_to_compute(instance.getAgentNum());
            std::iota(agents_to_compute.begin(), agents_to_compute.end(), 0);
            solution.joinPaths(agents_to_compute);

            printPaths();
            // compute the updated sum of costs
            solution.sum_of_costs = 0;
            for (int agent = 0; agent < instance.getAgentNum(); agent++) {
                solution.sum_of_costs += solution.agents[agent].path.end_time();
            }

            PLOGD << "Old sum of costs = " << previous_solution.sum_of_costs << endl;
            PLOGD << "New sum of costs = " << solution.sum_of_costs << endl;

            // Accept the solution only if the new one has lower number of conflicts or it has lower
            // cost of the solution
            conflicted_tasks.clear();
            valid = validateSolution(&conflicted_tasks);

            PLOGD << "Number of conflicts in old solution: "
                  << previous_solution.neighbor.conflicted_tasks.size() << endl;
            PLOGD << "Number of conflicts in new solution: " << conflicted_tasks.size() << endl;

            if (previous_solution.neighbor.conflicted_tasks.size() < conflicted_tasks.size()) {
                // reject this solution
                for (int agent = 0; agent < instance.getAgentNum(); agent++)
                    delete solution.agents[agent].path_planner;
                solution = previous_solution;
                PLOGD << "Rejecting this solution!\n";
            } else if (previous_solution.neighbor.conflicted_tasks.size() ==
                       conflicted_tasks.size()) {
                if (previous_solution.sum_of_costs < solution.sum_of_costs) {
                    // reject this solution
                    for (int agent = 0; agent < instance.getAgentNum(); agent++)
                        delete solution.agents[agent].path_planner;
                    solution = previous_solution;
                    PLOGD << "Rejecting this solution!\n";
                }
            } else {
                // accept this solution
                for (int agent = 0; agent < instance.getAgentNum(); agent++)
                    delete previous_solution.agents[agent].path_planner;
            }

            // for debug only
            /* assert(false); */
        }
    }

    PLOGV << "MAPF-PC-LNS: "
          << "\n\tRuntime = " << runtime << "\n\tIterations = " << iteration_stats.size()
          << "\n\tSolution Cost = " << sum_of_costs
          << "\n\tInitial Solution Cost = " << initial_sum_of_costs
          << "\n\tNumber of failures = " << num_of_failures << endl;

    // change this later
    return true;
}

void
LNS::prepareNextIteration()
{
    // remove the conflicted tasks from the agents paths and recompile their paths
    PLOGI << "Preparing the solution object for next iteration\n";

    for (int agent = 0; agent < instance.getAgentNum(); agent++) {
        solution.agents[agent].path_planner = new MultiLabelSpaceTimeAStar(instance, agent);
        vector<int> task_locations = instance.getTaskLocations(solution.getAgentGlobalTasks(agent));
        solution.agents[agent].path_planner->setGoalLocations(task_locations);
        solution.agents[agent].path_planner->compute_heuristics();
    }

    vector<int> tasks_to_fix; // if t_id is deleted then t_id + 1 task needs to be fixed
    unordered_map<int, int> affected_agents, global_to_local_task_id;
    for (int ct : solution.neighbor.conflicted_tasks) {
        PLOGI << "Conflict task prep = " << ct; // todo: DLETE WANTED TO CHECK FOR DEBUG
        int agent = solution.getAgentWithTask(ct),
            ct_position = solution.getLocalTaskIndex(agent, ct),
            path_size = (int)solution.paths[ct].size(); // path_size is used for heuristic estimate
        PLOGD << "Conflicting task: " << ct << ", Agent: " << agent << endl;

        // if the conflicted task was not the last local task of this agent then t_id + 1 exists
        if (ct_position != solution.getAssignedTaskSize(agent) - 1) {
            int next_task = solution.getAgentGlobalTasks(agent, ct_position + 1);
            tasks_to_fix.push_back(next_task);
            path_size += solution.paths[next_task].size();
            PLOGD << "Next task: " << next_task << endl;
        }

        solution.paths[ct] = Path();
        affected_agents.insert(make_pair(ct, agent));
        global_to_local_task_id.insert(make_pair(ct, ct_position));
        solution.neighbor.conflicted_tasks_path_size.insert(make_pair(ct, path_size));

        // marking past information about this conflicting task
        solution.clearIntraAgentPrecedenceConstraint(ct);
        // needs to happen after clearing precedence constraints
        solution.task_assignments[agent][ct_position] = -1;
        solution.agents[agent].task_paths[ct_position] = Path();
    }

    vector<int> planning_order;
    assert(topological_sort(&instance, &solution.precedence_constraints, planning_order));

    // marking past information about conflicting tasks
    for (pair<int, int> task_agent : affected_agents) {
        int agent = task_agent.second;

        // for an affected agent there can be multiple conflicting tasks so need to do it this way
        solution.agents[agent].path = Path();
        solution.task_assignments[agent].erase(
          std::remove_if(solution.task_assignments[agent].begin(),
                         solution.task_assignments[agent].end(),
                         [](int task) { return task == -1; }),
          solution.task_assignments[agent].end());
        solution.agents[agent].task_paths.erase(
          std::remove_if(solution.agents[agent].task_paths.begin(),
                         solution.agents[agent].task_paths.end(),
                         [](Path p) { return isSamePath(p, Path()); }),
          solution.agents[agent].task_paths.end());

        vector<int> task_locations = instance.getTaskLocations(solution.getAgentGlobalTasks(agent));
        solution.agents[agent].path_planner->setGoalLocations(task_locations);
        solution.agents[agent].path_planner->compute_heuristics();
    }

    // find the paths for the tasks whose previous tasks were removed
    for (int task : planning_order) {
        if (std::find(tasks_to_fix.begin(), tasks_to_fix.end(), task) != tasks_to_fix.end()) {

            PLOGD << "Going to find path for next task: " << task << endl;

            int start_time = 0, agent = solution.getAgentWithTask(task);
            int task_position = solution.getLocalTaskIndex(agent, task);


            if (task_position != 0){ // THIS IS CAUSING THE ISSUE OF COMBINATION WHEN TASK REMOVED IS IN FRONT?
                start_time = solution.agents[agent].task_paths[task_position - 1].end_time();}
            else{
                solution.paths.at(task).begin_time = 0; // does this help? // FIXME: Enabling solution path class to be updated as well
            }
            assert(task_position <= solution.getAssignedTaskSize(agent) - 1);

            ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
            build_constraint_table(constraint_table, task);
            solution.paths[task] = solution.agents[agent].path_planner->findPathSegment(
              constraint_table, start_time, task_position, 0);
            solution.agents[agent].task_paths[task_position] = solution.paths[task];

            // Once the path was found fix the begin times for subsequent tasks of the agent
            for (int k = task_position + 1; k < solution.getAssignedTaskSize(agent); k++){
                solution.agents[agent].task_paths[k].begin_time =
                  solution.agents[agent].task_paths[k - 1].end_time();
                int task = solution.getAgentGlobalTasks(agent, k); // FIXME: getting information about task at position k
                int pred = solution.getAgentGlobalTasks(agent, k-1); // FIXME: getting infomration about task at position k-1
                solution.paths.at(task).begin_time = solution.paths.at(pred).end_time(); // did this help? FIXME: enforcing the times to be same
            }
        }
    }

    vector<int> agents_to_compute;
    for (pair<int, int> task_agent : affected_agents)
        agents_to_compute.push_back(task_agent.second);
    solution.joinPaths(agents_to_compute);
}

void
LNS::computeRegret()
{
    solution.neighbor.regret_max_heap.clear();
    for (int task : solution.neighbor.conflicted_tasks)
        computeRegretForTask(task);
}

void
LNS::computeRegretForTask(int task)
{

    pairing_heap<Utility, compare<Utility::compare_node>> service_times;
    // vector<Utility> combo_service_times; // changing to a vector so its easier to use with new format.
    // Find the precedence constraints involving the task or any other task that is not in the
    // conflicting set
    vector<pair<int, int>> precedence_constraints;
    for (pair<int, int> pc : solution.precedence_constraints) {
        if (pc.first == task || pc.second == task ||
            (std::find_if(solution.neighbor.conflicted_tasks.begin(),
                          solution.neighbor.conflicted_tasks.end(),
                          [pc](int task) { return pc.first == task || pc.second == task; })) ==
              solution.neighbor.conflicted_tasks.end())
            precedence_constraints.push_back(pc);
    }

    // compute the set of tasks that are needed to complete before we can complete this task
    vector<int> successors;
    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : precedence_constraints) {
        if (precedence_constraint.first == task)
            successors.push_back(precedence_constraint.second);
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);
    }

    stack<int> q({ task });
    unordered_set<int> previous_tasks;
    int earliest_timestep = 0, latest_timestep = INT_MAX;
    PLOGD << "Finding earliest timestep for task " << task << endl;
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (previous_tasks.find(current) != previous_tasks.end())
            continue;
        previous_tasks.insert(current);
        if (current != task && !solution.paths[current].empty()) {
            PLOGD << "current = " << current << endl;
            int agent = solution.getAgentWithTask(current);
            int task_idx = solution.getLocalTaskIndex(agent, current);
            if (earliest_timestep < solution.agents[agent].path.timestamps[task_idx]) {
                PLOGD << "Going to update earliest timestep from " << earliest_timestep << endl;
                earliest_timestep = solution.agents[agent].path.timestamps[task_idx];
                PLOGD << "New earliest timestep " << earliest_timestep << endl;
                PLOGD << "Came from " << agent << ", " << current << " at " << task_idx << endl;
            }
        }
        for (int agent_task_ancestor : ancestors[current])
            if (previous_tasks.find(agent_task_ancestor) == previous_tasks.end())
                q.push(agent_task_ancestor);
    }
    previous_tasks.erase(task);

    PLOGD << "Finding latest timestep for task " << task << endl;
    for (int succ : successors) {
        if (solution.paths[succ].empty())
            continue;
        int agent = solution.getAgentWithTask(succ);
        int task_idx = solution.getLocalTaskIndex(agent, succ);
        if (latest_timestep > solution.agents[agent].path.timestamps[task_idx]) {
            PLOGD << "Going to update latest timestep from " << latest_timestep << endl;
            latest_timestep = solution.agents[agent].path.timestamps[task_idx];
            PLOGD << "New latest timestep " << latest_timestep << endl;
            PLOGD << "Came from " << agent << ", " << succ << " at " << task_idx << endl;
        }
    }

    if (combo_flag == "original" || combo_flag == "online" || combo_flag == "single")
    {
        for (int agent = 0; agent < instance.getAgentNum(); agent++){
            computeRegretForTaskWithAgent(
            task, agent, earliest_timestep, latest_timestep, &precedence_constraints, &service_times);}

        Utility best_utility = service_times.top();
        service_times.pop();

        if (!service_times.empty()){
            Utility second_best_utility = service_times.top();
            Regret regret(task,
                        best_utility.agent,
                        best_utility.task_position,
                        best_utility.endtime,
                        second_best_utility.value - best_utility.value,
                        best_utility.saved_path_task);
            solution.neighbor.regret_max_heap.push(regret);
        }
        else{
            Regret regret(task,
                        best_utility.agent,
                        best_utility.task_position,
                        best_utility.endtime,
                        best_utility.value,
                        best_utility.saved_path_task); // WHAT should the second value be?
            solution.neighbor.regret_max_heap.push(regret);
        }
    }
}

void
LNS::computeRegretForTaskWithAgent(
  int task,
  int agent,
  int earliest_timestep,
  int latest_timestep,
  vector<pair<int, int>>* precedence_constraints,
  pairing_heap<Utility, compare<Utility::compare_node>>* service_times)
{

    // compute the first position along the agent's task assignments where we can insert this task
    int first_valid_position = 0, last_valid_position = solution.getAssignedTaskSize(agent) + 1;
    vector<int> agent_tasks = solution.getAgentGlobalTasks(agent);
    for (int j = solution.getAssignedTaskSize(agent) - 1; j >= 0; j--) {
        if (solution.agents[agent].path.timestamps[j] <= earliest_timestep) {
            first_valid_position = j + 1;
            break;
        }
    }
    // compute the last position along the agent's task assignment where we can insert this task
    for (int j = 1; j < solution.getAssignedTaskSize(agent); j++) {
        if (solution.agents[agent].path.timestamps[j] >= latest_timestep) {
            last_valid_position = j - 1;
            break;
        }
    }
    // PLOGW << "valid position start " << first_valid_position << " valid position end " << last_valid_position << " agent " << agent;
    assert(first_valid_position >= 0);
    assert(last_valid_position <= solution.getAssignedTaskSize(agent) + 1);

    for (int j = first_valid_position; j < last_valid_position; j++) {

        // compute the distance estimate it would take to finish the insertion
        int distance = 0;
        if (j > 0 && j < solution.getAssignedTaskSize(agent)) {
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(solution.getAgentGlobalTasks(agent, j - 1)),
              instance.getTaskLocations(task));
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(task),
              instance.getTaskLocations(solution.getAgentGlobalTasks(agent, j)));
        } else if (j == 0){
            distance += instance.getManhattanDistance(
              solution.agents[agent].path_planner->start_location, instance.getTaskLocations(task));}
        else{
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(solution.getAgentGlobalTasks(agent, j - 1)),
              instance.getTaskLocations(task));}

        // if the computed distance estimated is longer than the original path size then why bother
        if (distance > solution.neighbor.conflicted_tasks_path_size[task])
            continue;

        if (j != 0) // asserting that the earliest time bounds be met
        {
            if (solution.agents[agent].path.timestamps[j-1] < earliest_timestep) 
                continue;
        }
        if (j <= solution.getAssignedTaskSize(agent)-1) // asserting that latest time bounds are met
        {
            if (solution.agents[agent].path.timestamps[j+1] > latest_timestep) 
                continue;            
        }

        // PLOGW << "Regret based latest timestamp = " << latest_timestep;
        // PLOGW << "Chosen begin timestamp = " << solution.agents[agent].path.timestamps[j-1] << " for pos " << j << " and agent " << agent;
        vector<Path> task_paths = solution.paths;
        vector<vector<int>> task_assignments = solution.task_assignments;
        vector<pair<int, int>> prec_constraints = *precedence_constraints;
        Utility utility =
          insertTask(task, agent, j, &task_paths, &task_assignments, &prec_constraints, distance);
        service_times->push(utility);
    }
}

Utility
LNS::insertTask(int task,
                int agent,
                int task_position,
                vector<Path>* task_paths,
                vector<vector<int>>* task_assignments,
                vector<pair<int, int>>* precedence_constraints,
                bool commit)
{

    double path_size_change = 0;
    int start_time = 0, previous_task = -1, next_task = -1;

    vector<Path>& task_paths_ref = *task_paths;
    vector<vector<int>>& task_assignments_ref = *task_assignments;
    vector<pair<int, int>>& prec_constraints_ref = *precedence_constraints;

    int agent_tasks_size = (int)task_assignments_ref[agent].size();

    // in this case we are inserting a task between two existing tasks
    if (task_position > 0 && task_position < agent_tasks_size) {

        previous_task = task_assignments_ref[agent][task_position - 1];
        next_task = task_assignments_ref[agent][task_position];

        start_time = task_paths_ref[previous_task].end_time();
        path_size_change = (double)task_paths_ref[next_task].size();

        task_paths_ref[next_task] = Path(); // this path will change

        task_assignments_ref[agent].insert(task_assignments_ref[agent].begin() + task_position,
                                           task);
        prec_constraints_ref.erase(std::remove_if(prec_constraints_ref.begin(),
                                                  prec_constraints_ref.end(),
                                                  [previous_task, next_task](pair<int, int> x) {
                                                      return x.first == previous_task &&
                                                             x.second == next_task;
                                                  }),
                                   prec_constraints_ref.end());
        prec_constraints_ref.push_back(make_pair(previous_task, task));
        prec_constraints_ref.push_back(make_pair(task, next_task));

        if (commit) {
            solution.agents[agent].task_paths[task_position] = Path();
            solution.agents[agent].task_paths.insert(
              solution.agents[agent].task_paths.begin() + task_position, Path());
        }

    } else if (task_position == 0) { // in this case we are inserting at the very start

        next_task = task_assignments_ref[agent][task_position];
        path_size_change = (double)task_paths_ref[next_task].size();

        task_paths_ref[next_task] = Path();

        task_assignments_ref[agent].insert(task_assignments_ref[agent].begin() + task_position,
                                           task);
        prec_constraints_ref.push_back(make_pair(task, next_task));

        if (commit) {
            solution.agents[agent].task_paths[task_position] = Path();
            solution.agents[agent].task_paths.insert(
              solution.agents[agent].task_paths.begin() + task_position, Path());
        }

    } else if (task_position == agent_tasks_size) { // in this case we are inserting at the very end

        previous_task = task_assignments_ref[agent][task_position - 1];
        start_time = task_paths_ref[previous_task].end_time();

        task_assignments_ref[agent].push_back(task);

        prec_constraints_ref.push_back(make_pair(previous_task, task));

        if (commit)
            solution.agents[agent].task_paths.push_back(Path());
    }

    vector<int> goal_locations = instance.getTaskLocations(task_assignments_ref[agent]);
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
    if (!commit) {
        MultiLabelSpaceTimeAStar local_planner = MultiLabelSpaceTimeAStar(instance, agent);
        local_planner.setGoalLocations(goal_locations);
        local_planner.compute_heuristics();
        if (combo_flag == "original"){
            build_constraint_table(constraint_table,
                                task,
                                instance.getTaskLocations(task),
                                task_paths,
                                task_assignments,
                                precedence_constraints);
        }
        if (combo_flag == "online" || combo_flag == "single"){ // using a different online build constraint table
            Onlinebuild_constraint_table(constraint_table,
                                task,
                                instance.getTaskLocations(task),
                                task_paths,
                                task_assignments,
                                precedence_constraints);
        }
        task_paths_ref[task] =
          local_planner.findPathSegment(constraint_table, start_time, task_position, 0);

        if (next_task != -1) {
            start_time = task_paths_ref[task].end_time();
            if (combo_flag == "original"){
            build_constraint_table(constraint_table,
                                   next_task,
                                   instance.getTaskLocations()[next_task],
                                   task_paths,
                                   task_assignments,
                                   precedence_constraints);
            }
            if (combo_flag == "online" || combo_flag == "single"){ // using a different online build constraint table
                Onlinebuild_constraint_table(constraint_table,
                                task,
                                instance.getTaskLocations(task),
                                task_paths,
                                task_assignments,
                                precedence_constraints);               
            }
            task_paths_ref[next_task] =
              local_planner.findPathSegment(constraint_table, start_time, task_position + 1, 0);
        }
    } else {

        solution.agents[agent].path_planner->setGoalLocations(goal_locations);
        solution.agents[agent].path_planner->compute_heuristics();

        build_constraint_table(constraint_table, task);
        task_paths_ref[task] = solution.agents[agent].path_planner->findPathSegment(
          constraint_table, start_time, task_position, 0);
        solution.agents[agent].task_paths[task_position] = task_paths_ref[task];


        if (next_task != -1) {
            start_time = task_paths_ref[task].end_time();

            build_constraint_table(constraint_table, next_task);
            task_paths_ref[next_task] = solution.agents[agent].path_planner->findPathSegment(
              constraint_table, start_time, task_position + 1, 0);
            solution.agents[agent].task_paths[task_position + 1] = task_paths_ref[next_task];
        }

        for (int k = task_position + 1; k < solution.getAssignedTaskSize(agent); k++)
            solution.agents[agent].task_paths[k].begin_time =
              solution.agents[agent].task_paths[k - 1].end_time();
    }

    if (!commit) {
        double value = -path_size_change + (double)task_paths_ref[task].size();
        if (next_task != -1)
            value += (double)task_paths_ref[next_task].size();

        int end_time = task_paths_ref[task].begin_time + task_paths_ref[task].size() - 1; // to account for when task is at last location
        Path temp = task_paths_ref[task];
        Utility utility(agent, task_position, task, end_time, value, temp); // adding end time for limiting successor time
        return utility;
    } else
        return Utility();
}

void
LNS::commitBestRegretTask(Regret best_regret)
{

    PLOGD << "Committing for task " << best_regret.task << " to agent " << best_regret.agent
          << " with regret = " << best_regret.value << " and position = " << best_regret.task_position<< endl;
    insertTask(best_regret.task,
               best_regret.agent,
               best_regret.task_position,
               &solution.paths,
               &solution.task_assignments,
               &solution.precedence_constraints,
               true);
    solution.neighbor.conflicted_tasks.erase(best_regret.task);
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table,
                            int task,
                            int task_location,
                            vector<Path>* paths,
                            vector<vector<int>>* task_assignments,
                            vector<pair<int, int>>* precedence_constraints)
{
    constraint_table.goal_location = task_location;

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : (*precedence_constraints))
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (set_of_tasks_to_complete.find(current) != set_of_tasks_to_complete.end())
            continue;
        set_of_tasks_to_complete.insert(current);
        for (int agent_task_ancestor : ancestors[current])
            if (set_of_tasks_to_complete.find(agent_task_ancestor) ==
                set_of_tasks_to_complete.end())
                q.push(agent_task_ancestor);
    }
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        assert(!(*paths)[id].empty());
        int task_position = -1, task_set_size = -1;
        for (int i = 0; i < instance.getAgentNum(); i++) {
            for (int j = 0; j < (int)(*task_assignments)[i].size(); j++) {
                if ((*task_assignments)[i][j] == id) {
                    task_position = j;
                    task_set_size = (int)(*task_assignments)[i].size();
                    break;
                }
            }
        }
        bool wait_at_goal = task_position == task_set_size - 1;
        constraint_table.addPath((*paths)[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!(*paths)[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, (*paths)[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

void
LNS::build_constraint_table(ConstraintTable& constraint_table, int task)
{
    constraint_table.goal_location = instance.getTaskLocations(task);

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());

    // only care about tasks that are not in conflict right now
    for (pair<int, int> precedence_constraint : solution.precedence_constraints) {
        if (std::find_if(solution.neighbor.conflicted_tasks.begin(),
                         solution.neighbor.conflicted_tasks.end(),
                         [precedence_constraint](int task) {
                             return precedence_constraint.first == task ||
                                    precedence_constraint.second == task;
                         }) != solution.neighbor.conflicted_tasks.end())
            continue;
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);
    }

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (set_of_tasks_to_complete.find(current) != set_of_tasks_to_complete.end())
            continue;
        set_of_tasks_to_complete.insert(current);
        for (int agent_task_ancestor : ancestors[current])
            if (set_of_tasks_to_complete.find(agent_task_ancestor) ==
                set_of_tasks_to_complete.end())
                q.push(agent_task_ancestor);
    }
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        int agent = solution.getAgentWithTask(id);
        int task_position = solution.getLocalTaskIndex(agent, id);
        bool wait_at_goal = task_position == (int)solution.getAssignedTaskSize(agent) - 1;
        constraint_table.addPath(solution.paths[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!solution.paths[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, solution.paths[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}

bool
LNS::validateSolution(set<int>* conflicted_tasks)
{

    bool result = true;

    // // Check that the precedence constraints are not violated
    // for (pair<int, int> precedence_constraint : solution.precedence_constraints) {

    //     // only care about tasks that are not in conflict
    //     if (std::find_if(solution.neighbor.conflicted_tasks.begin(),
    //                      solution.neighbor.conflicted_tasks.end(),
    //                      [precedence_constraint](int task) {
    //                          return precedence_constraint.first == task ||
    //                                 precedence_constraint.second == task;
    //                      }) != solution.neighbor.conflicted_tasks.end())
    //         continue;

    //     int agent_a = solution.getAgentWithTask(precedence_constraint.first),
    //         agent_b = solution.getAgentWithTask(precedence_constraint.second);
    //     int task_position_a = solution.getLocalTaskIndex(agent_a, precedence_constraint.first),
    //         task_position_b = solution.getLocalTaskIndex(agent_b, precedence_constraint.second);

    //     if (solution.agents[agent_a].path.timestamps[task_position_a] >=
    //         solution.agents[agent_b].path.timestamps[task_position_b]) {
    //         PLOGE << "Temporal conflict between agent " << agent_a << " doing task "
    //               << precedence_constraint.first << " and agent " << agent_b << " doing task "
    //               << precedence_constraint.second << endl;
    //         result = false;
    //         if (conflicted_tasks == nullptr)
    //             return false;
    //         else {
    //             conflicted_tasks->insert(precedence_constraint.first);
    //             conflicted_tasks->insert(precedence_constraint.second);
    //         }
    //     }
    // }

    unordered_map<int, vector<int>> task_ancestor = instance.getTaskDependencies();
    for (const auto & [task, ancestors] : task_ancestor)
    {
        int task_agent = solution.getAgentWithTask(task);
        int task_local_pos = solution.getLocalTaskIndex(task_agent, task);
        int task_time = solution.agents[task_agent].path.timestamps[task_local_pos];
        for(int anc: ancestors)
        {
            int anc_agent = solution.getAgentWithTask(anc);
            int anc_local_pos = solution.getLocalTaskIndex(anc_agent, anc);
            int anc_time = solution.agents[anc_agent].path.timestamps[anc_local_pos];
            if (task_time <= anc_time)
            {
                PLOGE << "Temporal conflict between agent " << anc_agent << " doing ancestor task "<< anc << " and agent "<< task_agent << " doing successor task " << task;
                result = false;
                if (conflicted_tasks == nullptr)
                    return false;
                else {
                    conflicted_tasks->insert(task);
                    conflicted_tasks->insert(anc);
                }
            }
        }
    }

    for (int agent_i = 0; agent_i < instance.getAgentNum(); agent_i++)
        for (int agent_j = 0; agent_j < instance.getAgentNum(); agent_j++) {
            if (agent_i == agent_j)
                continue;
            size_t min_path_length =
              solution.agents[agent_i].path.size() < solution.agents[agent_j].path.size()
                ? solution.agents[agent_i].path.size()
                : solution.agents[agent_j].path.size();
            for (int timestep = 0; timestep < (int)min_path_length; timestep++) {
                int location_agent_i = solution.agents[agent_i].path.at(timestep).location;
                int location_agent_j = solution.agents[agent_j].path.at(timestep).location;

                // Check that any two agents are not at the same location at the same timestep
                if (location_agent_i == location_agent_j) {
                    pair<int, int> coord = instance.getCoordinate(location_agent_i);
                    PLOGE << "Agents " << agent_i << " and " << agent_j
                          << " collide with each other at (" << coord.first << ", " << coord.second
                          << ") at timestep " << timestep << endl;
                    result = false;
                    if (conflicted_tasks == nullptr)
                        return false;
                    else {
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_i);
                             task_idx++)
                            if (solution.agents[agent_i].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_i, task_idx));
                                break;
                            }
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_j);
                             task_idx++)
                            if (solution.agents[agent_j].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_j, task_idx));
                                break;
                            }
                    }
                }
                // Check that any two agents are not following the same edge in the opposite
                // direction at the same timestep
                else if (timestep < (int)min_path_length - 1 &&
                         location_agent_i ==
                           solution.agents[agent_j].path.at(timestep + 1).location &&
                         location_agent_j ==
                           solution.agents[agent_i].path.at(timestep + 1).location) {
                    pair<int, int> coord_i = instance.getCoordinate(location_agent_i),
                                   coord_j = instance.getCoordinate(location_agent_j);
                    PLOGE << "Agents " << agent_i << " and " << agent_j
                          << " collide with each other at (" << coord_i.first << ", "
                          << coord_i.second << ") --> (" << coord_j.first << ", " << coord_j.second
                          << ") at timestep " << timestep << endl;
                    result = false;
                    if (conflicted_tasks == nullptr)
                        return false;
                    else {
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_i);
                             task_idx++)
                            if (solution.agents[agent_i].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_i, task_idx));
                                break;
                            }
                        for (int task_idx = 0; task_idx < solution.getAssignedTaskSize(agent_j);
                             task_idx++)
                            if (solution.agents[agent_j].path.timestamps[task_idx] > timestep) {
                                conflicted_tasks->insert(
                                  solution.getAgentGlobalTasks(agent_j, task_idx));
                                break;
                            }
                    }
                }
            }

            // Check that any two agents are not at the same location at the same timestep where
            // one agent might be waiting already
            if (solution.agents[agent_i].path.size() != solution.agents[agent_j].path.size()) {
                int smaller_path_agent =
                  solution.agents[agent_i].path.size() < solution.agents[agent_j].path.size()
                    ? agent_i
                    : agent_j;
                int larger_path_agent =
                  solution.agents[agent_i].path.size() < solution.agents[agent_j].path.size()
                    ? agent_j
                    : agent_i;
                int last_location_of_smaller_path_agent =
                  solution.agents[smaller_path_agent].path.back().location;
                for (int timestep = (int)min_path_length;
                     timestep < (int)solution.agents[larger_path_agent].path.size();
                     timestep++) {
                    int location_of_larger_path_agent =
                      solution.agents[larger_path_agent].path.at(timestep).location;
                    if (last_location_of_smaller_path_agent == location_of_larger_path_agent) {
                        pair<int, int> coord =
                          instance.getCoordinate(last_location_of_smaller_path_agent);
                        PLOGE << "Agents " << agent_i << " and " << agent_j
                              << " collide with each other at (" << coord.first << ", "
                              << coord.second << ") at timestep " << timestep << endl;
                        result = false;
                        if (conflicted_tasks == nullptr)
                            return false;
                        else {
                            if (solution.agents[agent_i]
                                  .path.timestamps[solution.getAssignedTaskSize(agent_i) - 1] <
                                timestep) {
                                conflicted_tasks->insert(solution.getAgentGlobalTasks(
                                  agent_i, solution.getAssignedTaskSize(agent_i) - 1));
                            } else {
                                for (int task_idx = 0;
                                     task_idx < solution.getAssignedTaskSize(agent_i);
                                     task_idx++)
                                    if (solution.agents[agent_i].path.timestamps[task_idx] >
                                        timestep) {
                                        conflicted_tasks->insert(
                                          solution.getAgentGlobalTasks(agent_i, task_idx));
                                        break;
                                    }
                            }
                            if (solution.agents[agent_j]
                                  .path.timestamps[solution.getAssignedTaskSize(agent_j) - 1] <
                                timestep) {
                                conflicted_tasks->insert(solution.getAgentGlobalTasks(
                                  agent_j, solution.getAssignedTaskSize(agent_j) - 1));
                            } else {
                                for (int task_idx = 0;
                                     task_idx < solution.getAssignedTaskSize(agent_j);
                                     task_idx++)
                                    if (solution.agents[agent_j].path.timestamps[task_idx] >
                                        timestep) {
                                        conflicted_tasks->insert(
                                          solution.getAgentGlobalTasks(agent_j, task_idx));
                                        break;
                                    }
                            }
                        }
                    }
                }
            }
        }
    return result;
}

void
LNS::printPaths() const
{
    for (int i = 0; i < instance.getAgentNum(); i++) {
        cout << "Agent " << i << " (cost = " << solution.agents[i].path.size() - 1 << "): ";
        cout << "\n\tPaths:\n\t";
        for (int t = 0; t < (int)solution.agents[i].path.size(); t++) {
            pair<int, int> coord = instance.getCoordinate(solution.agents[i].path.at(t).location);
            cout << "(" << coord.first << ", " << coord.second << ")@" << t;
            if (solution.agents[i].path.at(t).is_goal)
                cout << "*";
            if (i != (int)solution.agents[i].path.size() - 1)
                cout << " -> ";
        }
        cout << endl;
        cout << "\tTimestamps:\n\t";
        for (int j = 0; j < (int)solution.getAssignedTaskSize(i); j++) {
            pair<int, int> goal_coord =
              instance.getCoordinate(solution.agents[i].path_planner->goal_locations[j]);
            cout << "(" << goal_coord.first << ", " << goal_coord.second << ")@"
                 << solution.agents[i].path.timestamps[j];
            if (j != solution.getAssignedTaskSize(i) - 1)
                cout << " -> ";
        }
        cout << endl;
        cout << "\tTasks:\n\t";
        for (int j = 0; j < solution.getAssignedTaskSize(i); j++) {
            cout << solution.getAgentGlobalTasks(i)[j];
            if (j != solution.getAssignedTaskSize(i) - 1)
                cout << " -> ";
        }
        cout << endl;
    }
}

// TODO: Hosting changes to original functions below. where new things begin, try and make this more streamlined or into its own file
/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
THE FUNCTION BELOW ARE FOR THE USE OF ONLINE BASED SOLUTION APPROACH
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
*/

// new online compute regret function
void
LNS::OnlinecomputeRegret(int task, int earlyT, CopySolution* cp_soln, unordered_map<int, vector<pair<int,int>>> cancelled_positions)
{
    solution.neighbor.regret_max_heap.clear();
    OnlinecomputeRegretForTask(task, earlyT, cp_soln, cancelled_positions); // sending in cancelled positions, copy solutions, earlyT
}

// new online compute regret for task function that uses copysolution methods instead of methods of solution class
void
LNS::OnlinecomputeRegretForTask(int task, int earlyT, CopySolution* cp_soln, unordered_map<int, vector<pair<int,int>>> cancelled_positions)
{

    pairing_heap<Utility, compare<Utility::compare_node>> service_times;
    // 1. make a vector that stores all distances, with agent and task position pair, so that when 
    // best regret is found you can find that distance easily and push it to CopySolution vector
    
    vector<pair<pair<int,int>,int>> distances; //this needs to go into regretwTask() and get populated

    // vector<Utility> combo_service_times; // changing to a vector so its easier to use with new format.
    // Find the precedence constraints involving the task or any other task that is not in the
    // conflicting set

    vector<pair<int, int>> precedence_constraints;
    for (pair<int, int> pc : cp_soln->precedence_refs) {
        if (pc.first == task || pc.second == task ||
            (std::find_if(solution.neighbor.conflicted_tasks.begin(),
                          solution.neighbor.conflicted_tasks.end(),
                          [pc](int task) { return pc.first == task || pc.second == task; })) ==
              solution.neighbor.conflicted_tasks.end())
            precedence_constraints.push_back(pc);
    }

    // compute the set of tasks that are needed to complete before we can complete this task
    vector<int> successors;
    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum());
    for (pair<int, int> precedence_constraint : precedence_constraints) {
        if (precedence_constraint.first == task)
            successors.push_back(precedence_constraint.second);
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);
    }

    // esnure you have added the global temporal dependencies for the task which get missed above in precedence constraints
    unordered_map<int,vector<int>> task_depen = instance.getTaskDependencies();
    vector<int> ancestor_list = task_depen[task];
    for(int anc: ancestor_list)
    {
        ancestors[task].push_back(anc);
    }

    stack<int> q({ task });
    unordered_set<int> previous_tasks;
    int earliest_timestep = 0, latest_timestep = INT_MAX;
    PLOGD << "Finding earliest timestep for task " << task << endl;
    PLOGD << "Early time of ancestor = " << earlyT;
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (previous_tasks.find(current) != previous_tasks.end())
            continue;
        previous_tasks.insert(current);
        if (current != task && !solution.paths[current].empty()) {
            PLOGD << "current = " << current << endl;
            int agent = solution.getAgentWithTask(current); // this will not have conflicted task information
            int task_idx = solution.getLocalTaskIndex(agent, current);
            if (earliest_timestep < solution.agents[agent].path.timestamps[task_idx]) {
                PLOGD << "Going to update earliest timestep from " << earliest_timestep << endl;
                earliest_timestep = solution.agents[agent].path.timestamps[task_idx];
                PLOGD << "New earliest timestep " << earliest_timestep << endl;
                PLOGD << "Came from " << agent << ", " << current << " at " << task_idx << endl;
            }
        }
        for (int agent_task_ancestor : ancestors[current])
            if (previous_tasks.find(agent_task_ancestor) == previous_tasks.end())
                q.push(agent_task_ancestor);
    }
    previous_tasks.erase(task);
    if (earlyT > earliest_timestep)
    {
        PLOGD << "New earliest_timestep = " << earlyT << " came from task's ancestor";
        earliest_timestep = earlyT;
    }

    PLOGD << "Finding latest timestep for task " << task << endl;
    for (int succ : successors) {
        if (solution.paths[succ].empty())
            continue;
        int agent = solution.getAgentWithTask(succ);
        int task_idx = solution.getLocalTaskIndex(agent, succ);
        if (latest_timestep > solution.agents[agent].path.timestamps[task_idx]) {
            PLOGD << "Going to update latest timestep from " << latest_timestep << endl;
            latest_timestep = solution.agents[agent].path.timestamps[task_idx];
            PLOGD << "New latest timestep " << latest_timestep << endl;
            PLOGD << "Came from " << agent << ", " << succ << " at " << task_idx << endl;
        }
    }
    if (latest_timestep < earliest_timestep)
    {
        PLOGW << "Task " << task <<" has latest_timestep lower than earlier, subsituting with max";
        latest_timestep = INT_MAX;
    }

    if (combo_flag == "original" || combo_flag == "online" || combo_flag == "single")
    {
        for (int agent = 0; agent < instance.getAgentNum(); agent++){
            OnlinecomputeRegretForTaskWithAgent(
            task, agent, earliest_timestep, latest_timestep, &precedence_constraints, &service_times, cp_soln, cancelled_positions, &distances);} // TODO: send in both cancelled positions and distance vector

        Utility best_utility = service_times.top();
        service_times.pop();

        int a = best_utility.agent;
        int tp = best_utility.task_position;
        for(pair<pair<int,int>,int> dist : distances)
        {
            pair<int,int> pack = dist.first;
            if(pack.first == a)
            {
                if(pack.second == tp)
                {
                    cp_soln->new_distances[task] = dist.second; // this works?
                }
            }
        }

        if (!service_times.empty()){
            Utility second_best_utility = service_times.top();
            Regret regret(task,
                        best_utility.agent,
                        best_utility.task_position,
                        best_utility.endtime,
                        second_best_utility.value - best_utility.value,
                        best_utility.saved_path_task);
            solution.neighbor.regret_max_heap.push(regret);
        }
        else{ // employing a condition which does not look for a second best regret, if one does not exist, and sends back its own value
            Regret regret(task,
                        best_utility.agent,
                        best_utility.task_position,
                        best_utility.endtime,
                        best_utility.value,
                        best_utility.saved_path_task); // WHAT should the second value be?
            solution.neighbor.regret_max_heap.push(regret);
        }
    }
}


// new online compute regret for task with agent function
// Allow a task path virutal copy, (similar to that in InserTask) to get here
// Compare current task against that (so that it has all predecessor paths in there)
// Save distance accumulated (will use it in the end)
// Check against cancelled task map to "continue" that selection
// Everything below needs to use CP SOLN variables
// TODO: Can't handle an empty agent task sequence
void
LNS::OnlinecomputeRegretForTaskWithAgent(
  int task,
  int agent,
  int earliest_timestep,
  int latest_timestep,
  vector<pair<int, int>>* precedence_constraints,
  pairing_heap<Utility, compare<Utility::compare_node>>* service_times,
  CopySolution* cp_soln,
  unordered_map<int, vector<pair<int,int>>> cancelled_positions,
  vector<pair<pair<int,int>,int>>* distances)
{
    
    //compute the first position along the agent's task assignments where we can insert this task
    int first_valid_position = 0, last_valid_position = cp_soln->task_assign_refs[agent].size();
    // PLOGW << "Agent = " << agent << " first pos " << first_valid_position << " last pos " << last_valid_position;
    vector<int> agent_tasks = cp_soln->task_assign_refs[agent];
    for (int j = cp_soln->task_assign_refs[agent].size() - 1; j >= 0; j--) {
        if (cp_soln->agent_refs[agent].path.timestamps[j] <= earliest_timestep) {
            // PLOGI << "first valid time "  <<cp_soln->agent_refs[agent].path.timestamps[j];
            first_valid_position = j + 1;
            break;
        }
    }
    // compute the last position along the agent's task assignment where we can insert this task
    for (int j = 1; j < (int)cp_soln->task_assign_refs[agent].size(); j++) {
        if (cp_soln->agent_refs[agent].path.timestamps[j] >= latest_timestep) {
            // PLOGI << "last valid time "  <<cp_soln->agent_refs[agent].path.timestamps[j];
            last_valid_position = j - 1;
            break;
        }
    }    

    assert(first_valid_position >= 0);
    // assert(last_valid_position <= solution.getAssignedTaskSize(agent) + 1);
    assert(last_valid_position <= (int)cp_soln->task_assign_refs[agent].size() + 1);

    for (int j = first_valid_position; j <= last_valid_position; j++) { // had to make j <= last_valid position since, first = last is possible?


        // compute the distance estimate it would take to finish the insertion
        int distance = 0;
        if (j > 0 && j < (int)cp_soln->task_assign_refs[agent].size()) {
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(cp_soln->task_assign_refs[agent][j-1]),
              instance.getTaskLocations(task));
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(task),
              instance.getTaskLocations(cp_soln->task_assign_refs[agent][j]));
        } else if (j == 0){
            distance += instance.getManhattanDistance(
              cp_soln->agent_refs[agent].path_planner->start_location, instance.getTaskLocations(task));}
        else{
            distance += instance.getManhattanDistance(
              instance.getTaskLocations(cp_soln->task_assign_refs[agent][j-1]),
              instance.getTaskLocations(task));}

        // FIXME: removed this because we are doing something different
        // if the computed distance estimated is longer than the original path size then why bother
        // if (distance > solution.neighbor.conflicted_tasks_path_size[task]) 
        //     continue;

        if (j != 0) // asserting that the earliest time bounds be met
        {
            if (cp_soln->agent_refs[agent].path.timestamps[j-1] < earliest_timestep)
                continue;
        }
        // if (j <= cp_soln->task_assign_refs[agent].size()-1) // asserting that latest time bounds are met
        // {
        //     if (cp_soln->agent_refs[agent].path.timestamps[j+1] > latest_timestep) // FIXME: this is not checking what you want it to check temporally
        //         continue;            
        // }

        // if agent and task position for this task in cancelled positions, "continue"
        bool cancel_flag = false;
        if(cancelled_positions.find(task) != cancelled_positions.end())
        {
            for(pair<int,int> cancel : cancelled_positions[task])
            {
                if(cancel.first == agent)
                {
                    if(cancel.second == j)
                    {
                        cancel_flag = true;
                        PLOGW << "Part of cancel group";
                        break;
                    }
                }

            }
        }

        if(cancel_flag)
            continue;

        // save the distance in the vector for this task and position or whatever is there
        distances->push_back(make_pair(make_pair(agent, j), distance));

        vector<Path> task_paths = cp_soln->task_paths_refs;
        vector<vector<int>> task_assignments = cp_soln->task_assign_refs;
        vector<pair<int, int>> prec_constraints = cp_soln->precedence_refs;

        Utility utility = insertTask(task, agent, j, &task_paths, &task_assignments, &prec_constraints);
        service_times->push(utility);
    }
}

// new online build constraint table method for using in insert task
void
LNS::Onlinebuild_constraint_table(ConstraintTable& constraint_table,
                            int task,
                            int task_location,
                            vector<Path>* paths,
                            vector<vector<int>>* task_assignments,
                            vector<pair<int, int>>* precedence_constraints)
{
    constraint_table.goal_location = task_location;

    vector<vector<int>> ancestors;
    ancestors.resize(instance.getTasksNum()); // TODO change this to ignore reading conflict set of tasks
    // for (pair<int, int> precedence_constraint : (*precedence_constraints))
    //     ancestors[precedence_constraint.second].push_back(precedence_constraint.first);

    // doing this to only care about tasks that are not in conflict right now
    for (pair<int, int> precedence_constraint : solution.precedence_constraints) {
        if (std::find_if(solution.neighbor.conflicted_tasks.begin(),
                         solution.neighbor.conflicted_tasks.end(),
                         [precedence_constraint](int task) {
                             return precedence_constraint.first == task ||
                                    precedence_constraint.second == task;
                         }) != solution.neighbor.conflicted_tasks.end())
            continue;
        ancestors[precedence_constraint.second].push_back(precedence_constraint.first);
    }

    unordered_set<int> set_of_tasks_to_complete;
    stack<int> q({ task });
    while (!q.empty()) {
        int current = q.top();
        q.pop();
        if (set_of_tasks_to_complete.find(current) != set_of_tasks_to_complete.end())
            continue;
        set_of_tasks_to_complete.insert(current);
        for (int agent_task_ancestor : ancestors[current])
            if (set_of_tasks_to_complete.find(agent_task_ancestor) ==
                set_of_tasks_to_complete.end())
                q.push(agent_task_ancestor);
    }
    set_of_tasks_to_complete.erase(task);

    for (int id : set_of_tasks_to_complete) {
        assert(!(*paths)[id].empty());
        int task_position = -1, task_set_size = -1;
        for (int i = 0; i < instance.getAgentNum(); i++) {
            for (int j = 0; j < (int)(*task_assignments)[i].size(); j++) {
                if ((*task_assignments)[i][j] == id) {
                    task_position = j;
                    task_set_size = (int)(*task_assignments)[i].size();
                    break;
                }
            }
        }
        bool wait_at_goal = task_position == task_set_size - 1;
        constraint_table.addPath((*paths)[id], wait_at_goal);
    }

    for (int agent_task_ancestor : ancestors[task]) {
        assert(!(*paths)[agent_task_ancestor].empty());
        constraint_table.length_min =
          max(constraint_table.length_min, (*paths)[agent_task_ancestor].end_time() + 1);
    }
    constraint_table.latest_timestep =
      max(constraint_table.latest_timestep, constraint_table.length_min);
}


// new online prepare next iteration
void
LNS::IslandprepareNextIteration()
{
    // remove the conflicted tasks from the agents paths and recompile their paths
    PLOGI << "Preparing the solution object for next iteration\n";

    for (int agent = 0; agent < instance.getAgentNum(); agent++) {
        solution.agents[agent].path_planner = new MultiLabelSpaceTimeAStar(instance, agent);
        vector<int> task_locations = instance.getTaskLocations(solution.getAgentGlobalTasks(agent));
        solution.agents[agent].path_planner->setGoalLocations(task_locations);
        solution.agents[agent].path_planner->compute_heuristics();
    }

    vector<int> tasks_to_fix; // if t_id is deleted then t_id + 1 task needs to be fixed
    unordered_map<int, int> affected_agents, global_to_local_task_id;
    for (int ct : solution.neighbor.conflicted_tasks) {
        PLOGI << "Conflict task prep = " << ct; // todo: DLETE WANTED TO CHECK FOR DEBUG
        int agent = solution.getAgentWithTask(ct),
            ct_position = solution.getLocalTaskIndex(agent, ct),
            path_size = (int)solution.paths[ct].size(); // path_size is used for heuristic estimate
        PLOGD << "Conflicting task: " << ct << ", Agent: " << agent << endl;

        // if the conflicted task was not the last local task of this agent then t_id + 1 exists
        if (ct_position != solution.getAssignedTaskSize(agent) - 1) {
            int next_task = solution.getAgentGlobalTasks(agent, ct_position + 1);
            if (solution.neighbor.conflicted_tasks.find(next_task) == solution.neighbor.conflicted_tasks.end()) // adding condition to check if next task is in ct then ignore
            {
                tasks_to_fix.push_back(next_task);
                path_size += solution.paths[next_task].size();
                PLOGD << "Next task: " << next_task << endl;                
            }
        }
        solution.paths[ct] = Path();
        affected_agents.insert(make_pair(ct, agent));
        global_to_local_task_id.insert(make_pair(ct, ct_position));
        solution.neighbor.conflicted_tasks_path_size.insert(make_pair(ct, path_size));

        // marking past information about this conflicting task
        solution.clearIntraAgentPrecedenceConstraint(ct);
        // needs to happen after clearing precedence constraints
        solution.task_assignments[agent][ct_position] = -1;
        solution.agents[agent].task_paths[ct_position] = Path();
    }

    vector<int> planning_order;
    assert(topological_sort(&instance, &solution.precedence_constraints, planning_order));

    // marking past information about conflicting tasks
    for (pair<int, int> task_agent : affected_agents) {
        int agent = task_agent.second;

        // for an affected agent there can be multiple conflicting tasks so need to do it this way
        solution.agents[agent].path = Path();
        solution.task_assignments[agent].erase(
          std::remove_if(solution.task_assignments[agent].begin(),
                         solution.task_assignments[agent].end(),
                         [](int task) { return task == -1; }),
          solution.task_assignments[agent].end());
        solution.agents[agent].task_paths.erase(
          std::remove_if(solution.agents[agent].task_paths.begin(),
                         solution.agents[agent].task_paths.end(),
                         [](Path p) { return isSamePath(p, Path()); }),
          solution.agents[agent].task_paths.end());

        vector<int> task_locations = instance.getTaskLocations(solution.getAgentGlobalTasks(agent));
        solution.agents[agent].path_planner->setGoalLocations(task_locations);
        solution.agents[agent].path_planner->compute_heuristics();
    }

    // find the paths for the tasks whose previous tasks were removed
    for (int task : planning_order) {
        if (std::find(tasks_to_fix.begin(), tasks_to_fix.end(), task) != tasks_to_fix.end()) {

            PLOGD << "Going to find path for next task: " << task << endl;

            int start_time = 0, agent = solution.getAgentWithTask(task);
            int task_position = solution.getLocalTaskIndex(agent, task);


            if (task_position != 0){ // THIS IS CAUSING THE ISSUE OF COMBINATION WHEN TASK REMOVED IS IN FRONT?
                start_time = solution.agents[agent].task_paths[task_position - 1].end_time();}
            else{
                solution.paths.at(task).begin_time = 0; // does this help? // FIXME: Enabling solution path class to be updated as well
            }
            assert(task_position <= solution.getAssignedTaskSize(agent) - 1);

            ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
            build_constraint_table(constraint_table, task);
            solution.paths[task] = solution.agents[agent].path_planner->findPathSegment(
              constraint_table, start_time, task_position, 0);
            solution.agents[agent].task_paths[task_position] = solution.paths[task];

            // Once the path was found fix the begin times for subsequent tasks of the agent
            for (int k = task_position + 1; k < solution.getAssignedTaskSize(agent); k++){
                solution.agents[agent].task_paths[k].begin_time =
                  solution.agents[agent].task_paths[k - 1].end_time();
                int task = solution.getAgentGlobalTasks(agent, k); // FIXME: getting information about task at position k
                int pred = solution.getAgentGlobalTasks(agent, k-1); // FIXME: getting infomration about task at position k-1
                solution.paths.at(task).begin_time = solution.paths.at(pred).end_time(); // did this help? FIXME: enforcing the times to be same
            }
        }
    }

    set<int> agents_to_compute; // trying for set to resolve the issue of multiple tasks from same agent being in the conflict set
    for (pair<int, int> task_agent : affected_agents)
        agents_to_compute.insert(task_agent.second);
        // agents_to_compute.push_back(task_agent.second);
    solution.OnlinejoinPaths(agents_to_compute); // change the join path algo a bit
}



