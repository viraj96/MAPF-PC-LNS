#pragma once
#include "lns.hpp"
#include <numeric>
#include "common.hpp"
#include "utils.hpp"

#include <plog/Log.h>
#include <limits>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <utility>
#include <stack>
#include <queue>
#include <string>
#include <chrono>
#include <iomanip> // put_time
#include <sstream> // stringstream

using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::set;
using std::pair;
using std::make_pair;
using std::stack;
using std::queue;
using std::string;
using std::ofstream;
using std::stringstream;

#include<boost/tokenizer.hpp>

class SavetoTxt
{
    /*
    Please add  - 

    1) #include "cbs_pc_cost_checker.h" to lns.cpp @lns.cpp::5

    2) to lns.cpp::281
        SavetoTxt sv;
        sv.printStart();
        sv.run_data(&instance_, &solution_);
        sv.filesave();
    
    Description - 
        Simple function to save the instance data such as agent task assignment, start locations,
        task locations and global precedence constraints in a txt format for ingestion to 
        MAPF-PC code. 

        The text file is saved with a timestamp as a unique identifier. 

        Please copy the text file to the appropriate location in the MAPF-PC codebase.

        The corresponding MAPF-PC command 
        ./bin/cbs -m sample_input/empty-16-16.map -a sample_input/report_2023-10-25-22\:10\:11.txt -s 2 -k 10
    */
    public:
        string output_file;
        void makefile();
        void printStart();
        void run_data(const Instance* inst, const Solution* sol);
        void filesave();
};