#pragma once
#include "common.hpp"
#include "lns.hpp"

#include <plog/Log.h>

#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>  // stringstream
#include <stack>
#include <string>
#include <utility>
#include <vector>

using std::cin;
using std::cout;
using std::endl;
using std::make_pair;
using std::ofstream;
using std::pair;
using std::queue;
using std::set;
using std::stack;
using std::string;
using std::stringstream;
using std::vector;

#include <boost/tokenizer.hpp>

class SaveToTxt {
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
  string outputFile;
  void fileSave();
  void makeFile();
  void printStart();
  void runData(const Instance* inst, const Solution* sol);
};
