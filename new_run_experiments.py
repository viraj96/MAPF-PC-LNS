########################################### IMPORTS ########################################
from subprocess import PIPE, Popen
import argparse
import glob
import time
import os
import datetime
from pathlib import Path


############## BASELINE FUNCTIONS #################

######################################### MAPD-LNS-wPBS ##########################################
def run_mapd(args, scene, logfile):
    '''
    Run the MAPD-LNS-wPBS commands
    
    Input:
        args (argument object)\n
        scene - instance file\n
        logfile - filepath for logfile
    '''

    commands_prog = "./build/lifelong" 
    commands_map = " -m "+ args.map
    commands_task = " --task=" + scene
    commands_args = " -k 10 --simulation_time=1600 -t 120 --solver=PBS --seed=0 --scenario=KIVA --simulation_window=15 --planning_window=15"
    full_command = commands_prog + commands_map + commands_task + commands_args
    run_subprocess(full_command, args, logfile)
    time.sleep(0.75) # adding some buffer time between runs



######################################### MAPF-PC ##########################################
def run_mapf_pc(args, scene, logfile):
    '''
    Run the MAPF-PC commands
    
    Input:
        args (argument object)\n
        scene - instance file\n
        logfile - filepath for logfile
    '''

    commands_prog = "./bin/task_assignment" 
    commands_map = " -m "+ args.map
    commands_agent = " -a " + scene
    commands_args = " -k 30 -t 2 --solver CBS"
    full_command = commands_prog + commands_map + commands_agent + commands_args
    run_subprocess(full_command, args, logfile)
    time.sleep(0.75) # adding some buffer time between runs

######################################### RMCA ############################################
def run_rmca(args, scene, logfile):
    '''
    Run the MCA-RMCA commands
    
    Input:
        args (argument object)\n
        scene - instance file\n
        logfile - filepath for logfile
    '''

    commands_prog = "./build/MAPD" 
    commands_map = " -m "+ args.map
    commands_agent = " -a " + args.map
    commands_task = " -t " + scene
    commands_args = " --capacity 10000 --kiva --regret -s PP"
    full_command = commands_prog + commands_map + commands_agent + commands_task + commands_args
    run_subprocess(full_command, args, logfile)
    time.sleep(0.75) # adding some buffer time between runs

################################### MAPF-PC-LNS ###########################################

def run_mapf_pc_lns(args, scene, logfile):
    '''
    Run the MAPF-PC-LNS commands
    
    Input:
        args (argument object)\n
        scene - instance file\n
        logfile - filepath for logfile
    '''

    commands_prog = "./build/mapf_pc_lns" 
    commands_map = " -m "+ args.map
    commands_agent = " -a " + scene
    commands_args = " -k 30 -l 200 -d 6 -i 1 -t 3 -s sota_pbs"
    full_command = commands_prog + commands_map + commands_agent + commands_args
    run_subprocess(full_command, args, logfile)
    time.sleep(0.75) # adding some buffer time between runs
    
################################################## COMMON FUNCTIONS ##########################################
def run_subprocess(fullcommand, args, logfile):
    '''
    Common subprocess function to be called by different baselines
    
    Input:
        fullcommand - command line for shell to execute\n
        args (argument object)\n
        logfile - filepath to logfile
    '''
    # run subprocess
    process = Popen(fullcommand, stdout=PIPE, stderr=PIPE, text=True,shell=True, cwd=args.dir)
    # collect all the output
    f = open(logfile, "w")
    counter = 0
    while True:
        line = process.stdout.readline()
        f.write(line)
        if not line:
            if counter == 0:
                linerr = process.stderr.readline()
                if not linerr:
                    print("No STDOUT from subprocess! Please debug the command")
                    exit(1)
                else:
                    print(linerr)
                    exit(1)
            break
        counter += 1
    # save the file
    f.close()
    process.terminate()


def load_instances(args):
    '''
    Load all the instances in the target folder
    
    Input: 
        args (argument object)
    Ouput: 
        files - List of paths to instance files
    '''
    if(os.path.isfile(args.instance_dir)):
        temp_list = []
        temp_list.append(args.instance_dir)
        return temp_list
    else:
        files = glob.glob(args.instance_dir + '*.txt',recursive=True)
        if (len(files)==0): # means its a .task file
            files = glob.glob(args.instance_dir + '*/*.task',recursive=True)
    return files


def get_logfolder_name(args):
    '''
    Generate a Log folder for the current run
    Input:
        args (argument object)
    Output: 
        dir_name: string name of directory
    '''

    if(not os.path.isdir("Logs")):
        os.mkdir("Logs")
    if(not os.path.isdir("Logs/"+args.baseline)):
        os.mkdir("Logs/"+args.baseline)
    map_dir = os.path.basename(args.map).split('.')[0]
    if(not os.path.isdir("Logs/"+args.baseline+"/"+map_dir)):
        os.mkdir("Logs/"+args.baseline+"/"+map_dir)
    

    dir_name = "Logs/"+args.baseline+"/"+map_dir+"/"+str(args.subfolder).strip()
    return dir_name

def createLogFile(args, scene, dir):
    '''
    Create a logfile path for current instance
    
    Input:
        args (argument object)\n
        scene - instance file\n
        dir - directory path to store the logfile\n
    Output:
        file_path - string for the logfile path
    '''
        
    
    if (args.baseline == "rmca"):
        if (not os.path.isdir(dir)):
            os.mkdir(dir)
            path = Path(scene)
            parent_dir = os.path.basename(str(path.parent.absolute()))
            split_str = os.path.basename(scene).split('.')[0]
            file_head = "LogFile_"+parent_dir+"_"+split_str+".txt"
            file_path = dir+"/"+file_head
            return file_path
        else:
            path = Path(scene)
            parent_dir = os.path.basename(str(path.parent.absolute()))
            split_str = os.path.basename(scene).split('.')[0]
            file_head = "LogFile_"+parent_dir+"_"+split_str+".txt"
            file_path = dir+"/"+file_head
            return file_path
    else:
        if (not os.path.isdir(dir)):
            os.mkdir(dir)
            split_str = os.path.basename(scene).split('.')[0]
            file_head = "LogFile_"+split_str+".txt"
            file_path = dir+"/"+file_head
            return file_path
        else:
            split_str = os.path.basename(scene).split('.')[0]
            file_head = "LogFile_"+split_str+".txt"
            file_path = dir+"/"+file_head
            return file_path

################################################# MAIN FUNCTION #############################################
def main():
    
    parser = argparse.ArgumentParser(description='Input Arguments for Experiment script')
    parser.add_argument('--target_dir', required=True, dest='instance_dir', help='Full path to the instances folder')
    parser.add_argument('--map', required=True, dest='map', help='Full path to the map for the instances')
    parser.add_argument('--pwd', required=True, dest='dir', help='Full path to working directory for the execution of the baseline')
    parser.add_argument('--baseline', choices=['mapf_pc_lns', 'mapf_pc', 'rmca', 'mapd_wpbs'],\
        required=True, dest='baseline',help='Name of the baseline to use. Can be mapf_pc, mapf_pc_lns, rmca, mapd_wpbs' )
    parser.add_argument('--name', required=True, dest='subfolder', help='Please provide a name for the folder where logs will be saved')
    
    
    args = parser.parse_args()

    
    instance_files = load_instances(args)
    log_dir = get_logfolder_name(args)
    num_instances = len(instance_files)
    count = 1
    print("Number of total instances to run: {}".format(num_instances))    
    if(args.baseline == "mapf_pc_lns"):
        for scene in instance_files:
            print("[{}/{}]\tRunning on instance: {}".format(count, num_instances, os.path.basename(scene)))
            Lfile = createLogFile(args,scene, log_dir)
            run_mapf_pc_lns(args, scene, Lfile)
            count+=1
    elif(args.baseline == "mapf_pc"):
        for scene in instance_files:
            if ("kiva" in scene):
                print("You cannot run Kiva map instances on MAPF-PC!")
                exit(1)
            print("[{}/{}]\tRunning on instance: {}".format(count, num_instances, os.path.basename(scene)))
            Lfile = createLogFile(args,scene, log_dir)
            run_mapf_pc(args, scene, Lfile)
            count+=1
    elif(args.baseline == "rmca"):
        for scene in instance_files:
            if ("random" in scene or "warehouse" in scene):
                print("You cannot run this instance on RMCA!")
                exit(1)
            print("[{}/{}]\tRunning on instance: {}".format(count, num_instances, os.path.basename(scene)))
            Lfile = createLogFile(args,scene, log_dir)
            run_rmca(args, scene, Lfile)
            count+=1
    elif(args.baseline == "mapd_wpbs"):
        for scene in instance_files:
            if ("random" in scene or "warehouse" in scene):
                print("You cannot run this instance on MAPD!")
                exit(1)
            print("[{}/{}]\tRunning on instance: {}".format(count, num_instances, os.path.basename(scene)))
            Lfile = createLogFile(args,scene, log_dir)
            run_mapd(args, scene, Lfile)
            count+=1
    
 

####################################### PROGRAM #########################################
if __name__ == "__main__":
    main()