import sys
import os
import time

import scipy.io as sio

from modules import Visualizer
from modules import AgentSampler
from modules import DataLoader
from modules import AstarPlanner
from modules import MsAstarPlanner
from modules import RrtPlanner
from modules import ExitScheduler
from modules import MultiThreadPlanner
from modules import DPlanner

GOALS = [
    (0, 23),
    (0, 63),
    (99, 23),
    (92, 99),
    (50, 99)
]

class Evaluator(object):
    def __init__(self, map_file):
        self.dloader_ = DataLoader()
        self.sampler_ = AgentSampler()

        self.grid_map_ = self.dloader_.load(map_file, "map")


    def evaluate(self, n_rounds, planner_type):
        self.create_folder_(planner_type + "_result")
        
        n_agents_test = [i for i in range(50, 1001, 50)]
        
        planner = self.craete_planner_(planner_type)
        planner.update_grid(self.grid_map_)

        if planner_type == "mt_planner":
            self.test_async_(n_agents_test, n_rounds, planner, planner_type)
        else:
            self.test_single_core_(n_agents_test, n_rounds, planner, planner_type)

    
    def test_single_core_(self, n_agents_test, n_rounds, planner, outname):
        time_hist = []

        for n_agents in n_agents_test:
            n_agents_time = []
            
            for i in range(n_rounds):
                agents = self.sampler_.sampling(self.grid_map_, n_agents)

                start_time = time.time()

                for agent in agents:
                    planner.search(agent, GOALS)
                
                end_time = time.time()

                elapsed_time = end_time - start_time
                n_agents_time.append(elapsed_time)

                print("[Round {}]\t: planner: {}\tn_agents: {}\telapsed_time:{:4f}".format(
                    i, outname, n_agents, elapsed_time))
            
            time_hist.append(n_agents_time)
        
        output = outname + "_result/" + outname + ".mat"
        sio.savemat(output, {"t_hist": time_hist})

    
    def test_async_(self, n_agents_test, n_rounds, planner, outname):
        # n_cores_test = [2, 4, 8, 16, 32]
        n_cores_test = [1]

        for n_cores in n_cores_test:
            time_hist = []

            for n_agents in n_agents_test:
                n_agents_time = []
                
                for i in range(n_rounds):
                    agents = self.sampler_.sampling(self.grid_map_, n_agents)

                    start_time = time.time()

                    planner.search(agents, GOALS, n_cores=n_cores, planner_type="ms_astar")
                    
                    end_time = time.time()

                    elapsed_time = end_time - start_time
                    n_agents_time.append(elapsed_time)

                    print("[Round {}]\tn_cores: {}\tn_agents: {}\telapsed_time:{:4f}".format(
                        i, n_cores, n_agents, elapsed_time))
                
                time_hist.append(n_agents_time)
            
            output = outname + "_result/" + str(n_cores) + "_cores.mat"
            sio.savemat(output, {"t_hist": time_hist})


    def craete_planner_(self, planner_type):
        if planner_type == "astar":
            return AstarPlanner()
        elif planner_type == "rrt":
            return RrtPlanner()
        elif planner_type == "ms_astar":
            return MsAstarPlanner()
        elif planner_type == "mt_planner":
            return MultiThreadPlanner()
        elif planner_type == "dplanner":
            return DPlanner()

    
    def create_folder_(self, folder_name):
        if not os.path.exists(folder_name):
            print("[INFO] create folder: {}".format(folder_name))
            os.system("mkdir {}".format(folder_name))


def main():
    try:
        map_file = sys.argv[1]
    except:
        raise Exception("Need to specify the map file and number of agents")
    
    evaluator = Evaluator(map_file)
    evaluator.evaluate(n_rounds=50, planner_type="dplanner")


if __name__ == "__main__":
    main()