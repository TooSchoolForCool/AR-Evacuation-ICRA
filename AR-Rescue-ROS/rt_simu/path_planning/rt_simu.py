import sys
import os
import time

from modules import Visualizer
from modules import AgentSampler
from modules import DataLoader
from modules import AstarPlanner
from modules import MsAstarPlanner
from modules import RrtPlanner
from modules import ExitScheduler
from modules import DensityMap
from modules import MultiThreadPlanner

import numpy as np

GOALS = [
    (0,23),
    (0,63),
    (99,23),
    (92,99),
    (50,99)
]

# def Simulator(object):
    
#     def __init__(self, map_file, planner_type):
#         self.dloader_ = DataLoader()
#         self.sampler_ = AgentSampler()
#         self.planner_ = self.craete_planner_(planner_type)
#         self.dm_ = DensityMap()

#         self.m_ = self.dloader_ .load(map_file, "map")


#     def run(self, n_agents):
#         agents = self.sampler_.sampling(self.m_, n_agents)
#         dm = self.dm_.process()
        

    
#     def craete_planner_(self, planner_type):
#         if planner_type == "astar":
#             return AstarPlanner()
#         elif planner_type == "rrt":
#             return RrtPlanner()
#         elif planner_type == "ms_astar":
#             return MsAstarPlanner()
#         elif planner_type == "mt_planner":
#             return MultiThreadPlanner()
#         elif planner_type == "dplanner":
#             return DPlanner()

    
#     def create_folder_(self, folder_name):
#         if not os.path.exists(folder_name):
#             print("[INFO] create folder: {}".format(folder_name))
#             os.system("mkdir {}".format(folder_name))


if __name__ == "__main__":
    map_file = "./assets/map.png"
    n_agents = 200
    trials = 10

    dloader = DataLoader()
    sampler = AgentSampler()
    dm_generator = DensityMap()
    planner = MsAstarPlanner()

    grid_map = dloader.load(map_file, "map")
    planner.update_grid(grid_map)

    for gamma in range(11, 12, 2):
        for trial in range(trials):
            # start_time = time.time()
            print("Runing trial {}...".format(trial))
            g1 = np.random.randint(30, 50)
            g2 = np.random.randint(30, 50)
            g3 = np.random.randint(30, 50)
            g4 = 200 - g1 - g2 - g3

            agents = sampler.bias_sampling(grid_map, g1, np.random.randint(20, 80), np.random.randint(20, 80))
            agents2 = sampler.bias_sampling(grid_map, g2, np.random.randint(20, 80), np.random.randint(20, 80))
            agents3 = sampler.bias_sampling(grid_map, g3, np.random.randint(20, 80), np.random.randint(20, 80))
            agents4 = sampler.sampling(grid_map, g4)
            for agent in agents2:
                agents.append(agent)
            for agent in agents3:
                agents.append(agent)
            for agent in agents4:
                agents.append(agent)
            # agents = sampler.sampling(grid_map, n_agents)

            # With density map
            scheduler = ExitScheduler(grid_map, "astar")
            dm = dm_generator.process(grid_map, agents)
            
            exit_plan = []
            for i in range(len(agents)):
                exit_plan.append(scheduler.assign([agents[i]], GOALS, np.multiply(dm, gamma))[0])
            #print(exit_plan)
            
            # end_time = time.time()
            # print("Exit allocation time elapse: {}".format(end_time - start_time))

            # viz = Visualizer()
            # viz.show(grid_map, agents)
            
            evacuation_plan = {goal: [] for goal in GOALS}

            for agent, goal in zip(agents, exit_plan):
                path, cost = planner.search(agent, [goal])
                # viz.show(grid_map, path)

                evacuation_plan[goal].append(path)
            
            dloader.save(evacuation_plan, "agent_traj_wdensity_"+str(trial)+"_nrand_gamma"+str(gamma)+".mat", "planning")

            # Without density map
            scheduler = ExitScheduler(grid_map, "ms_astar")
            exit_plan = scheduler.assign(agents, GOALS)
            
            # end_time = time.time()
            # print("Exit allocation time elapse: {}".format(end_time - start_time))

            # viz = Visualizer()
            # viz.show(grid_map, agents)
            
            evacuation_plan = {goal: [] for goal in GOALS}

            for agent, goal in zip(agents, exit_plan):
                path, cost = planner.search(agent, [goal])

                # viz.show(grid_map, path)

                evacuation_plan[goal].append(path)
            
            dloader.save(evacuation_plan, "agent_traj_wodensity_"+str(trial)+"_nrand_gamma"+str(gamma)+".mat", "planning")