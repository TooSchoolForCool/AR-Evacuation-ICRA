import sys
import os
import time
import multiprocessing

from modules import Visualizer
from modules import AgentSampler
from modules import DataLoader
from modules import DensityMap
from modules import AstarPlanner
from modules import MsAstarPlanner
from modules import DPlanner

import numpy as np
from pathlib import Path

# t{trail#}_{density/nodensity}_a{n_agents}_gamma{gamma}.mat
FILE_NAME_BASE = "t{}_{}_a{}_gamma{}.mat"

# number of threads
N_PROC = 10

PLANNERS = {
    "dijkstra": DPlanner
}

class Simulator():

    def __init__(
        self,
        map_file,
        goals
    ):
        self.goals_ = goals
        self.map_file_ = map_file

        self.loader_ = DataLoader()
        self.sampler_ = AgentSampler()
        self.density_map_ = DensityMap()

        self.grid_map_ = self.loader_.load(self.map_file_, "map")

    def async_run(
        self,
        n_agents,
        n_trails,
        planner_type,
        gamma,
        beta,
        out_dir
    ):  
        for trail in range(n_trails):
            print("Trail {}:\t[n_agent {}]\t[gamma {}]".format(trail + 1, n_agents, gamma))

            agents = self.sample_agents_(n_agents)

            # without density map
            self.async_run_(trail, agents, gamma, beta, planner_type, False, out_dir)
            # with density map
            self.async_run_(trail, agents, gamma, beta, planner_type, True, out_dir)


    def async_run_(self, trail, agents, gamma, beta, planner_type, enable_density, out_dir):
        evacuation_plan = {goal: [] for goal in self.goals_}
        results = [[] for _ in range(N_PROC)]
        processes = []
        density_map = None

        if enable_density:
            dm = self.density_map_.process(self.grid_map_, agents)
            density_map = np.multiply(dm, gamma)

        chunk_sz = int(len(agents) / N_PROC) + 1
        for i in range(N_PROC):
            processes.append(
                multiprocessing.Process(
                    target=self.search_proc_, 
                    args=(agents[i * chunk_sz : (i+1) * chunk_sz], self.goals_[:], 
                        planner_type, beta, results[i], density_map
                    )
                )
            )
        
        for proc in processes:
            proc.start()

        for proc in processes:
            proc.join()

        for proc_res in results:
            for path in proc_res:
                evacuation_plan[path[-1]].append(path)

        self.loader_.save(
            evacuation_plan, 
            FILE_NAME_BASE.format(
                trail + 1, "density" if enable_density else "nodensity", 
                len(agents), gamma
            ), 
            out_dir, "planning"
        )
        

    def search_proc_(self, agents, goals, planner_type, beta, results, dm=None):
        planner = self.create_planner_(planner_type, beta)
        planner.update_grid(self.grid_map_)

        for agent in agents:
            path, _ = planner.search(agent, goals, dm)
            results.append(path)
        

    def run(
        self,
        n_agents,
        n_trails,
        planner_type,
        gamma,
        beta,
        out_dir
    ):
        planner = self.create_planner_(planner_type, beta)
        planner.update_grid(self.grid_map_)

        for trail in range(n_trails):
            print("Trail {}:\t[n_agent {}]\t[gamma {}]".format(trail + 1, n_agents, gamma))

            agents = self.sample_agents_(n_agents)

            # without density map
            evacuation_plan = {goal: [] for goal in self.goals_}

            for agent in agents:
                path, cost = planner.search(agent, self.goals_)
                evacuation_plan[path[-1]].append(path)
            
            self.loader_.save(evacuation_plan, FILE_NAME_BASE.format(
                trail + 1, "nodensity", n_agents, gamma), out_dir, "planning")

            # with density map
            evacuation_plan = {goal: [] for goal in self.goals_}
            dm = self.density_map_.process(self.grid_map_, agents)
            density_map = np.multiply(dm, gamma)

            for agent in agents:
                path, cost = planner.search(agent, self.goals_, density_map)
                evacuation_plan[path[-1]].append(path)

            self.loader_.save(evacuation_plan, FILE_NAME_BASE.format(
                trail + 1, "density", n_agents, gamma), out_dir, "planning")

    
    def create_planner_(self, planner_type, beta):
        assert(planner_type in PLANNERS)
        return PLANNERS[planner_type](beta=beta)


    def sample_agents_(self, n_agents):
        return self.sampler_.sampling(self.grid_map_, n_agents)

    
    def init_dir_(self, outdir):
        return Path(outdir)
        

def main():
    out_dirs = ["simu_res_{}".format(i) for i in range(15)]

    map_file = "./assets/map.png"
    goals = [
        (0,23),
        (0,63),
        (99,23),
        (92,99),
        (50,99)
    ]

    n_trails = 10
    n_agents_tests = [100 * i for i in range(1, 13)]
    gamma_tests = [3, 5, 8, 12]
    beta = 0.5

    simulator = Simulator(map_file=map_file, goals=goals)

    for out_dir in out_dirs:
        for n_agents in n_agents_tests:
            for gamma in gamma_tests:
                simulator.run(
                    n_agents=n_agents,
                    n_trails=n_trails,
                    planner_type="dijkstra",
                    gamma=gamma,
                    beta=beta,
                    out_dir=out_dir
                )


if __name__ == "__main__":
    main()