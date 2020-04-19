import threading
import multiprocessing

import numpy as np

from .astar import AstarPlanner
from .ms_astar import MsAstarPlanner
from .rrt import RrtPlanner


class MultiThreadPlanner(object):

    def __init__(self, grid=None):
        self.g_ = grid

        if grid is not None:
            self.width_ = grid.shape[1]
            self.height_ = grid.shape[0]

    
    def update_grid(self, grid):
        self.g_ = grid
        self.width_ = grid.shape[1]
        self.height_ = grid.shape[0]
    

    def search(self, agents, dst_list, n_cores, planner_type):
        threads = []

        dsize = int(len(agents) / n_cores)

        for i in range(n_cores):
            start = i * dsize
            end = start + dsize

            # print("Thread {}: agent {} ~ {}".format(i, start, end))

            threads.append(
                multiprocessing.Process(
                    target=self.run_, 
                    args=(agents[start : end], dst_list, planner_type)
                )
            )

        
        for t in threads:
            t.start()

        
        for t in threads:
            t.join()

    
    def run_(self, agents, dst_list, planner_type):
        planner = self.create_planner(planner_type)
        planner.update_grid(self.g_)

        for agent in agents:
            planner.search(agent, dst_list)

    
    def create_planner(self, planner_type):
        if planner_type == "astar":
            return AstarPlanner()
        elif planner_type == "rrt":
            return RrtPlanner()
        elif planner_type == "ms_astar":
            return MsAstarPlanner()