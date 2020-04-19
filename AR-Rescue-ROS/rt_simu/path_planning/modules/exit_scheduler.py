import numpy as np

from .astar import AstarPlanner
from .ms_astar import MsAstarPlanner
from .rrt import RrtPlanner

class ExitScheduler(object):

    def __init__(self, grid, planner_type):
        self.g_ = np.copy(grid)

        self.width_ = grid.shape[1]
        self.height_ = grid.shape[0]

        self.planner_ = self.create_planner_(planner_type)
        self.planner_.update_grid(self.g_)
    

    def assign(self, agents, goals, density_map=None, n_iter=1):
        exit_plan = []
        
        for agent in agents:
            if density_map is not None:
                path, cost = self.planner_.search(agent, goals, density=density_map)
                exit_plan.append(path[-1])
            else:
                path, cost = self.planner_.search(agent, goals)
                exit_plan.append(path[-1])
        
        return exit_plan

    
    def create_planner_(self, planner_type):
        if planner_type == "astar":
            return AstarPlanner()
        elif planner_type == "rrt":
            return RrtPlanner()
        elif planner_type == "ms_astar":
            return MsAstarPlanner()
        