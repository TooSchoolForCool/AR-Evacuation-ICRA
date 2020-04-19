import sys

import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio

from modules import DensityMap
from modules import Visualizer
from modules import AgentSampler
from modules import AstarPlanner
from modules import MsAstarPlanner
from modules import DPlanner
from modules import DataLoader


def main():
    GOALS = [
    (0,23),
    (0,24),
    (0,63),
    (99,23),
    (92,99),
    (50,99)
    ]

    dm_generator = DensityMap()
    dloader = DataLoader()
    sampler = AgentSampler()
    # planner = AstarPlanner()
    # planner = MsAstarPlanner()
    planner = DPlanner()
    viz = Visualizer()

    map_file = "./assets/map.png"
    m = dloader.load(map_file, "map")

    agents = sampler.bias_sampling(m, 100, 20, 25)
    agents2 = sampler.bias_sampling(m, 100, 70, 30)
    agents3 = sampler.bias_sampling(m, 100, 70, 80)
    agents4 = sampler.sampling(m, 5)
    for agent in agents2:
        agents.append(agent)
    for agent in agents3:
        agents.append(agent)
    for agent in agents4:
        agents.append(agent)
    
    dm = dm_generator.process(m, agents)

    # show agents
    # viz.show(m, agents)
    # exit(0)
    

    planner.update_grid(m)

    path1, cost = planner.search((35, 10), GOALS)
    viz.show(m, agents + path1)

    # path, cost = planner.search((60, 30), GOALS, dm)
    # viz.show(m, agents + path, cmap=["white", "red"])
    
    path2, cost = planner.search((35, 10), GOALS, np.multiply(dm, 3))
    viz.show(m, agents + path2)

    # print(np.multiply(dm, 5))

    # path, cost = planner.search((60, 30), GOALS, np.multiply(dm, 100))
    # viz.show(m, agents + path, cmap=["white", "red"])

    # print(np.multiply(dm, 100))

    sio.savemat("density.mat", {"dm": dm})
    sio.savemat("dm_agents.mat", {"agents": agents})
    sio.savemat("dm_path.mat", {"path": path1})
    sio.savemat("dm_path_woden.mat", {"path": path2})


if __name__ == "__main__":
    main()