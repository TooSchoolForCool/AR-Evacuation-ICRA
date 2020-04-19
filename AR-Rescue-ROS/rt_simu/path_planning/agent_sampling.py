import sys

from modules import Visualizer
from modules import AgentSampler
from modules import DataLoader


if __name__ == "__main__":
    try:
        map_file = sys.argv[1]
        n_agents = int(sys.argv[2])
        out_dir = sys.argv[3]
    except:
        raise Exception("Need to specify the map file and number of agents")

    dloader = DataLoader()
    sampler = AgentSampler()

    grid_map = dloader.load(map_file, "map")
    agents = sampler.sampling(grid_map, n_agents)

    dloader.save(agents, out_dir, file_type="agent_samples")
    print("Agent data saved at: {}".format(out_dir))

    viz = Visualizer()
    viz.show(grid_map, agents)
