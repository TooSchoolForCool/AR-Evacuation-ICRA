import numpy as np


class AgentSampler(object):

    def __init__(self):
        pass

    
    def sampling(self, grid_map, n_agents):
        """
        Sampling a list of agents' init locations

        Returns:
            agents (list of tuple): [(x1, y1), ...]
        """
        agents = []
        height, width = grid_map.shape

        while n_agents > 0:
            c, r = self.random_xy_(width - 1, height - 1)
            # print("add agent ({}, {}), left {}".format(c, r, n_agents))


            if self.is_block_(grid_map, c, r):
                continue

            if not self.is_valid(c, r, width, height):
                continue

            # if not self.is_isolate(agents, c, r, 1):
            #     continue
            
            agents.append( (c, r) )
            n_agents -= 1
        
        return agents

    
    def bias_sampling(self, grid_map, n_agents, rx, ry):
        """
        Sampling a list of agents' init locations

        Returns:
            agents (list of tuple): [(x1, y1), ...]
        """
        agents = []
        height, width = grid_map.shape

        rx = max(1, min(width - 1, rx))
        ry = max(1, min(height - 1, ry))

        while n_agents > 0:
            c, r = self.gausian_xy_(rx, ry)
            # print("add agent ({}, {}), left {}".format(c, r, n_agents))

            if self.is_block_(grid_map, c, r):
                continue
            
            if not self.is_valid(c, r, width, height):
                continue

            # if not self.is_isolate(agents, c, r, 2):
            #     continue
            
            agents.append( (c, r) )
            n_agents -= 1
        
        return agents

    
    def gausian_xy_(self, rx, ry, sigma=4.5):
        x = int(np.random.normal(rx, sigma))
        y = int(np.random.normal(ry, sigma))

        return x, y


    def random_xy_(self, width, height):
        x = np.random.randint(1, width)
        y = np.random.randint(1, height)

        return x, y


    def is_block_(self, grid_map, c, r):
        return grid_map[r, c] == -1


    def is_isolate(self, agents, c, r, gap=2):
        """
        Check if there are two agents next to each other, or overlap
        """
        for a in agents:
            if abs(a[0] - c) <= gap and abs(a[1] - r) <= gap:
                return False

        return True

    def is_valid(self, x, y, width, height):
        if x < 0 or y < 0 or x >= width or y >= height:
            return False

        return True