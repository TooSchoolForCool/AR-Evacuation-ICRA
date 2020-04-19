import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


class Visualizer(object):

    def __init__(self):
        # 0: road   --> white
        # 1: block  --> black
        # 2: agent  --> red
        self.cmap_ = colors.ListedColormap(['black', 'white', 'red'])


    def show(self, grid_map, agents, cmap=None):
        # deep copy
        world = np.copy(grid_map)

        cmap = self.cmap_ if cmap is None else colors.ListedColormap(cmap)

        for ag in agents:
            world[ag[1], ag[0]] = 2
        
        imgplot = plt.imshow(world, cmap=cmap)
        plt.show()
