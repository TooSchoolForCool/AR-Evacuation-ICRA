import math

import numpy as np


class DensityMap(object):

    def __init__(self, r=5):
        self.r_ = r

        self.steps_ = [
            (dx, dy) 
            for dx in range(-self.r_, self.r_)
            for dy in range(-self.r_, self.r_)
        ]

        #print(self.steps_)
    

    def process(self, m, agents):
        height, width = m.shape

        cnt_map = np.zeros((height, width))
        density_map = np.zeros((height, width))

        for x, y in agents:
            cnt_map[y, x] += 1
        
        for y in range(height):
            for x in range(width):
                self.patch_(density_map, x, y, cnt_map[y, x])
        
        return density_map


    def patch_(self, dm, x, y, n):
        if n <= 0:
            return
        
        height, width = dm.shape

        for dx, dy in self.steps_:
            nx = x + dx
            ny = y + dy

            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue

            dist = max(dx ** 2 + dy ** 2, 1)

            dm[ny, nx] += n / math.exp(dist)

        

        