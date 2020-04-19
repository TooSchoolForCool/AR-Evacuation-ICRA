import cv2
import numpy as np
import scipy.io

class GridMapGenerator(object):

    def __init__(self):
        pass
    

    def generate(self, map_file, scaler):
        map_img = cv2.imread(map_file)
        map_gray = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)

        grid_map = self.discretize_(map_gray, scaler)

        self.save_(grid_map, map_file)

    
    def discretize_(self, m, scaler):
        h, w = m.shape

        grid_map = np.ones(((h + scaler-1) // scaler, (w + scaler-1) // scaler))

        for r in range(0, h, scaler):
            for c in range(0, w, scaler):
                if self.is_block_(m, r, c, scaler):
                    grid_map[r//scaler, c//scaler] = 0
                else:
                    grid_map[r//scaler, c//scaler] = 1
        
        return grid_map
    
    def is_block_(self, m, r, c, scaler):
        val = np.sum(m[r:r+scaler, c:c+scaler])
        return val < (128 * scaler * scaler)

    def save_(self, grid_map, map_file):
        print(grid_map.shape)

        grid_map.dump("new.numpy")
        scipy.io.savemat("new.mat", {"map": grid_map})

        cv2.imshow("grid", grid_map)
        cv2.waitKey()