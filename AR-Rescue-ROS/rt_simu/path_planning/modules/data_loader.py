import os

import cv2
import numpy as np
import scipy.io as sio


class DataLoader(object):

    def __init__(self):
        pass


    def load(self, file_dir, file_type):
        if file_type == "map":
            return self.load_map_(file_dir)
        else:
            raise Exception("[ERROR] Do not support type {}".format(file_type))


    def save(self, data, out_path, file_type, out_dir="simu_results"):
        if file_type == "agent_samples":
            self.save_nparray_(data, out_path)
        elif file_type == "planning":
            self.save_planning_(data, out_path, out_dir)
        else:
            raise Exception("[ERROR] Do not support type {}".format(file_type))
    

    def load_map_(self, file_dir):
        """
        Load Grid world map

        Args:
            file_dir (string): file path

        Returns:
            grid_map (np.ndarray): 
                0   -->     available road
                1   -->     block
        """
        map_bgr = cv2.imread(file_dir)

        if map_bgr is None:
            raise Exception("[ERROR] Cannot read image file {}".format(file_dir))
        
        map_gray = cv2.cvtColor(map_bgr, cv2.COLOR_BGR2GRAY)

        height, width = map_gray.shape
        grid_map = np.zeros((height, width))

        for h in range(0, height):
            for w in range(0, width):
                if map_gray[h, w] < 200:
                    grid_map[h, w] = -1
        
        return grid_map

    
    def save_nparray_(self, arr, out_path):
        """
        Save agent samples

        Args:
            agents (list of tuples): a list of initial agent locations (x, y)
            out_path (string): output directory
        """
        np_arr = np.array(arr)
        np_arr.dump(out_path)

    
    def save_planning_(self, data, out_path, out_dir="simu_results"):
        os.system("mkdir -p {}".format(out_dir))

        cnt = 0
        for k, v in data.items():
            nparr = np.array(v)
            
            sio.savemat("./{}/group".format(out_dir) + str(cnt) + "_" + out_path, {"group": nparr})
            cnt += 1


        