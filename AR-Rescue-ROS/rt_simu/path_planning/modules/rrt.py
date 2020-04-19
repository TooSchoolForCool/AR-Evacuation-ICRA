import numpy as np
import math
from scipy import interpolate


class RrtPlanner(object):
    
    def __init__(self, grid=None):
        self.g_ = grid

        if grid is not None:
            self.width_ = grid.shape[1]
            self.height_ = grid.shape[0]

        # a list of nodes (x, y)
        self.nodes_ = []
        # record parents of each node
        self.edges_ = {}

        self.goal_sample_rate_ = 0.2
        # expansion radius
        self.r_ = 1.5

    
    def update_grid(self, grid):
        self.g_ = grid
        self.width_ = grid.shape[1]
        self.height_ = grid.shape[0]


    def search(self, src, dst):
        self.nodes_ = [src]
        self.edges_ = {src : None}

        while True:
            node = self.random_node_(dst)

            nearest = self.find_nearest_(node)
            expanded = self.calc_expanded_node_(nearest, node)

            if self.check_collision_(expanded, nearest):
                continue
            
            self.add_node_(expanded, nearest)

            if self.if_find_goal(expanded, dst):
                path, cost = self.find_path_(expanded, dst)
                return path, cost

    
    def grow_tree(self, src, n_iter):
        self.nodes_ = [src]
        self.edges_ = {src : None}

        for _ in range(n_iter):
            node = self.random_node_()

            nearest = self.find_nearest_(node)
            expanded = self.calc_expanded_node_(nearest, node)

            if self.check_collision_(expanded, nearest):
                continue
            
            self.add_node_(expanded, nearest)

            return self.nodes_, self.edges_

    
    def add_node_(self, child, parent):
        assert(child != parent)

        self.nodes_.append(child)
        self.edges_[child] = parent


    def random_node_(self, dst=None):
        if dst is not None and np.random.rand() < self.goal_sample_rate_:
            return dst
        
        while True:
            x = np.random.randint(0, self.width_)
            y = np.random.randint(0, self.height_)

            # already in the list
            if (x, y) in self.nodes_:
                continue
            # hit obstacle
            if self.is_collision_(x, y):
                continue
            
            return (x, y)

    
    def find_nearest_(self, node):
        x, y = node

        # vectorization
        arr = np.array(self.nodes_)
        x_vec = arr[:, 0] - x
        y_vec = arr[:, 1] - y
        dist_vec = (x_vec ** 2) + (y_vec ** 2)

        idx = np.argmin(dist_vec)

        return self.nodes_[idx]

    
    def calc_expanded_node_(self, src, dst):
        theta = math.atan2(dst[1] - src[1], dst[0] - src[0])
        x = int(src[0] + self.r_ * math.cos(theta))
        y = int(src[1] + self.r_ * math.sin(theta))

        return x, y

    
    def if_find_goal(self, src, dst):
        dx = src[0] - dst[0]
        dy = src[1] - dst[1]
        dist = dx ** 2 + dy ** 2

        return dist <= self.r_ ** 2

    
    def is_collision_(self, x, y):
        return self.g_[y, x] == -1

    
    def find_path_(self, src, dst):
        path = [dst]
        cost = 0

        while src is not None:
            path.append(src)
            src = self.edges_[src]

            cost += 1
        
        return path[::-1], cost

    
    def check_collision_(self, src, dst):
        # max_x = max(src[0], dst[0])
        # min_x = min(src[0], dst[0])
        # interp_x = [i for i in range(min_x, max_x + 1)]

        # xp = [src[0], dst[0]]
        # yp = [src[1], dst[1]]

        # interp_y = np.interp(interp_x, xp, yp)

        # # print("src: {}, dst: {}".format(src, dst))
        # # print(interp_x)
        # # print(interp_y)

        # for x, y in zip(interp_x, interp_y):
        #     if self.is_collision_(x, int(y)):
        #         return True
        return self.is_collision_(dst[0], dst[1])
