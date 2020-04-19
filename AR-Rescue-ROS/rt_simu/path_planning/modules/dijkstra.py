import math
from heapq import heappush, heappop


STEPS = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]


class DPlanner(object):

    def __init__(self, grid=None, beta=0.5):
        self.g_ = grid

        assert(beta >= 0 and beta <= 1)
        self.beta_ = beta

        if grid is not None:
            self.width_ = grid.shape[1]
            self.height_ = grid.shape[0]

    
    def update_grid(self, grid):
        self.g_ = grid
        self.width_ = grid.shape[1]
        self.height_ = grid.shape[0]


    def search(self, src, dst, density=None):
        if isinstance(dst, tuple):
            return self.search_(src, [dst], density)
        elif isinstance(dst, list):
            return self.search_(src, dst, density)
            
    
    def search_(self, src, dst_list, density=None):
        visited_goals = set()
        visited = set()
        dist_set = {}
        queue = [(0, src, None)]

        while queue:
            path_len, v, parent = heappop(queue)
            
            if v in visited:
                continue

            dist_set[v] = (path_len, parent)
            visited.add(v)

            for dx, dy in STEPS:
                nx = v[0] + dx
                ny = v[1] + dy
                
                if not self.is_valid(nx, ny):
                    continue

                if not self.is_unblocked(nx, ny):
                    continue

                if (nx, ny) not in visited:
                    cost = self.cost_((nx, ny), v, density=density)
                    heappush(queue, (path_len + cost, (nx, ny), v))
        
        return self.tracepath_(dist_set, dst_list, src)
        

    def find_next_(self, dist_set, visited):
        cost = float('inf')
        ret = None

        for k, v in dist_set.items():
            if k in visited:
                continue
            
            if v[0] < cost:
                cost = v[0]
                ret = k
        
        return ret, cost


    def is_valid(self, x, y):
        return x < self.width_ and x >=0 and y < self.height_ and y >= 0 

    
    def is_unblocked(self, x, y):
        return self.g_[y, x] != -1

        
    def cost_(self, a, b, density=None):
        d_cost = 0.0
        beta = 1 if density is None else self.beta_

        if density is not None:
            x = b[0]
            y = b[1]
            d_cost = density[y, x]

        if a[0] == b[0] or a[1] == b[1]:
            return beta * 1.0 + (1 - beta) * d_cost
        else:
            return beta * 1.414 + (1 - beta) * d_cost


    def tracepath_(self, dist_set, dst_list, src):
        shortest_path = []
        best_cost = float('inf')
        best_dst = None

        for dst in dst_list:
            if dist_set[dst][0] < best_cost:
                best_cost = dist_set[dst][0]
                best_dst = dst

        node = best_dst
        path = []
        while node != src:
            path.append(node)
            node = dist_set[node][1]
        
        return path[::-1], best_cost
