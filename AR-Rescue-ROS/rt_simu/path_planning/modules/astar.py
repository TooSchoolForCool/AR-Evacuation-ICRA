import math

STEPS = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

class Node(object):
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, pos=None):
        self.parent = parent
        self.pos = pos

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.pos == other.pos

    
    def __str__(self):
        ret = "({}, {}): {}".format(self.pos[0], self.pos[1], self.f)
        return ret


class AstarPlanner(object):

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
            return self.search_(src, dst, density)
        elif isinstance(dst, list):
            min_cost = float('inf')
            assigned_dst = None
            assigned_path = None

            for d in dst:
                path, cost = self.search_(src, d, density)
                if cost < min_cost:
                    min_cost = cost
                    assigned_dst = d
                    assigned_path = path

            return assigned_path, min_cost
            
    
    def search_(self, src, dst, density=None):
        src_node = Node(None, src)
        dst_node = Node(None, dst)

        open_list = {}
        closed_list = {}
    
        open_list[src] = src_node
        while len(open_list) > 0:

            node = self.pop_next_(open_list)
            closed_list[node.pos] = node
            
            if node == dst_node:
                return self.tracepath_(node, closed_list)

            for step in STEPS:
                next_pos = (node.pos[0] + step[0], node.pos[1] + step[1])

                # Make sure within range
                if not self.is_valid(next_pos[0], next_pos[1]):
                    continue

                # Make sure walkable terrain
                if not self.is_unblocked(next_pos[0], next_pos[1]):
                    continue

                new_node = Node(node, next_pos)
                new_node.g = node.g + self.cost_(new_node, node, density)
                new_node.h = self.heuristic_(new_node, dst_node)
                new_node.f = new_node.g + new_node.h

                if self.if_add_node_(new_node, open_list, closed_list):
                    open_list[next_pos] = new_node
        
        # Cannot find a path
        return [], None


    def is_valid(self, x, y):
        return x < self.width_ and x >=0 and y < self.height_ and y >= 0 

    
    def is_unblocked(self, x, y):
        return self.g_[y, x] != -1


    def heuristic_(self, a, b):
        return math.sqrt(
            ((a.pos[0] - b.pos[0]) ** 2) 
                + ((a.pos[1] - b.pos[1]) ** 2)
        )
    

    def cost_(self, a, b, density=None):
        d_cost = 0.0
        beta = 1 if density is None else self.beta_

        if density is not None:
            x = b.pos[0]
            y = b.pos[1]
            d_cost = density[y, x]

        if a.pos[0] == b.pos[0] or a.pos[1] == b.pos[1]:
            return beta * 1.0 + (1 - beta) * d_cost
        else:
            return beta * 1.414 + (1 - beta) * d_cost

    
    def pop_next_(self, open_list):
        next_node = None

        for pos, node in open_list.items():
            if next_node is None:
                next_node = node
            else:
                if node.f < next_node.f:
                    next_node = node

        return open_list.pop(next_node.pos)


    def tracepath_(self, node, closed_list):
        path = []
        cost = 0.0

        while node is not None:
            path.append(node.pos)
            cost += node.g

            node = node.parent
        
        # Return reversed path
        return path[::-1], cost


    def if_add_node_(self, node, open_list, closed_list):
        # Child is on the closed list
        if node.pos in closed_list:
            return node.f < closed_list[node.pos].f
        
        # Child is already in the open list
        if node.pos in open_list:
            return node.f < open_list[node.pos].f
        
        return True