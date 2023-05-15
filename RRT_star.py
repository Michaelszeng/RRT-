"""
Helper functions and classes for RRT* algorithm
"""
import math
import random
import pygame
import time
import numpy as np

from RRT import *

class RRTStarGraph(RRTGraph):
    def __init__(self, start, goal, map_dimensions, step_size, obstacles, goal_tolerance = 50, neighborhood_radius = 0):
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.step_size = step_size
        self.obstacles = obstacles
        self.path = []
        self.goal_tolerance = goal_tolerance
        self.neighborhood_radius = 1.5 * step_size  # for RRT* rewiring step

        # Tree dictionary. keys are node ID, values are tuple of own coordinates, parent ID, and current cost from start node.
        self.tree = {0: (self.start, 0, 0)}  # start node is its own parent


    def add_node(self, x, y, nearest_node_id):
        """
        Assumes the input x and y are valid (i.e. not in an obstacle and
        there is a clear path to the nearest node)

        Returns True if the new node is within the goal tolerance of the goal.
        False otherwise.
        """
        new_node_id = len(self.tree)  # Create new id for node
        parent_id = nearest_node_id
        parent_cost = self.tree[parent_id][2]
        new_node_cost = parent_cost + self.step_size
        self.tree[new_node_id] = ((x,y), parent_id, new_node_cost)
        
        rewired_edges = self.rewire(new_node_id, parent_id)

        # Manhattan distance
        if math.fabs(x - self.goal[0]) + math.fabs(y - self.goal[1]) < self.goal_tolerance:
            return new_node_id, True, rewired_edges

        return new_node_id, False, rewired_edges
    

    def rewire(self, new_node_id, parent_id, visualize = True):
        """
        Rewiring step of RRT*. Takes in the ID of a new node and finds all nodes
        in the new node's neighborhood. 

        First, rewires the new node, seeing if there is a lower cost parent among 
        its neighbors for the new node.
        
        Secondly, investigates every neighbor of the new node, and sees if the
        neighbor has a lower cost path through the new node.

        Note: to reduce computation, the neighborhood is calculated using
        mahattan distance instead of euclidean distance.

        If visualize is set to True, returns a dict of tuples of node IDs that 
        were rewired. This way, the program knows which edges to "undraw" (i.e.
        draw over with in black).
        """
        # dict mapping an edge to the edge that will replace it (an edge is a tuple of 2 node IDs)
        rewired_edges = {}

        new_node_coords = self.tree[new_node_id][0]
        new_node_x = self.tree[new_node_id][0][0]
        new_node_y = self.tree[new_node_id][0][1]

        new_node_neighbors = []  # list of node IDs that are in the neighborhood of the new node (not including the node itself, but yes including the node's original parent, nearest_node)
        for neighbor, neighbor_data in self.tree.items():
            if neighbor != new_node_id:
                node_x = neighbor_data[0][0]
                node_y = neighbor_data[0][1]

                dist = math.sqrt((node_x - new_node_x)**2 + (node_y - new_node_y)**2)  # distance between new node and neighbor

                if dist < self.neighborhood_radius:  # if neighbor is in neighborhood

                    new_node_neighbors.append(neighbor)

                    if neighbor_data[2] + dist + 1 < self.tree[new_node_id][2]:  # if new node has a lower cost path through node (+1 is to account for rounding errors)
                        # rewire new node with node as parent
                        self.tree[new_node_id] = (new_node_coords, neighbor, neighbor_data[2] + dist)
                        if visualize:
                            rewired_edges[(new_node_id, parent_id)] = (new_node_id, neighbor)


        def compare_cost_v2(neighbor_id, neighbor_x, neighbor_y, old_parent_id, old_parent_x, old_parent_y, new_parent_id, new_parent_x, new_parent_y):
            """
            Helper function to determine if rewiring is beneficial. Returns
            True if new_parent_id has lower cost path to neighbor.

            Note: new_parent_id is the id of the new node.
            """
            # We will explore up the tree from both the old parent and new parent,
            # keeping track of the total cost and sets of parent nodes explored. 
            # Once we find a common parent in the two sets, we will compare the
            # costs and return the ID of the parent with the lower cost.

            # Manhattan distance
            neighbor_to_old_parent_cost = math.fabs(neighbor_x - old_parent_x) + math.fabs(neighbor_y - old_parent_y)
            neighbor_to_new_parent_cost = math.fabs(neighbor_x - new_parent_x) + math.fabs(neighbor_y - new_parent_y)

            # keys are node IDs, values are total cost to get to that node
            nodes_explored_from_old_parent = {old_parent_id: neighbor_to_old_parent_cost}
            nodes_explored_from_new_parent = {new_parent_id: neighbor_to_new_parent_cost}

            # "current" variables hold id of current node being explored
            # start from the old parent's parent and new node's parent
            current_node_old_parent = old_parent_id
            current_node_new_parent = new_parent_id

            EXPLORATION_LIMIT = 50  # if it needs to search back more than 50, give up for efficiency's sake
            while len(nodes_explored_from_new_parent) < EXPLORATION_LIMIT:
                # explore a new node from both the old parent path and new parent path
                explore_from_old = self.tree[current_node_old_parent][1]
                explore_from_new = self.tree[current_node_new_parent][1]
                
                # calculate the total cost to the newly explored nodes
                nodes_explored_from_old_parent[explore_from_old] = nodes_explored_from_old_parent[current_node_old_parent] + math.fabs(self.tree[current_node_old_parent][0][0] - self.tree[explore_from_old][0][0]) + math.fabs(self.tree[current_node_old_parent][0][1] - self.tree[explore_from_old][0][1])
                nodes_explored_from_new_parent[explore_from_new] = nodes_explored_from_new_parent[current_node_new_parent] + math.fabs(self.tree[current_node_new_parent][0][0] - self.tree[explore_from_new][0][0]) + math.fabs(self.tree[current_node_new_parent][0][1] - self.tree[explore_from_new][0][1])

                # Check if there is a shared node in the two sets of explored nodes
                intersection = set(nodes_explored_from_old_parent.keys()) & set(nodes_explored_from_new_parent.keys())
                if intersection:  # if there is a shared node, compare the cost of the paths from the shared node to the new node
                    shared_parent_id = intersection.pop()
                    print("Intersection found!")
                    print("nodes_explored_from_old_parent: " + str(nodes_explored_from_old_parent))
                    print("nodes_explored_from_new_parent: " + str(nodes_explored_from_new_parent))
                    print("intersection: " + str(intersection))
                    print("nodes_explored_from_old_parent[shared_parent_id]: " + str(nodes_explored_from_old_parent[shared_parent_id]))
                    print("nodes_explored_from_new_parent[shared_parent_id]: " + str(nodes_explored_from_new_parent[shared_parent_id]))
                    if nodes_explored_from_new_parent[shared_parent_id] < nodes_explored_from_old_parent[shared_parent_id]:
                        print("-------------------------------------------------------------------------------TRUE")
                    # True if new parent has lower cost
                    return nodes_explored_from_new_parent[shared_parent_id] < nodes_explored_from_old_parent[shared_parent_id]
                
                # Update current nodes to progress up the tree
                current_node_old_parent = explore_from_old
                current_node_new_parent = explore_from_new



        for neighbor in new_node_neighbors:
            neighbor_x = self.tree[neighbor][0][0]
            neighbor_y = self.tree[neighbor][0][1]

            if self.clear_path(new_node_x, new_node_y, neighbor_x, neighbor_y):
                neighbor_parent = self.tree[neighbor][1]
                neighbor_parent_x = self.tree[neighbor_parent][0][0]
                neighbor_parent_y = self.tree[neighbor_parent][0][1]
                neighbor_cost = self.tree[neighbor][2]
                
                # True if cost of path going through new node is shorter than the path going through the neighbor's old parent
                if compare_cost_v2(neighbor, neighbor_x, neighbor_y, neighbor_parent, neighbor_parent_x, neighbor_parent_y, new_node_id, new_node_x, new_node_y):
                    if visualize:
                        rewired_edges[(neighbor, neighbor_parent)] = (neighbor, new_node_id)

                    # Update cost of neighbor
                    dist_neighbor_to_new_node = math.sqrt((neighbor_x - new_node_x)**2 + math.fabs(neighbor_y - new_node_y)**2)
                    dist_neighbor_to_neighbor_parent = math.sqrt((neighbor_x - neighbor_parent_x)**2 + math.fabs(neighbor_y - neighbor_parent_y)**2)
                    new_neighbor_cost = neighbor_cost - dist_neighbor_to_neighbor_parent + dist_neighbor_to_new_node

                    # Change parent of neighbor to new_node (remaking the whole tuple bc tuples are immutable)
                    self.tree[neighbor] = ((neighbor_x, neighbor_y), new_node_id, new_neighbor_cost)

        print(rewired_edges)
        return rewired_edges

    
    def valid_node(self, x, y, nearest_node_x, nearest_node_y):
        """
        Takes in coordinates of a potential new node. Check if the new node is
        valid (i.e. not in an obstacle and there is a clear path to the nearest
        node and in boundaries of the map). Returns True or False.
        """
        if self.in_obstacle(x, y):
            return False
        if x < 0 or x > self.map_dimensions[0] or y < 0 or y > self.map_dimensions[1]:
            return False
        if self.clear_path(x, y, nearest_node_x, nearest_node_y):
            return True
        return False

    def generate_random_next_node(self):
        """
        Select random points on map until it finds one that produces a valid
        next node. Returns the coordinates and parent ID of the new node.

        Assumes a possible node can always be found.
        """
        safety_counter = 1000
        while safety_counter > 0:
            x = random.randint(0, self.map_dimensions[0])
            y = random.randint(0, self.map_dimensions[1])

            # find nearest node
            nearest_node_id = self.get_nearest_node(x, y)
            nearest_node_x = self.tree[nearest_node_id][0][0]
            nearest_node_y = self.tree[nearest_node_id][0][1]

            x_vector = x - nearest_node_x
            y_vector = y - nearest_node_y

            # scale down to step size
            magnitude = math.sqrt((x-nearest_node_x)**2 + (y-nearest_node_y)**2)
            try:
                x_vector = x_vector / magnitude * self.step_size
                y_vector = y_vector / magnitude * self.step_size
            except ZeroDivisionError:
                continue

            x = nearest_node_x + x_vector
            y = nearest_node_y + y_vector

            if self.valid_node(x, y, nearest_node_x, nearest_node_y):
                return x, y, nearest_node_id
            
            safety_counter -= 1