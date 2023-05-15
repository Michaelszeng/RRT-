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
    def __init__(self, start, goal, map_dimensions, step_size, obstacles, goal_tolerance = 50, neighborhood_radius = 250):
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.step_size = step_size
        self.obstacles = obstacles
        self.path = []
        self.goal_tolerance = goal_tolerance
        self.neighborhood_radius = neighborhood_radius  # for RRT* rewiring step

        # Tree dictionary. keys are node ID, values are tuple of own coordinates and parent ID
        self.tree = {0: (self.start, 0)}  # start node is its own parent

    def get_nearest_node(self, input_node_x, input_node_y, id_to_ignore = []):
        """
        Find the nearest node to the node with the given ID for which there is
        a clear path to the node with the given ID. Returns the ID of the nearest
        node.
        """
        min_dist = math.inf
        nearest_node_id = None
        for node_id in self.tree:  # iterate through all nodes in tree
            if node_id != id and node_id not in id_to_ignore:  # ignore the input node itself
                dist = math.sqrt((input_node_x - self.tree[node_id][0][0])**2 + (input_node_y - self.tree[node_id][0][1])**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_node_id = node_id
        return nearest_node_id

    def add_node(self, x, y, nearest_node_id):
        """
        Assumes the input x and y are valid (i.e. not in an obstacle and
        there is a clear path to the nearest node)

        Returns True if the new node is within the goal tolerance of the goal.
        False otherwise.
        """
        new_node_id = len(self.tree)  # Create new id for node
        parent = nearest_node_id
        self.tree[new_node_id] = ((x,y), parent)
        
        rewired_edges = self.rewire(new_node_id)

        # Manhattan distance
        if math.fabs(x - self.goal[0]) + math.fabs(y - self.goal[1]) < self.goal_tolerance:
            return True, rewired_edges

        return False, rewired_edges
    
    def rewire(self, new_node_id, visualize = True):
        """
        Rewiring step of RRT*. Takes in the ID of a new node and finds all nodes
        in the new node's neighborhood. Attempts to find a shorter path to each
        of the new node's neighbors.

        Note: to reduce computation, the neighborhood is calculated using
        mahattan distance instead of euclidean distance.

        If visualize is set to True, Returns a list of tuples of node IDs that 
        were rewired. This way, the program knows which edges to "undraw" (i.e.
        draw over with in black).
        """
        def compare_cost_v2(neighbor_id, old_parent_id, new_parent_id):
            """
            Helper function to determine if rewiring is beneficial. Returns
            True if new_parent_id has lower cost path to neighbor.

            Note: new_parent_id is the id of the new node.
            """
            # We will explore up the tree from both the old parent and new parent,
            # keeping track of the total cost and sets of parent nodes explored. 
            # Once we find a common parent in the two sets, we will compare the
            # Costs and return the ID of the parent with the lower cost.

            # Manhattan distance
            neighbor_to_old_parent_cost = math.fabs(self.tree[neighbor_id][0][0] - self.tree[old_parent_id][0][0]) + math.fabs(self.tree[neighbor_id][0][1] - self.tree[old_parent_id][0][1])
            neighbor_to_new_parent_cost = math.fabs(self.tree[neighbor_id][0][0] - self.tree[new_parent_id][0][0]) + math.fabs(self.tree[neighbor_id][0][1] - self.tree[new_parent_id][0][1])

            # keys are node IDs, values are total cost to get to that node
            nodes_explored_from_old_parent = {old_parent_id: neighbor_to_old_parent_cost}
            nodes_explored_from_new_parent = {new_parent_id: neighbor_to_new_parent_cost}

            # "current" variables hold id of current node being explored
            # start from the old parent's parent and new node's parent
            current_node_old_parent = old_parent_id
            current_node_new_parent = new_parent_id
            # current_node_old_parent = self.tree[old_parent_id][1]
            # current_node_new_node = self.tree[new_parent_id][1]

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
                    # print("Intersection found!")
                    # print("nodes_explored_from_old_parent: " + str(nodes_explored_from_old_parent))
                    # print("nodes_explored_from_new_parent: " + str(nodes_explored_from_new_parent))
                    # print("intersection: " + str(intersection))
                    # print("nodes_explored_from_old_parent[shared_parent_id]: " + str(nodes_explored_from_old_parent[shared_parent_id]))
                    # print("nodes_explored_from_new_parent[shared_parent_id]: " + str(nodes_explored_from_new_parent[shared_parent_id]))
                    # True if new parent has lower cost
                    return nodes_explored_from_new_parent[shared_parent_id] < nodes_explored_from_old_parent[shared_parent_id]
                
                # Update current nodes to progress up the tree
                current_node_old_parent = explore_from_old
                current_node_new_parent = explore_from_new
        
        new_node_x = self.tree[new_node_id][0][0]
        new_node_y = self.tree[new_node_id][0][1]

        # list of tuples of node IDs that were rewired
        rewired_edges = []
        for node_id in self.tree:  # node_id is id of the potential neighbor being explored
            node_x = self.tree[node_id][0][0]
            node_y = self.tree[node_id][0][1]
            # check that found node isn't the new node, found node isn't the new node's parent, and found node is in neighborhood of new node
            if self.tree[new_node_id][1] != node_id and (math.fabs(new_node_x - node_x) + math.fabs(new_node_y - node_y)) < self.neighborhood_radius:
                if self.clear_path(new_node_x, new_node_y, node_x, node_y):
                    node_parent_id = self.tree[node_id][1]
                    # Check if new path from new node to found node is shorter than original path to found node (from found node's parent)
                    if compare_cost_v2(node_id, node_parent_id, new_node_id):  # True if path from new node to found node is shorter
                        if visualize:
                            rewired_edges.append((node_id, node_parent_id))
                        # Change parent of node (remaking the whole tuple bc tuples are immutable)
                        self.tree[node_id] = (self.tree[node_id][0], new_node_id)
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