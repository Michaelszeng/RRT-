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
    def __init__(self, start, goal, map_dimensions, step_size, obstacles, goal_tolerance = 20, neighborhood_radius = 50):
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_dimensions = map_dimensions
        self.step_size = step_size
        self.obstacles = obstacles
        self.goal_flag = False
        self.path = []
        self.goal_tolerance = goal_tolerance
        self.neighborhood_radius = neighborhood_radius  # for RRT* rewiring step

        # Tree dictionary. keys are node ID, values are tuple of own coordinates and parent ID
        self.tree = {0: (start, 0)}  # start node is its own parent

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

        if math.sqrt((x - self.goal[0])**2 + (y - self.goal[1])**2) < self.goal_tolerance:
            self.goal_flag = True
            return True, rewired_edges

        return False, rewired_edges
    
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

        Assumes a possible node is can always be found.
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
        print("new_node_id: " + str(new_node_id))
        def compare_cost(main_node_id, old_parent_id, new_parent_id):
            """
            Helper function to determine if rewiring is beneficial. Returns
            either old_parent_id or new_parent_id depending on which has lower
            cost.
            """
            # First step: find common parent between both paths. We explore from
            # the newest node up the tree.
            # Using sets for easier set to set comparision
            nodes_explored_from_old_parent = set()
            nodes_explored_from_new_node = set()
            # "current" variables hold id of current node being explored
            # start from the old parent's parent and new node's parent
            current_node_old_parent = self.tree[old_parent_id][1]
            current_node_new_node = self.tree[new_parent_id][1]
            while True:
                nodes_explored_from_old_parent.add(current_node_old_parent)
                nodes_explored_from_new_node.add(current_node_new_node)
                intersection = nodes_explored_from_old_parent & nodes_explored_from_new_node
                if intersection:  # if the intersection of the 2 sets is not empty
                    # find the common parent
                    common_parent_id = intersection.pop()
                    break
                current_node_old_parents_parent = self.tree[current_node_old_parent][1]
                current_node_new_nodes_parent = self.tree[current_node_new_node][1]
                current_node_old_parent = current_node_old_parents_parent
                current_node_new_node = current_node_new_nodes_parent
            print(main_node_id)
            print("common parent id: {}".format(common_parent_id))

            # Second step: calculate cost of both paths. we start from the main
            # node until we reach the to common parent.
            # cost heuristic is path length using manhattan distance
            old_cost = 0
            node1 = self.tree[main_node_id]
            node2 = self.tree[old_parent_id]
            while node1[1] != common_parent_id:
                # node 2 should always be the parent of node 1
                old_cost += math.fabs(node1[0][0] - node2[0][0]) + math.fabs(node1[0][1] - node2[0][1])
                print(node1[1])
                # time.sleep(1)
                node2 = self.tree[node2[1]]  # set node2 to its parent
                node1 = node2  # set node1 to its parent

            print("escaped")
            
            new_cost = 0
            node1 = self.tree[main_node_id]
            node2 = self.tree[new_parent_id]
            while node1[1] != common_parent_id:
                # node 2 should always be the parent of node 1
                old_cost += math.fabs(node1[0][0] - node2[0][0]) + math.fabs(node1[0][1] - node2[0][1])
                node2 = self.tree[node2[1]]  # set node2 to its parent
                node1 = node2   # set node1 to its parent

            # Third step: compare costs and return the parent with the lower cost
            if old_cost < new_cost:
                return old_parent_id
            return new_parent_id
        
        

        new_node_x = self.tree[new_node_id][0][0]
        new_node_y = self.tree[new_node_id][0][1]

        # list of tuples of node IDs that were rewired
        rewired_edges = []
        for node_id in self.tree:
            node_x = self.tree[node_id][0][0]
            node_y = self.tree[node_id][0][1]
            if node_id != new_node_id:
                # check if node is in neighborhood of new node and isn't the new node's parent
                if math.fabs(new_node_x - node_x) + math.fabs(new_node_y - node_y) < self.neighborhood_radius and self.tree[new_node_id][1] != node_id:
                    if self.clear_path(new_node_x, new_node_y, node_x, node_y):
                        node_parent_id = self.tree[node_id][1]
                        # Check if new path is shorter
                        if compare_cost(node_id, node_parent_id, new_node_id) == new_node_id:
                            if visualize:
                                rewired_edges.append((node_id, node_parent_id))
                            # Change parent of node (remaking the whole tuple bc tuples are immutable)
                            self.tree[node_id] = (self.tree[node_id][0], new_node_id)
        return rewired_edges