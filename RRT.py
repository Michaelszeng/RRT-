"""
Helper functions and classes for RRT algorithm
"""
import math
import random
import pygame
import numpy as np


class RRTGraph:
    def __init__(self, start, goal, map_dimensions, step_size, obstacles, goal_tolerance=50):
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_dimensions = map_dimensions
        self.step_size = step_size
        self.obstacles = obstacles
        self.goal_flag = False
        self.path = []
        self.goal_tolerance = goal_tolerance

        # Tree dictionary. keys are node ID, values are tuple of own coordinates and parent ID
        self.tree = {0: (start, 0)}  # start node is its own parent

    def in_obstacle(self, x, y):
        # Check if the node intersects with any obstacles
        for obs in self.obstacles:
            if math.sqrt((x - obs.x)**2 + (y - obs.y)**2) < obs.radius:
                return True
        return False

    def clear_path(self, x1, y1, x2, y2):
        """
        Check if there is a clear path between two nodes. Returns True or False.
        """
        # System of equations
        # slope of path
        try:
            m = (y1 - y2) / (x1 - x2)
        except:
            return False

        for obs in self.obstacles:
            try:
                # Calculating intersection point between path and perpendicular line to the path from obstacle
                m_perp = -1 / m  # slope of perpendicular line
                matrix = np.array([[-m, 1], [-m_perp, 1]])
                b = np.array([y1 - m * x1, obs.y - m_perp * obs.x])
                x, y = np.linalg.inv(matrix).dot(b)
            except:  # if the path is vertical (matrix should never be singular)
                print("Matrix Calculation Error")
                return False

            obstacle_to_path_distance = math.sqrt(
                (x - obs.x)**2 + (y - obs.y)**2)
            if obstacle_to_path_distance < obs.radius:
                return False
        return True

    def get_nearest_node(self, input_node_x, input_node_y, id_to_ignore=[]):
        """
        Find the nearest node to the node with the given ID for which there is
        a clear path to the node with the given ID. Returns the ID of the nearest
        node.
        """
        min_dist = math.inf
        nearest_node_id = None
        for node_id in self.tree:  # iterate through all nodes in tree
            if node_id != id and node_id not in id_to_ignore:  # ignore the input node itself
                dist = math.sqrt(
                    (input_node_x - self.tree[node_id][0][0])**2 + (input_node_y - self.tree[node_id][0][1])**2)
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
        id = len(self.tree)  # Create new id for node
        parent = nearest_node_id
        self.tree[id] = ((x, y), parent)

        if math.sqrt((x - self.goal[0])**2 + (y - self.goal[1])**2) < self.goal_tolerance:
            self.goal_flag = True
            return True
        return False

    def remove_node(self, id):
        del self.tree[id]

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
            magnitude = math.sqrt((x-nearest_node_x) **
                                  2 + (y-nearest_node_y)**2)
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

    def return_path(self):
        while True:
            path = [(self.goal[0], self.goal[1])]
            node_id = len(self.tree)-1
            while node_id is not 0:
                path.append(self.tree[node_id][0])  # (x,y) tuple
                node_id = self.tree[node_id][1]
            path.append((self.start[0], self.start[1]))
            path.reverse()
            return path
