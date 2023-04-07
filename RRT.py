"""
Helper functions and classes for RRT* algorithm
"""
import math
import random
import pygame
import numpy as np

class Window:
    def __init__(self, start, goal, map_dimensions, obstacles):
        self.width, self.height = map_dimensions
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

        # pygame settings
        pygame.display.set_caption("RRT*")
        self.window = pygame.display.set_mode((self.width, self.height))
        self.window.fill((0,0,0))
        self.node_radius = 3
        self.node_thickness = 0
        self.edge_thickness = 1

    def draw_map(self):
        # Draw start/goal nodes
        pygame.draw.circle(self.window, (255, 200, 0), self.start, self.node_radius, self.node_thickness)
        pygame.draw.circle(self.window, (0, 255, 0), self.goal, self.node_radius, self.node_thickness)
        # draw obstacles
        for obs in self.obstacles:
            pygame.draw.circle(self.window, (255, 255, 255), (obs.x, obs.y), obs.radius)


    def draw_path(self, path):
        pass

class RRTGraph:
    def __init__(self, start, goal, map_dimensions, step_size, obstacles):
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.map_dimensions = map_dimensions
        self.step_size = step_size
        self.obstacles = obstacles
        self.goal_flag = False
        self.path = []

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
        #System of equations
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

            obstacle_to_path_distance = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
            if obstacle_to_path_distance < obs.radius:
                return False
        return True

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
        """
        id = len(self.tree)  # Create new id for node
        parent = nearest_node_id
        self.tree[id] = ((x,y), parent)
        return True

    def remove_node(self, id):
        del self.tree[id]

    # def distance(self, id1, id2):
    #     # Euclidean distance formula
    #     return math.sqrt((self.tree[id1][0][0] - self.tree[id2][0][0])**2 + (self.tree[id1][0][1] - self.tree[id2][0][1])**2)
    
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

    def reached_goal(self):
        pass


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.trajectory = None

    def set_trajectory(self, trajectory):
        """
        Trajectory is stored as a dictionary of time: velocity. Only time stamps 
        where the velocity changes are stored. Missing time stamps are assumed to
        have the same velocity as the previous time stamp.
        """
        self.trajectory = trajectory

    def get_current_pos(self, time):
        """
        Get current position of obstacle
        """
        pass